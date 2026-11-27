"""Frozen-model pose diagnostic. Geometry boxes are proxies, not final human GT."""
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from PIL import Image
from ultralytics import YOLO


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def iou(a, b):
    area = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - area
    return area / union if union > 0 else 0


def main():
    root = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/geometry/pilot-pose6-01')
    weights = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/handyman30-salt-pepper-scene-ft-v1/weights/best.pt')
    expected_hash = '3d822e2fa3f13a2a237ca3be8ecae91f929991a279a139de2d69dd99964f594f'
    if digest(weights) != expected_hash:
        raise RuntimeError('Baseline changed; refusing comparison')
    batch = json.loads((root / 'batch.json').read_text())
    if batch['count'] != 72 or not batch['poseDiagnostic']:
        raise RuntimeError('Wrong dataset')
    out = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/pose6-baseline-20261005-v1')
    out.mkdir(exist_ok=False)
    model = YOLO(str(weights))
    rows = []
    for size in (640, 1280):
        for index in range(72):
            frame = json.loads((root / 'frames' / f'{index:06d}.json').read_text())
            item = frame['instances'][0]
            if model.names[item['globalClassId']] != item['className']:
                raise RuntimeError('Class mapping differs')
            source = root / frame['rgb']
            with Image.open(root / frame['geometryMask']) as im:
                mask = np.asarray(im.convert('RGB'))
            ys, xs = np.where(np.all(mask == (255, 0, 0), axis=2))
            if not len(xs):
                raise RuntimeError('Empty geometry reference')
            box = [int(xs.min()), int(ys.min()), int(xs.max())+1, int(ys.max())+1]
            pred = model.predict(str(source), imgsz=size, conf=.1, iou=.7, device=0, verbose=False)[0]
            detections = [dict(name=pred.names[int(b.cls.item())], confidence=float(b.conf.item()),
                               xyxy=b.xyxy[0].tolist(), reference_iou=iou(b.xyxy[0].tolist(), box)) for b in pred.boxes]
            correct = [d for d in detections if d['name'] == item['className'] and d['reference_iou'] >= .5]
            wrong = [d for d in detections if d['name'] != item['className'] and d['reference_iou'] >= .3]
            row = dict(index=index, imgsz=size, target=item['className'], pose=frame['poseName'],
                       requested_pixels=frame['requestedBoundsShortSidePixels'], source=str(source),
                       source_sha256=digest(source), geometry_box=box, detections=detections,
                       matched=bool(correct), best_correct_conf=max((d['confidence'] for d in correct), default=None),
                       overlapping_wrong=wrong, off_target_detections=[d for d in detections if d['reference_iou'] < .1])
            rows.append(row)
            with (out / 'predictions.jsonl').open('a') as f:
                f.write(json.dumps(row) + '\n')
            if (index+1) % 12 == 0:
                print(f'imgsz={size}: {index+1}/72', flush=True)
    groups = defaultdict(list)
    for r in rows:
        groups[(r['imgsz'], r['target'])].append(r)
    summary = [dict(imgsz=size, target=target, frames=len(rs), matched=sum(r['matched'] for r in rs),
                    frames_with_overlapping_wrong=sum(bool(r['overlapping_wrong']) for r in rs),
                    frames_with_off_target_detections=sum(bool(r['off_target_detections']) for r in rs))
               for (size, target), rs in groups.items()]
    report = dict(weights=str(weights), weights_sha256=expected_hash, batch_sha256=digest(root/'batch.json'),
                  conf=.1, correct_iou=.5, wrong_overlap_iou=.3, off_target_iou=.1, summary=summary,
                  limitations=['Geometry-mask boxes are diagnostic proxies, not fully human-reviewed ground truth.',
                               'No mAP or competition success claim; only six classes, one synthetic background, single objects.',
                               'Low-confidence detections retained to inspect errors; no threshold tuning or training performed.',
                               '1280 resizes the same 640x480 source; does not add captured detail.'])
    (out/'summary.json').write_text(json.dumps(report, indent=2))
    with (out/'cases.csv').open('w', newline='') as f:
        fields = ['imgsz','index','target','pose','requested_pixels','matched','best_correct_conf','wrong_classes','off_target_count']
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
        for r in rows:
            writer.writerow({**{k:r[k] for k in fields[:7]},
                             'wrong_classes': ';'.join(d['name'] for d in r['overlapping_wrong']),
                             'off_target_count':len(r['off_target_detections'])})
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
