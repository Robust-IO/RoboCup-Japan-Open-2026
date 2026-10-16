"""Offline/queued inference only. No ROS imports or robot control interfaces."""
import argparse
import json
from pathlib import Path
import time


def box_iou(a, b):
    overlap = max(0., min(a[2], b[2]) - max(a[0], b[0])) * max(0., min(a[3], b[3]) - max(a[1], b[1]))
    union = max(0., a[2]-a[0]) * max(0., a[3]-a[1]) + max(0., b[2]-b[0]) * max(0., b[3]-b[1]) - overlap
    return overlap / union if union > 0 else 0.


def classwise_keep(rows, threshold=.5):
    """Rows are xyxy, confidence, class. Keep strongest unchanged instance."""
    keep = []
    for i in sorted(range(len(rows)), key=lambda j: (-rows[j][4], j)):
        if not any(rows[i][5] == rows[j][5] and box_iou(rows[i], rows[j]) > threshold for j in keep):
            keep.append(i)
    return keep


def merge_scales(results):
    import torch
    rows, masks, origins = [], [], []
    for scale, result in results:
        if result.orig_shape != results[0][1].orig_shape:
            raise ValueError('Scale outputs have different original image shapes')
        if len(result.boxes) and (result.masks is None or tuple(result.masks.data.shape[-2:]) != tuple(result.orig_shape)):
            raise ValueError('Fusion requires full-resolution instance masks')
        for i, box in enumerate(result.boxes):
            rows.append(box.data[0].detach().cpu().tolist())
            masks.append(result.masks.data[i].detach().cpu())
            origins.append(scale)
    keep = classwise_keep(rows)
    merged = results[0][1].new()
    boxes = torch.tensor([rows[i] for i in keep], dtype=torch.float32).reshape(-1, 6)
    full_masks = torch.stack([masks[i] for i in keep]) if keep else None
    merged.update(boxes=boxes, masks=full_masks)
    merged.speed = {key: sum(r.speed.get(key, 0.) for _, r in results) for key in ['preprocess','inference','postprocess']}
    conflicts = [[a,b] for a in range(len(keep)) for b in range(a+1,len(keep))
                 if rows[keep[a]][5] != rows[keep[b]][5] and box_iou(rows[keep[a]],rows[keep[b]]) > .5]
    return merged, [origins[i] for i in keep], conflicts


def validate_registry(names):
    expected = ['apple', 'canned_juice', 'rabbit_doll', 'pink_cup', 'white_cup']
    actual = [names[i] for i in range(len(names))]
    if actual not in (expected, expected + ['filled_ketchup']):
        raise ValueError('Unexpected model class registry')
    return [0, 3, 15, 14, 26, 9][:len(actual)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--seconds', type=int, default=1800)
    parser.add_argument('--imgsz', type=int, choices=[640, 1280], default=640,
                        help='Inference input size; does not change camera resolution')
    parser.add_argument('--multi-scale', action='store_true', help='Run 640 and 1280; same-class NMS IoU 0.5, preserve selected mask')
    args = parser.parse_args()
    if not args.weights.is_file():
        raise SystemExit('Weights missing')
    args.output.mkdir(parents=True, exist_ok=False)
    from ultralytics import YOLO
    import numpy as np
    model = YOLO(str(args.weights))
    global_ids = validate_registry(model.names)
    scales = [640,1280] if args.multi_scale else [args.imgsz]
    for size in scales:
        model.predict(np.zeros((480, 640, 3), np.uint8), device=0, imgsz=size, retina_masks=True, verbose=False)
    print(f'READY: model warmed up, conf=0.25, scales={scales}; output:', args.output, flush=True)
    seen = set()
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        paths = [args.input] if args.input.is_file() else sorted(args.input.glob('[0-9]*.json'))
        for path in paths:
            if path.name in seen:
                continue
            if path.suffix == '.json':
                metadata = json.loads(path.read_text())
                source = path.parent / metadata['image']
            else:
                source = path
            started = time.perf_counter()
            results = [(size, model.predict(str(source), device=0, conf=.25, imgsz=size, retina_masks=True, verbose=False)[0]) for size in scales]
            if args.multi_scale:
                result, origins, conflicts = merge_scales(results)
            else:
                result = results[0][1]
                origins, conflicts = [args.imgsz]*len(result.boxes), []
            predict_merge_ms = (time.perf_counter()-started)*1000
            stem = path.stem
            detections = []
            for i, box in enumerate(result.boxes):
                cls = int(box.cls.item())
                detections.append({'class_id': cls, 'global_class_id': global_ids[cls], 'name': model.names[cls], 'confidence': float(box.conf.item()), 'source_imgsz': origins[i],
                    'xyxy': box.xyxy[0].tolist(),
                    'contour_xy': result.masks.xy[i].tolist() if result.masks is not None else None})
            if result.masks is not None:
                # Exact predicted masks, not just contour approximations.
                np.savez_compressed(args.output / (stem + '.masks.npz'), masks=result.masks.data.cpu().numpy())
            result.save(filename=str(args.output / (stem + '.jpg')))
            (args.output / (stem + '.json')).write_text(json.dumps({'source': str(source), 'weights': str(args.weights),
                'threshold': .25, 'imgsz': scales if args.multi_scale else args.imgsz, 'fusion': 'same-class-nms-0.5' if args.multi_scale else None,
                'class_conflicts': conflicts, 'predict_merge_ms': predict_merge_ms,
                'detections': detections, 'speed_ms': result.speed}, indent=2))
            print(stem, [(x['name'], round(x['confidence'], 3)) for x in detections], flush=True)
            seen.add(path.name)
        if args.once:
            break
        time.sleep(.5)


if __name__ == '__main__':
    main()
