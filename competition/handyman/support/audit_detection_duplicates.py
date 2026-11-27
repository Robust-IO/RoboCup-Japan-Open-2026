"""Read-only inference audit; overlapping boxes are suspects, not ground truth."""
import argparse
import collections
import json
from itertools import combinations
from pathlib import Path


def iou(a, b):
    intersection = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection
    return intersection / union if union > 0 else 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument('report', type=Path)
    p.add_argument('--inspect-model', type=Path)
    args = p.parse_args()
    report = json.loads(args.report.read_text())
    groups = {}
    pairs = []
    for case in report['cases']:
        for result in case['results']:
            key = f"{result['model']}-{result['imgsz']}"
            count = groups.setdefault(key, collections.Counter())
            count['frames'] += 1
            count['boxes'] += len(result['detections'])
            flags = set()
            for a, b in combinations(result['detections'], 2):
                overlap = iou(a['xyxy'], b['xyxy'])
                if overlap < .7:
                    continue
                kind = 'same_class' if a['name'] == b['name'] else 'cross_class'
                count[kind + '_pairs'] += 1
                flags.add(kind)
                pairs.append(dict(case=case['id'], run=key, kind=kind, iou=overlap, a=a, b=b))
            for kind in flags:
                count[kind + '_frames'] += 1
    output = dict(groups=groups, suspect_pairs=pairs,
                  note='Per-image, per-model, per-size only. IoU>=0.7 is a suspect, not confirmed duplication. No filtering or deployment.')
    if args.inspect_model:
        import ultralytics
        from ultralytics import YOLO
        model = YOLO(str(args.inspect_model))
        output['implementation'] = dict(version=ultralytics.__version__,
            end2end=bool(model.model.end2end), head=type(model.model.model[-1]).__name__)
        case = next(c for c in report['cases'] if c['id'] == 'case-019')
        comparisons = {}
        for threshold in [.1, .7]:
            prediction = model.predict(case['source'], imgsz=640, conf=.1,
                                       iou=threshold, device=0, verbose=False)[0]
            comparisons[str(threshold)] = prediction.boxes.data.cpu().tolist()
        output['iou_parameter_test'] = dict(case=case['id'], predictions=comparisons,
            identical=comparisons['0.1'] == comparisons['0.7'])
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
