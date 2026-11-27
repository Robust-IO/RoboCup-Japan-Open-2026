"""Explicit active-model entry point for offline image checks; no robot control."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('active_detection_model.json'))
    p.add_argument('--source', type=Path, required=True)
    args = p.parse_args()
    cfg = json.loads(args.config.read_text())
    weights = Path(cfg['weights'])
    if hashlib.sha256(weights.read_bytes()).hexdigest() != cfg['sha256']:
        raise ValueError('Active model hash mismatch')
    from ultralytics import YOLO
    model = YOLO(str(weights))
    if len(model.names) != 30:
        raise ValueError('Expected 30 detection classes')
    r = model.predict(str(args.source), imgsz=cfg['imgsz'], conf=cfg['conf'],
                      device=cfg['device'], verbose=False)[0]
    print(json.dumps(dict(weights=str(weights), sha256=cfg['sha256'], classes=model.names,
                         detections=[dict(name=r.names[int(b.cls.item())], confidence=float(b.conf.item()),
                                          xyxy=b.xyxy[0].tolist()) for b in r.boxes])))


if __name__ == '__main__':
    main()
