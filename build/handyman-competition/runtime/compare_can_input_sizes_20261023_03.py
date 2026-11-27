"""Saved-frame full-image model comparison; no ROS and no image crops."""
import contextlib
import hashlib
import json
from pathlib import Path
import sys

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');registry=json.loads((base/'active_detection_model.json').read_text())
weights=Path(registry['weights'])
if hashlib.sha256(weights.read_bytes()).hexdigest()!=registry['sha256']:raise ValueError('model_hash_mismatch')
with contextlib.redirect_stdout(sys.stderr):
    import cv2
    import numpy as np
    from ultralytics import YOLO
    model=YOLO(str(weights));rgb=cv2.imread(str(base/'can-auto-live-20261023-03/grasp/captures/identity-00/rgb.png'))
    results=[]
    for size in (640,960,1280):
        for _ in range(2):model.predict(np.zeros_like(rgb),imgsz=size,conf=.1,device=registry['device'],verbose=False)
        p=model.predict(rgb,imgsz=size,conf=.1,device=registry['device'],verbose=False)[0]
        detections=[dict(name=model.names[int(c)],confidence=float(conf),box=box) for box,c,conf in
            zip(p.boxes.xyxy.cpu().tolist(),p.boxes.cls.cpu().tolist(),p.boxes.conf.cpu().tolist())]
        results.append(dict(imgsz=size,detections=detections))
value=dict(model_sha256=registry['sha256'],same_original_full_frame=True,results=results,robot_commands_sent=False)
with (base/'can-auto-03-identity-audit/full-image-input-size-comparison.json').open('x') as f:json.dump(value,f,indent=2)
print(json.dumps(value),flush=True)
