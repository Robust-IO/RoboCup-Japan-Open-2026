"""Offline retained-frame ROI inference experiment. Not a live motion lease."""
import contextlib
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
from PIL import Image

root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/automatic-apple-20261017-05')
out=root/'offline-roi-inference-audit.json'
if out.exists():raise ValueError('output_exists')
registry=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/active_detection_model.json').read_text())
weights=Path(registry['weights'])
if hashlib.sha256(weights.read_bytes()).hexdigest()!=registry['sha256']:raise ValueError('model_hash_mismatch')
with contextlib.redirect_stdout(sys.stderr):
    from ultralytics import YOLO
    model=YOLO(str(weights))
rgb=np.asarray(Image.open(root/'source-observe/000126/rgb.png').convert('RGB'))
observation=json.loads((root/'source-observe/000126/observation.json').read_text())
box=next(d['box'] for d in observation['detections'] if d['name']=='apple')
cx=(box[0]+box[2])/2;cy=(box[1]+box[3])/2
rows=[]
for size in (None,160,224,320,448):
    if size is None:x=y=0;right=rgb.shape[1];bottom=rgb.shape[0]
    else:
        x=int(max(0,min(rgb.shape[1]-size,cx-size/2)))
        y=int(max(0,min(rgb.shape[0]-size,cy-size/2)))
        right=x+size;bottom=y+size
    # Ultralytics ndarray input is BGR, unlike the stored PIL RGB.
    crop=rgb[y:bottom,x:right,::-1].copy()
    for imgsz in (640,960):
        start=time.monotonic()
        with contextlib.redirect_stdout(sys.stderr):
            prediction=model.predict(crop,imgsz=imgsz,conf=.1,device=registry['device'],verbose=False)[0]
        detections=[dict(name=model.names[int(c)],confidence=float(conf),
                         box=[b[0]+x,b[1]+y,b[2]+x,b[3]+y])
                    for b,c,conf in zip(prediction.boxes.xyxy.cpu().tolist(),prediction.boxes.cls.cpu().tolist(),prediction.boxes.conf.cpu().tolist())]
        row=dict(crop_size=size,imgsz=imgsz,roi=[x,y,right,bottom],detections=detections,
                 elapsed_s=time.monotonic()-start)
        rows.append(row);print(json.dumps(row),flush=True)
with out.open('x') as f:
    json.dump(dict(source_capture=str(root/'source-observe/000126'),rows=rows,
                   model_sha256=registry['sha256'],offline_only=True,robot_commands_sent=False,
                   production_model_or_threshold_changed=False,
                   caveat='Retained failed frame; exploratory diagnostic only, not live identity confirmation or dataset-wide validation'),f,indent=2)
