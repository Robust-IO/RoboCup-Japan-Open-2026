"""Read-only full-frame640 check on ALL prior source confirmations."""
import contextlib
import hashlib
import json
from pathlib import Path
import sys

base=Path('/__HANDYMAN_PACKAGE__/runtime');read=lambda p:json.loads(p.read_text());registry=read(base/'active_detection_model.json')
if hashlib.sha256(Path(registry['weights']).read_bytes()).hexdigest()!=registry['sha256']:raise ValueError('model_hash_mismatch')
paths=[]
for case in ('can-auto-live-20261023-01','can-auto-live-20261023-02'):
    r=read(base/case/'grasp/result.json');paths.extend(Path(v['capture']) for v in r['initial_observations'])
paths.append(base/'can-auto-live-20261023-03/grasp/captures/identity-00')
with contextlib.redirect_stdout(sys.stderr):
    import cv2
    import numpy as np
    from ultralytics import YOLO
    model=YOLO(registry['weights'])
    for _ in range(3):model.predict(np.zeros((480,640,3),np.uint8),imgsz=640,conf=.1,device=registry['device'],verbose=False)
    rows=[]
    for path in paths:
        image=cv2.imread(str(path/'rgb.png'));result=model.predict(image,imgsz=640,conf=.1,device=registry['device'],verbose=False)[0]
        cans=[dict(confidence=float(score),box=box) for c,score,box in zip(result.boxes.cls.cpu().tolist(),result.boxes.conf.cpu().tolist(),result.boxes.xyxy.cpu().tolist()) if model.names[int(c)]=='canned_juice']
        rows.append(dict(capture=str(path),cans=cans,highest_confidence=max((v['confidence'] for v in cans),default=0)))
value=dict(status='historical_source640_replayed',model_sha256=registry['sha256'],imgsz=640,rows=rows,
    all_sources_have_high_confirmed_can=all(v['highest_confidence']>=.8 for v in rows),robot_commands_sent=False)
with (base/'can-auto-03-identity-audit/historical640-source-regression.json').open('x') as f:json.dump(value,f,indent=2)
print(json.dumps(value),flush=True)
