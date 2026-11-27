"""Recorded same-image model diagnosis only. No identity or motion authority."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
from ultralytics import YOLO

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
from apple_inference_worker import measured_proposal,confirm_near_grasp
capture=base/'apple-r11b-full-stop-state-20261018-01/000005'
registry=json.loads((base/'active_detection_model.json').read_text())
weights=Path(registry['weights'])
if hashlib.sha256(weights.read_bytes()).hexdigest()!=registry['sha256']:raise ValueError('model_hash_mismatch')
model=YOLO(str(weights));rows=[]
snapshot=json.loads((capture/'snapshot.json').read_text())
rgb=np.asarray(Image.open(capture/'rgb.png').convert('RGB'))
depth=np.load(capture/'depth_m.npy',allow_pickle=False)
for size in (640,960,1280):
    prediction=model.predict(str(capture/'rgb.png'),imgsz=size,conf=.1,device=registry['device'],verbose=False)[0]
    objects=[dict(name=model.names[int(c)],confidence=float(score),box=box)
        for box,c,score in zip(prediction.boxes.xyxy.cpu().tolist(),prediction.boxes.cls.cpu().tolist(),prediction.boxes.conf.cpu().tolist())]
    rows.append(dict(mode='full_frame',imgsz=size,detections=objects))
    if size==1280:
        proposals=[measured_proposal(rgb,depth,snapshot,d) for d in objects if d['name']=='apple']
        rows[-1].update(proposals=proposals,actual_bound_roi_verification=
            confirm_near_grasp(model,rgb,snapshot,proposals,objects,registry['sha256'],registry['device']))
rgb=np.asarray(Image.open(capture/'rgb.png').convert('RGB'))
# Operator-chosen diagnostic context, NOT a geometry proposal or live identity.
roi=[125,160,445,480];x,y,right,bottom=roi
prediction=model.predict(rgb[y:bottom,x:right,::-1].copy(),imgsz=640,conf=.1,device=registry['device'],verbose=False)[0]
objects=[dict(name=model.names[int(c)],confidence=float(score),box=[box[0]+x,box[1]+y,box[2]+x,box[3]+y])
    for box,c,score in zip(prediction.boxes.xyxy.cpu().tolist(),prediction.boxes.cls.cpu().tolist(),prediction.boxes.conf.cpu().tolist())]
rows.append(dict(mode='operator_selected_diagnostic_crop_ONLY',roi=roi,imgsz=640,detections=objects))
report=dict(capture=str(capture),model_sha256=registry['sha256'],rows=rows,
    robot_commands_sent=False,live_identity_authorized=False,model_or_threshold_changed=False)
with (base/'apple-r11b-stop-model-diagnosis-20261018-02.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report),flush=True)
