"""Recorded apple numeric replay and old can RGB recognition; no robot commands."""
import argparse
import contextlib
import hashlib
import json
import os
from pathlib import Path
import sys

parser=argparse.ArgumentParser();parser.add_argument('--candidate',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();args.output.mkdir(exist_ok=False)
sys.path.insert(0,str(args.candidate))
import numpy as np
from PIL import Image
from apple_rgbd_candidate import estimate as legacy_estimate
from object_surface_geometry import estimate
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
registry=base/'active_detection_model.json';config=json.loads(registry.read_text());registry_hash=sha(registry)
if sha(Path(config['weights']))!=config['sha256']:raise ValueError('model_hash_mismatch')
trial=base/'automatic-apple-r19-20261022-01'
capture=Path(json.loads((trial/'measured-grasp-plan.json').read_text())['capture'])
observation=json.loads((capture/'observation.json').read_text());snapshot=json.loads((capture/'snapshot.json').read_text())
source_hashes={n:sha(capture/n) for n in ('rgb.png','depth_m.npy','snapshot.json','observation.json')}
rgb=np.asarray(Image.open(capture/'rgb.png').convert('RGB'));depth=np.load(capture/'depth_m.npy',allow_pickle=False)
apples=[d for d in observation['detections'] if d['name']=='apple' and d['confidence']>=.8]
if len(apples)!=1:raise ValueError('reviewed_source_apple_frame_required')
old=legacy_estimate(rgb,depth,snapshot,apples[0],geometry_proposal_only=True)
wrapped=estimate(rgb,depth,snapshot,apples[0],'apple')
if old!=wrapped['legacy_candidate']:raise ValueError('apple_numeric_measurement_changed')
os.environ['YOLO_CONFIG_DIR']=str(base/'generic-object-yolo-settings')
with contextlib.redirect_stdout(sys.stderr):
    from ultralytics import YOLO
    model=YOLO(config['weights'])
if model.task!='detect' or len(model.names)!=30 or model.names[3]!='canned_juice' or model.names[0]!='apple':
    raise ValueError('model_class_mapping_does_not_match_profiles')
rows=[]
for frame in ('000001','000051'):
    source=base/'live30-20261003/canned-juice01'/(frame+'.png');before=sha(source)
    with contextlib.redirect_stdout(sys.stderr):
        prediction=model.predict(str(source),imgsz=960,conf=.1,device=config['device'],verbose=False)[0]
    detections=[dict(name=model.names[int(c)],class_id=int(c),confidence=float(score),box=box)
                for box,c,score in zip(prediction.boxes.xyxy.cpu().tolist(),prediction.boxes.cls.cpu().tolist(),prediction.boxes.conf.cpu().tolist())]
    targets=[d for d in detections if d['name']=='canned_juice']
    prediction.save(filename=str(args.output/('can-'+frame+'-current-model-960.jpg')))
    if sha(source)!=before:raise ValueError('source_image_modified')
    rows.append(dict(source=str(source),source_sha256=before,detections=detections,
        target_detections=len(targets),maximum_target_score=max((d['confidence'] for d in targets),default=None),
        current_rgb_recognition_passed=len(targets)==1 and targets[0]['confidence']>=.8,
        rgbd_geometry_validated=False,reason='Historical RGB only: no depth or new physical transfer evidence.'))
if sha(registry)!=registry_hash or sha(Path(config['weights']))!=config['sha256']:raise ValueError('active_model_changed')
if any(sha(capture/n)!=d for n,d in source_hashes.items()):raise ValueError('apple_recording_modified')
report=dict(status='recorded_apple_numeric_replay_and_can_rgb_checks_complete',
    apple_legacy_numeric_result_identical=True,apple_capture=str(capture),apple_center_m=old['center_m'],
    model_sha256=config['sha256'],imgsz=960,canned_rgb_checks=rows,
    can_real_rgbd_validation=False,can_live_transfer_verified=False,robot_commands_sent=False,
    source_recordings_unchanged=True,model_registry_and_weights_unchanged=True)
(args.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
