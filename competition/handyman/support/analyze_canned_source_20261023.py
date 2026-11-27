"""Saved-current-burst classifier/shape diagnostics. Never controls the robot."""
import contextlib
import hashlib
import json
import os
from pathlib import Path
import sys
import time

BASE=Path('/__HANDYMAN_PACKAGE__/runtime');CODE=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
SOURCE=BASE/'canned-source-observation-20261023-01'
OUT=BASE/'canned-source-analysis-20261023-01'
OUT.mkdir(exist_ok=False)
sys.path.insert(0,str(CODE))
from inspect_manipulation_capture import inspect
from apple_transfer_auto_plan import map_point
from scripts.search_room_gate import inside
import numpy as np
import yaml

sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
reg=BASE/'active_detection_model.json';cfg=json.loads(reg.read_text());reg_before=sha(reg)
if sha(Path(cfg['weights']))!=cfg['sha256']:raise ValueError('model_hash_mismatch')
os.environ['YOLO_CONFIG_DIR']=str(BASE/'generic-object-yolo-settings')
with contextlib.redirect_stdout(sys.stderr):
    from ultralytics import YOLO
    model=YOLO(cfg['weights'])
if len(model.names)!=30 or model.names[3]!='canned_juice':raise ValueError('class_mapping_mismatch')
environment=yaml.safe_load(Path('/__HANDYMAN_PACKAGE__/project/src/handyman_rebuild_ros2/config/environments/layout_a.yaml').read_text())
frames=sorted((SOURCE/'cameras').glob('*/snapshot.json'))
if len(frames)!=3:raise ValueError('completed_source_camera_burst_required')
rows=[];centers=[]
for f in frames:
    capture=f.parent;hashes={n:sha(capture/n) for n in ('rgb.png','depth_m.npy','snapshot.json')}
    with contextlib.redirect_stdout(sys.stderr):
        prediction=model.predict(str(capture/'rgb.png'),imgsz=960,conf=.1,device=cfg['device'],verbose=False)[0]
    detections=[dict(name=model.names[int(c)],class_id=int(c),confidence=float(score),box=box)
        for box,c,score in zip(prediction.boxes.xyxy.cpu().tolist(),prediction.boxes.cls.cpu().tolist(),prediction.boxes.conf.cpu().tolist())]
    prediction.save(filename=str(OUT/(capture.name+'-detected.jpg')))
    snapshot=json.loads(f.read_text());result=inspect(capture,detections,'canned_juice')
    result.update(detections=detections,analysis_time_ns=time.time_ns(),
        rgbd_age_at_analysis_s=(time.time_ns()-min(snapshot['streams'][k]['stamp_ns'] for k in ('rgb','depth')))/1e9,
        not_a_live_motion_lease=True)
    for item in result['geometry_candidates']:
        geometry=item['geometry'];mapped=map_point(snapshot,np.asarray(geometry['center_m'],float))
        rooms=[name for name,room in environment['rooms'].items() if inside(*mapped[:2],room['region'])]
        item.update(center_map_m=mapped.tolist(),room_membership=rooms)
        centers.append(mapped.tolist())
    if any(sha(capture/n)!=v for n,v in hashes.items()):raise ValueError('capture_modified')
    (OUT/(capture.name+'-analysis.json')).write_text(json.dumps(result,indent=2))
    rows.append(dict(frame=capture.name,capture=str(capture),
        target_detection_count=result['classification_detection_count'],geometry_candidate_count=len(result['geometry_candidates']),
        geometry_candidates=result['geometry_candidates'],rejections=result['rejections'],
        target_max_confidence=max((d['confidence'] for d in detections if d['name']=='canned_juice'),default=None)))
if sha(reg)!=reg_before or sha(Path(cfg['weights']))!=cfg['sha256']:raise ValueError('model_changed')
spread=float(np.max(np.linalg.norm(np.asarray(centers)-np.mean(centers,axis=0),axis=1))) if len(centers)==3 else None
report=dict(status='saved_current_rgbd_burst_analyzed',class_name='canned_juice',rows=rows,
    center_spread_radius_m=spread,all_three_frames_have_one_fit=all(r['geometry_candidate_count']==1 for r in rows),
    source_recordings_unchanged=True,model_sha256=cfg['sha256'],imgsz=960,robot_commands_sent=False,
    fresh_motion_authorized=False,physical_grasp_or_transfer_verified=False)
(OUT/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
