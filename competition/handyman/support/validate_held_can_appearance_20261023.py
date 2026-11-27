"""Replay only actual saved RGBD; never initializes ROS or sends commands."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
args=p.parse_args();sys.path.insert(0,str(args.candidate))
import cv2
import numpy as np
from held_can_appearance import build_appearance,assess

base=Path('/__HANDYMAN_PACKAGE__/runtime');root=args.output;root.mkdir(exist_ok=False,parents=True)
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
carry=base/'canned-carry-live-20261023-01'
before=read(carry/'before-retract.json');source=Path(before['rows'][-1]['capture'])
names=('rgb.png','depth_m.npy','snapshot.json','can-measurement.json')
source_hashes={name:sha(source/name) for name in names}
rgb=lambda path:cv2.cvtColor(cv2.imread(str(path/'rgb.png')),cv2.COLOR_BGR2RGB)
appearance=build_appearance(rgb(source),np.load(source/'depth_m.npy',allow_pickle=False),
    read(source/'snapshot.json'),read(source/'can-measurement.json'))
ref=read(base/'canned-grasp-lift-live-progress-20261023.json')
paths=sorted((carry/'captures').glob('after-retract-rejected-*'))
paths+=sorted((base/'tucked-can-rgb-probe-20261023-01/captures').glob('frame-*'))
if len(paths)!=46:raise ValueError('expected_all_43_post_retract_and_3_fresh_probe_frames')
rows=[]
for path in paths:
    evidence=assess(rgb(path),np.load(path/'depth_m.npy',allow_pickle=False),read(path/'snapshot.json'),ref,appearance)
    if not evidence['current_rgb_appearance_verified'] or evidence['measured_3d_object_position_available']:
        raise ValueError('incorrect_rgb_evidence_semantics')
    rows.append(dict(capture=str(path),evidence=evidence))
if any(sha(source/name)!=digest for name,digest in source_hashes.items()):raise ValueError('source_changed')
result=dict(status='all_recorded_tucked_frames_have_body_and_cap',frames=len(rows),appearance=appearance,
    source=str(source),source_sha256=source_hashes,rows=rows,robot_commands_sent=False,
    new_object_identity_verified=False,current_3d_reconstruction_verified=False)
(root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
