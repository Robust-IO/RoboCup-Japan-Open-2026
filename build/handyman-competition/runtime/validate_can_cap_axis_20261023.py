"""Actual failed-run replay plus prior successful captures; no robot actions."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

p=argparse.ArgumentParser();p.add_argument('--code',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
sys.path.insert(0,str(args.code))
import cv2
import numpy as np
from canned_grasp_geometry import measure_near,lift_follow,transform_point
from held_can_appearance import build_appearance
from apple_transfer_automation import write_json

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=args.output;root.mkdir(parents=True,exist_ok=False)
read=lambda path:json.loads(path.read_text());sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
results=[]
for name,pattern in [('can-auto-live-20261023-01/grasp','after-lift-rejected-*'),
                     ('canned-close-lift-continuation-20261023-01','after-lift-rejected-*')]:
    trial=base/name;report=read(trial/'result.json');original_sha=sha(trial/'result.json')
    before=trial/'captures/preclose';s0=read(before/'snapshot.json');depth=np.load(before/'depth_m.npy',allow_pickle=False)
    if (before/'can-measurement.json').is_file():m0=read(before/'can-measurement.json')
    else:m0=read(base/'canned-grasp-lift-live-progress-20261023.json')['recorded_replay']['before_measurement']
    appearance=build_appearance(cv2.cvtColor(cv2.imread(str(before/'rgb.png')),cv2.COLOR_BGR2RGB),depth,s0,m0)
    close=report['close'];holding=close['closure_evidence']['observed_opening_rad']
    c0=transform_point(s0,m0['center_m']);p0=transform_point(s0,s0['palm_base_at_depth'])
    rows=[]
    for path in sorted((trial/'captures').glob(pattern)):
        snap=read(path/'snapshot.json');p1=transform_point(snap,snap['palm_base_at_depth']);expected=transform_point(snap,c0+p1-p0,inverse=True)
        m=measure_near(np.load(path/'depth_m.npy',allow_pickle=False),snap,expected,m0['footprint_radius_m'],m0['height_m'],
            rgb=cv2.cvtColor(cv2.imread(str(path/'rgb.png')),cv2.COLOR_BGR2RGB),appearance=appearance)
        proof=lift_follow(m0,s0,m,snap,holding)
        rows.append(dict(capture=str(path),measurement=m,proof=proof))
    if sha(trial/'result.json')!=original_sha:raise ValueError('original_failed_result_changed')
    if len(rows)<3:raise ValueError('actual_recorded_frames_missing')
    results.append(dict(trial=name,frames=len(rows),rows=rows,
        tilt_range_deg=[min(r['measurement']['tilt_deg'] for r in rows),max(r['measurement']['tilt_deg'] for r in rows)],
        maximum_follow_error_m=max(r['proof']['object_palm_follow_error_m'] for r in rows),original_result_sha256=original_sha))
value=dict(status='actual_tilted_and_prior_upright_lift_replay_passed',trials=results,
    robot_commands_sent=False,original_results_preserved=True,radius_limits_relaxed=False,new_uninterrupted_trial_verified=False)
write_json(root/'result.json',value)
print(json.dumps(dict(status=value['status'],trials=[{k:v for k,v in r.items() if k!='rows'} for r in results],robot_commands_sent=False)),flush=True)
