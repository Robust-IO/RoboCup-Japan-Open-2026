"""Independent recorded depth verification; keeps original failed verdicts."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

parser=argparse.ArgumentParser();parser.add_argument('--candidate',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();args.output.mkdir(exist_ok=False)
BASE=Path('/__HANDYMAN_PACKAGE__/runtime');sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
with (args.output/'legacy-and-original-rgbd.log').open('x') as stream:
    subprocess.run([sys.executable,str(BASE/'validate_canned_depth_replay_20261023.py'),
        '--candidate',str(args.candidate),'--output',str(args.output/'legacy-and-original-rgbd')],
        stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=90)
sys.path.insert(0,str(args.candidate))
from canned_grasp_geometry import measure_near,predicted_before_contact,transform_point,lift_follow
from run_apple_close_lift_trial import lift_allowed

trial=BASE/'canned-close-lift-continuation-20261023-01';original=trial/'result.json';prior=json.loads(original.read_text())
if not lift_allowed(prior.get('close',{})) or prior.get('lift',{}).get('status')!='passed':
    raise ValueError('actual_contact_and_lift_endpoint_required')
source=Path(prior['predecessor']);reference=json.loads((source/'locked-can-reference.json').read_text())
before=Path(prior['close']['before_capture']);bs=json.loads((before/'snapshot.json').read_text())
inputs={original:sha(original),source/'locked-can-reference.json':sha(source/'locked-can-reference.json')}
def remember(cap):
    for name in ('rgb.png','depth_m.npy','snapshot.json'):inputs[cap/name]=sha(cap/name)
remember(before)
b=measure_near(np.load(before/'depth_m.npy',allow_pickle=False),bs,
    predicted_before_contact(reference,bs,reference['task_id']),reference['radius_m'],reference['height_m'])
c0=transform_point(bs,b['center_m']);p0=transform_point(bs,bs['palm_base_at_depth'])
opening=prior['close']['closure_evidence']['observed_opening_rad'];rows=[]
for path in sorted((trial/'captures').glob('after-lift-rejected-*')):
    remember(path);s=json.loads((path/'snapshot.json').read_text())
    p1=transform_point(s,s['palm_base_at_depth']);expected=transform_point(s,c0+p1-p0,inverse=True)
    measured=measure_near(np.load(path/'depth_m.npy',allow_pickle=False),s,expected,reference['radius_m'],reference['height_m'])
    proof=lift_follow(b,bs,measured,s,opening)
    rows.append(dict(proof,capture=str(path),measurement=measured))
if len(rows)<3 or (rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9<.4:
    raise ValueError('multiple_distinct_actual_postlift_frames_required')
relative=np.array([row['center_minus_palm_odom_m'] for row in rows])
spread=float(np.max(np.linalg.norm(relative-relative.mean(0),axis=1)))
if spread>.02:raise ValueError('held_can_not_stable_with_palm')
if any(sha(p)!=digest for p,digest in inputs.items()):raise ValueError('original_recordings_modified')
report=dict(status='independent_actual_can_grasp_lift_verified',
    can_recorded_lift_follow_verified=True,can_three_actual_rgbd_frames_validated=True,
    actual_postlift_frames=len(rows),source_trial=str(trial),before_capture=str(before),
    before_measurement=b,held_opening_rad=opening,rows=rows,center_palm_relative_spread_m=spread,
    maximum_object_palm_follow_error_m=max(row['object_palm_follow_error_m'] for row in rows),
    minimum_object_rise_m=min(row['object_delta_odom_m'][2] for row in rows),
    maximum_object_rise_m=max(row['object_delta_odom_m'][2] for row in rows),
    original_failed_verdict_and_rgbd_retained=True,source_sha256={str(p):digest for p,digest in inputs.items()},
    robot_commands_sent=False,not_a_new_motion_authorization=True,placement_verified=False,
    successful_grasp_cases=1,success_rate_not_measured=True)
(args.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('rows','source_sha256','before_measurement')}),flush=True)
