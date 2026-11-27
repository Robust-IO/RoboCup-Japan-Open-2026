"""Read-only r14 grasp/r15 placement regression. Never imports ROS or sends commands."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

p=argparse.ArgumentParser();p.add_argument('--code',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise ValueError('new_recorded_audit_required')
sys.path.insert(0,str(a.code))
from audit_preclose_identity_lift_follow import audit
from apple_transfer_auto_plan import release_region
from tracked_apple_surface import measure
from verify_apple_lift_sequence import palm_in_base
from verify_table_placement import verify

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
grasp=base/'apple-r14-reviewed-close-lift-20261018-01/grasp/close-lift'
place=base/'apple-r15-reviewed-carry-place-20261018-01'
read=lambda path:json.loads(path.read_text())
trial=read(grasp/'result.json');recording=Path(trial['observation_directory'])
plan=read(place/'placement-plan.json');release=read(place/'place-release/result.json')
capture=Path(release['release_geometry_evidence']['capture'])
final=read(place/'placed-final-audit.json')
paths={grasp/'result.json',grasp/'visual-lift-verification.json',grasp/'independent-lift-verification.json',
       place/'result.json',place/'placement-plan.json',place/'placed-final-audit.json',place/'placement-verification.json'}
paths.update(x for folder in recording.iterdir() if folder.is_dir() for x in folder.iterdir() if x.is_file())
paths.update(capture/name for name in ('snapshot.json','rgb.png','depth_m.npy'))
paths.update(place/f'place-{name}/result.json' for name in ('release','raise','retract'))
for row in final['frames']:
    paths.update(Path(row['capture'])/name for name in ('snapshot.json','rgb.png','depth_m.npy'))
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
before={str(path):sha(path) for path in paths}
held=audit(grasp,expected_recording=recording,expected_epoch=trial['tracking_epoch'],expected_task_id=trial['tracking_task_id'])
s=read(capture/'snapshot.json')
measured=measure(np.asarray(Image.open(capture/'rgb.png').convert('RGB')),
    np.load(capture/'depth_m.npy',allow_pickle=False),s,palm_in_base(s)+np.asarray(plan['held_center_minus_palm_m']))
region=release_region(measured,s,plan)
actions={name:read(place/f'place-{name}/result.json') for name in ('release','raise','retract')}
placed=verify(final,release['release_geometry_evidence'],actions,plan['destination_id'],plan['destination_room'],placement_plan=plan)
if any(sha(Path(path))!=digest for path,digest in before.items()):raise ValueError('original_evidence_changed')
passed=held['grasp_success_verified'] and region['ready'] and placed['placement_success_verified']
result=dict(status='recorded_outcome_regression_passed' if passed else 'recorded_outcome_regression_failed',
    grasp=held,release_region_geometry_only=region,placement=placed,
    original_files_unchanged=True,original_hashes=before,robot_commands_sent=False,
    new_live_action_lease=False,new_live_trial_performed=False,whole_uninterrupted_transfer_verified=False,
    thresholds_and_acceptance_policy_changed=True,
    warning='Recomputed historical RGBD, not current state, publication permission or a new uninterrupted physical trial. Original failures/results are unchanged.')
with a.output.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
print(json.dumps(dict(status=result['status'],grasp_frames=held['accepted_after_frames'],
    grasp_follow_spread_m=held['object_palm_relative_spread_m'],release_region_ready=region['ready'],
    placement_frames=placed['frames'],placement_spread_m=placed['center_spread_m'],
    original_files_unchanged=True,robot_commands_sent=False)),flush=True)
raise SystemExit(0 if passed else 1)
