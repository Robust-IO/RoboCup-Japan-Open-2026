"""Read-only known-apple association in new RGBD, preserving palm orientation."""
import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np
from PIL import Image
import yaml

sys.path.insert(0, '/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
from hand_camera_ray import rotation
from verify_apple_lift_sequence import palm_in_base
from tracked_apple_surface import measure
from apple_transfer_auto_plan import map_pose, file_hash

p = argparse.ArgumentParser()
p.add_argument('capture', type=Path)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--prepared-plan', type=Path)
a = p.parse_args()
if a.output.exists():
    raise ValueError('output_exists')
root = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
prior_file = root/'apple-held-after-retract-20261017-01/held-retraction-audit.json'
prior = json.loads(prior_file.read_text())
if prior.get('holding_object_verified') is not True:
    raise ValueError('verified_held_apple_required')
row = prior['rows'][-1]
previous = json.loads((Path(row['capture'])/'snapshot.json').read_text())
def orient(s):
    return rotation(s['base_from_hand']['quaternion']) @ rotation(s['palm_from_hand']['quaternion']).T
offset = orient(previous).T @ (np.asarray(row['measurement']['center_base'])-palm_in_base(previous))
names = ['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint']
expected_joints = [0.,0.,0.,-math.pi/2,0.,prior['hand_opening_rad']]
prepared_plan = None
if a.prepared_plan:
    from inverse_table_place_plan import OFFSET,FLEX,WRIST
    prepared_plan = json.loads(a.prepared_plan.read_text())
    if (prepared_plan['destination_id'] != 'square_low_table#living_room'
            or abs(prepared_plan['holding_opening_rad']-prior['hand_opening_rad']) > 1e-8):
        raise ValueError('wrong_prepared_plan')
    expected_joints = [prepared_plan['palm_height_m']-OFFSET,FLEX,0.,WRIST,0.,prior['hand_opening_rad']]
rows = []
last_stamp = row['stamp_ns']
for path in sorted(a.capture.glob('*/snapshot.json')):
    s = json.loads(path.read_text())
    stamp = s['streams']['depth']['stamp_ns']
    if stamp <= last_stamp or max(abs(s['joints'][n]-v) for n,v in zip(names,expected_joints)) > .003:
        raise ValueError('current_holding_pose_or_capture_order_failed')
    if s.get('tracking_epoch') != a.capture.name:
        raise ValueError('wrong_capture_epoch')
    last_stamp = stamp
    expected = palm_in_base(s)+orient(s)@offset
    m = measure(np.asarray(Image.open(path.parent/'rgb.png').convert('RGB')),
                np.load(path.parent/'depth_m.npy',allow_pickle=False),s,expected)
    if abs(m['radius_m']-row['measurement']['radius_m']) > .008:
        raise ValueError('tracked_radius_changed')
    rows.append(dict(capture=str(path.parent),stamp_ns=stamp,joints=[s['joints'][n] for n in names],
                     base_map_pose=map_pose(s),measurement=m))
span = (rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9 if len(rows)>1 else 0
spread = float(np.max(np.linalg.norm(np.asarray([r['measurement']['center_base'] for r in rows])
                                    -rows[-1]['measurement']['center_base'],axis=1))) if rows else None
if len(rows)<3 or span<.4 or spread>.01:
    raise ValueError('multi_frame_held_confirmation_failed')
config = json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text())
if file_hash(config['environment']) != config['environment_sha256']:
    raise ValueError('environment_changed')
environment = yaml.safe_load(Path(config['environment']).read_text())
semantic = json.loads(Path(config['semantic_map']).read_text())
result = dict(status='current_carried_apple_rgbd_verified',holding_object_verified=True,
              arm_is_tucked=prepared_plan is None,prepared_plan=str(a.prepared_plan) if a.prepared_plan else None,
              verification_source=str(prior_file),offset_in_palm_frame_m=offset.tolist(),
              samples=len(rows),span_s=span,measured_center_spread_m=spread,rows=rows,
              source_table=next(x for x in semantic['destinations'] if x['id']==config['source_table_id']),
              destination_table=next(x for x in semantic['destinations'] if x['id']==config['destination_id']),
              carrying_route=next(r for r in environment['routes'] if r['from']=='bedroom' and r['to']=='living_room'),
              destination_staging_goal=config['destination_staging_goal'],
              room_regions={k:v['region'] for k,v in environment['rooms'].items()},
              robot_commands_sent=False,motion_authorized=False,
              limitation='Recorded known-apple geometry check only; actual movement tools must check current joints/TF/controller ownership again.')
with a.output.open('x') as f:
    json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in result.items() if k not in ('rows','room_regions')},ensure_ascii=False),flush=True)
print(json.dumps({'last':rows[-1]},ensure_ascii=False),flush=True)
