"""Recorded failure diagnosis only; original files and robot remain unchanged."""
import json
from pathlib import Path
import sys
import math

CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(CODE))
from hand_camera_ray import rotation
import numpy as np
from apple_approach_lease import closure_position_error
BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');ROOT=BASE/'automatic-apple-r17-20261022-01'
read=lambda p:json.loads(p.read_text())
report=dict(status=read(ROOT/'result.json'),grasp=read(ROOT/'grasp/result.json'),
    grasp_plan=read(ROOT/'measured-grasp-plan.json'),rows=[],robot_commands_sent=False)
height=report['grasp_plan']['palm_height_m']
for path in sorted((ROOT/'source-observe').glob('*/observation.json'))[-10:]:
    observation=read(path);snapshot=read(path.parent/'snapshot.json')
    identity=observation.get('identity_tracking',{});row=dict(capture=path.parent.name,
        identity_status=identity.get('status'),identity_confirmed=identity.get('identity_confirmed'),
        reason=identity.get('reason'),depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'])
    if identity.get('measurement'):
        measurement=identity['measurement'];tf=snapshot['tracking_transforms']['odom_from_base_at_depth']
        target=rotation(tf['quaternion']).T@(np.asarray(measurement['point_odom_m'])-tf['translation'])
        row.update(target_base_m=target.tolist(),target_minus_contact_m=(target-[.6892952185,.078,height]).tolist())
        try:closure_position_error(target,height);row['inside_close_region']=True
        except ValueError as exc:row.update(inside_close_region=False,close_reason=str(exc))
    report['rows'].append(row)
old=BASE/'automatic-apple-r16b-20261022-01'
report['previous_source_plan']=read(old/'measured-grasp-plan.json')
with (ROOT/'stopped-readonly-diagnosis.json').open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps(report,indent=2),flush=True)
