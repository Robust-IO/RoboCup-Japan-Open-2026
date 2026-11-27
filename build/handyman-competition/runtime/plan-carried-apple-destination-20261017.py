"""Recorded destination-table measurement and edge-safe plan; never sends motion."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, '/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
from apple_transfer_auto_plan import (map_pose, map_point, plan_placement, validate_config)
from hand_camera_ray import rotation
from measure_destination_surface import measure
from pregrasp_geometry import expected_horizontal_center
from verify_apple_lift_sequence import palm_in_base

p=argparse.ArgumentParser()
p.add_argument('capture',type=Path)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
a.output.mkdir(exist_ok=False)
report=dict(status='inconclusive',motion_authorized=False,robot_commands_sent=False,
            release_authorized=False,scope='Recorded plan only; fresh actual measurements required before each future action.')
try:
    held_audit=json.loads((a.capture/'carried-apple-audit.json').read_text())
    if held_audit.get('holding_object_verified') is not True:
        raise ValueError('new_carried_apple_verification_required')
    last=held_audit['rows'][-1]
    s=json.loads((Path(last['capture'])/'snapshot.json').read_text())
    m=last['measurement']
    palm=palm_in_base(s)
    r=rotation(s['base_from_hand']['quaternion'])@rotation(s['palm_from_hand']['quaternion']).T
    offset_palm=r.T@(np.asarray(m['center_base'])-palm)
    reference=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/apple-held-before-retract-20261017-01/000004/snapshot.json').read_text())
    horizontal_rotation=rotation(reference['base_from_hand']['quaternion'])@rotation(reference['palm_from_hand']['quaternion']).T
    horizontal_offset=horizontal_rotation@offset_palm
    held=dict(center_minus_palm_m=horizontal_offset.tolist(),
              bottom_below_palm_m=float(m['radius_m']-horizontal_offset[2]),
              radius_m=m['radius_m'],holding_opening_rad=last['joints'][5],
              verified_capture=last['capture'],center_offset_in_palm_frame_m=offset_palm.tolist(),
              offset_reexpressed_for_horizontal_preparation=True)
    config=validate_config(json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text()))
    table=held_audit['destination_table']
    planes=[]
    for row in held_audit['rows']:
        path=Path(row['capture'])
        snap=json.loads((path/'snapshot.json').read_text())
        depth=np.load(path/'depth_m.npy',allow_pickle=False)
        planes.append(dict(capture=str(path),measurement=measure(depth,snap,table)))
    if len(planes)<3 or np.ptp([row['measurement']['table_height_map_m'] for row in planes])>.01:
        raise ValueError('stable_destination_surface_required')
    plane=planes[-1]['measurement']
    depth=np.load(Path(last['capture'])/'depth_m.npy',allow_pickle=False)
    (a.output/'held-geometry.json').write_text(json.dumps(held,indent=2,allow_nan=False))
    (a.output/'destination-surface.json').write_text(json.dumps(plane,indent=2,allow_nan=False))
    (a.output/'surface-observations.json').write_text(json.dumps(planes,indent=2,allow_nan=False))
    report.update(held_geometry=held,base_map_pose=map_pose(s),
                  table_height_m=plane['table_height_map_m'],table_plane_samples=len(planes),
                  plane_residual_p95_m=plane['residual_p95_m'],
                  observed_table_xy_min=plane['observed_xy_min'],observed_table_xy_max=plane['observed_xy_max'])
    plan=plan_placement(depth,s,table,plane,held,config,a.capture.name)
    predicted=map_point(s,expected_horizontal_center(plan['palm_height_m'],held['center_minus_palm_m']))
    table_front=min(plane['observed_xy_min'][1],plan['footprint_center'][1]-plan['footprint_half_size'][1])
    if predicted[1]+max(held['radius_m'],.08)>=table_front-.03:
        raise ValueError('horizontal_preparation_not_outside_table')
    plan['predicted_apple_before_approach_map_m']=predicted.tolist()
    (a.output/'placement-plan.json').write_text(json.dumps(plan,indent=2,allow_nan=False))
    report.update(status='recorded_destination_edge_safe_plan_available',
                  release_xy_map_m=plan['release_xy_map_m'],palm_height_m=plan['palm_height_m'],
                  planned_approach_distance_m=plan['planned_approach_distance_m'],
                  table_edge_safety=plan['table_edge_safety'],
                  horizontal_preparation_outside_table=True)
except Exception as exc:
    report['reason']=str(exc)
(a.output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
print(json.dumps(report),flush=True)
if report['status']!='recorded_destination_edge_safe_plan_available':
    raise SystemExit(1)
