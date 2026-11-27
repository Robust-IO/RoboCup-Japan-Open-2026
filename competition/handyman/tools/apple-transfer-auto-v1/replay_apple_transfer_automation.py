"""Recorded geometry regression for automatic transfer. Never imports ROS.

Uses historical timestamps as a replay clock; these are NEVER live motion leases.
No model inference, robot action, protocol output or mutation of original captures.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from apple_approach_lease import measured_target
from apple_transfer_auto_plan import map_pose, plan_grasp, plan_placement, validate_config
from hand_camera_ray import rotation
from measure_destination_surface import measure
from table_release_gate import validate as validate_release


def read(path):
    return json.loads(Path(path).read_text())


def replay(root, config):
    semantic = read(config['semantic_map'])
    tables = {x['id']:x for x in semantic['destinations']}
    paths = sorted((root/'return-bedroom-apple-observe-01').glob('*/observation.json'))
    source_path = next((path for path in paths if read(path).get('identity_tracking',{}).get('identity_confirmed') is True), None)
    if source_path is None:
        # The initial read-only pilot did not establish an uninterrupted identity
        # track. Use the actual verified closure's evidence, never promote a seed.
        trial = read(root/'return-bedroom-close-lift-02/result.json')
        if trial.get('grasp_success_verified') is not True:
            raise ValueError('recorded_verified_grasp_required')
        source_path = Path(trial['close']['closure_identity_evidence']['evidence'])
    source = read(source_path)
    snapshot = read(source_path.parent/'snapshot.json')
    tf = snapshot['tracking_transforms']['odom_from_base_at_depth']
    r = rotation(tf['quaternion'])
    task = source['identity_tracking_task_id']
    evidence = measured_target(source,snapshot,now_ns=max(snapshot['streams'][k]['stamp_ns'] for k in ('rgb','depth'))+1_000_000,
                               epoch=snapshot['tracking_epoch'],task_id=task,current_odom=[*tf['translation'][:2],math.atan2(r[1,0],r[0,0])],expected_room='bedroom')
    grasp = plan_grasp(source,snapshot,tables[config['source_table_id']],evidence,config)
    capture = root/'return-square-table-outside'
    outside = read(capture/'snapshot.json')
    depth = np.load(capture/'depth_m.npy',allow_pickle=False)
    plane = measure(depth,outside,tables[config['destination_id']])
    old_held = read(root/'return-held-apple-geometry.json')
    held = dict(center_minus_palm_m=(np.asarray(old_held['center_base'])-old_held['palm_base']).tolist(),
                bottom_below_palm_m=old_held['estimated_bottom_below_palm_m'],radius_m=old_held['radius_m'],holding_opening_rad=.4157592239095287)
    plan = plan_placement(depth,outside,tables[config['destination_id']],plane,held,config,task)
    pre_capture = root/'return-square-pre-release'
    pre = read(pre_capture/'snapshot.json')
    replay_clock = max([pre['streams'][k]['stamp_ns'] for k in ('rgb','depth')]+[pre['joint_stamp_ns']])+1_000_000
    names = ('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
    release = validate_release(pre_capture,plan,now_ns=replay_clock,epoch=pre['tracking_epoch'],task_id=task,
                               current_map_pose=map_pose(pre),current_joints=[pre['joints'][k] for k in names])
    return dict(status='recorded_geometry_replay_passed',clock='HISTORICAL_REPLAY_ONLY_NOT_LIVE',
                grasp_plan=grasp,placement_plan=plan,release_check=release,robot_commands_sent=False,
                task_finished_sent=False,automatic_live_transfer_verified=False,
                limitation='Checks measured geometry against previous successful captures, not live stage transitions, navigation or end-to-end autonomous execution.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recordings',type=Path,required=True)
    p.add_argument('--config',type=Path,default=Path(__file__).parent/'config/apple_transfer_auto_v1.json')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists(): raise ValueError('output_exists')
    result=replay(a.recordings,validate_config(read(a.config)))
    with a.output.open('x') as stream: json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(status=result['status'],grasp_palm_height_m=result['grasp_plan']['palm_height_m'],
                         placement_palm_height_m=result['placement_plan']['palm_height_m'],
                         release_xy_map_m=result['placement_plan']['release_xy_map_m'],robot_commands_sent=False)))


if __name__=='__main__': main()
