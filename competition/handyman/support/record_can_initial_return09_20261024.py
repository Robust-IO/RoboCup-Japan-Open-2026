"""Persist a completed unified automatic transfer from the current living room."""
import json
import math
import time
from pathlib import Path
import numpy as np
base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'can-centered-initial-return-live-20261024-09'
read=lambda p:json.loads(p.read_text())
result=read(root/'result.json');grasp=read(root/'grasp/result.json');carry=read(root/'carry/result.json')
place=read(root/'place/result.json');verification=read(root/'place/placement-verification.json')
if not (result.get('physical_transfer_verified') and result.get('uninterrupted_transfer_verified')
        and place.get('placement_success_verified') and verification.get('placement_success_verified')):
    raise ValueError('completed_unified_transfer_required')
previous=read(base/'canned-centered-continuous-success-progress-20261024-08.json')
prior_grasp=read(base/'can-centered-continuous-live-20261024-08/grasp/result.json')
centers=lambda g:np.mean([r['center_odom_m'] for r in g['initial_observations']],axis=0)
delta=centers(grasp)-centers(prior_grasp)
events=[json.loads(line) for line in (root/'journal.jsonl').read_text().splitlines()]
started={e['stage']:e['time_ns'] for e in events if e.get('event')=='intent'}
ended={e['stage']:e['time_ns'] for e in events if e.get('event')=='confirmed'}
durations={s:(ended[s]-t)/1e9 for s,t in started.items() if s in ended}
preflight=read(root/'preflight.json');pose=preflight['base_map_pose']
afterlift=[read(Path(r['capture'])/'can-measurement.json') for r in grasp['lift_verification']]
initial_error=math.hypot(pose[0]-1.184,pose[1]);initial_yaw_error=abs(math.atan2(math.sin(pose[2]),math.cos(pose[2])))
returned=read(base/'can-empty-return-initial-20261024-09/result.json')
if initial_error>.061 or initial_yaw_error>.081:raise ValueError('run_did_not_start_near_reviewed_initial_pose')
value=dict(status='returned_initial_pose_unified_automatic_can_transfer_verified',saved_ns=time.time_ns(),
    run_root=str(root),supersedes_robot_state_record='canned-centered-continuous-success-progress-20261024-08.json',
    class_name='canned_juice',source='LayoutA bedroom white_side_table',destination='living_room square_low_table',
    unified_entry='run_object_transfer.sh --object canned_juice',
    one_parent_run_verified=True,uninterrupted_transfer_verified=True,physical_transfer_verified=True,
    navigation_grasp_carry_place_all_confirmed=result['confirmed_stages'],
    initial_base_map_pose=pose,default_initial_pose_error_m=initial_error,default_initial_yaw_error_rad=initial_yaw_error,
    default_initial_pose_start_verified_within_original_navigation_tolerance=True,
    fresh_unmoved_unity_restart_verified=False,
    start_kind='empty_robot_returned_to_reviewed_start_before_this_timed_unified_run',
    pretest_return_result=str(base/'can-empty-return-initial-20261024-09/result.json'),
    pretest_return_excluded_from_transfer_elapsed_s=True,
    pretest_position_error_m=returned['initial_position_error_m'],
    source_position_odom_m=centers(grasp).tolist(),source_delta_from_prior_odom_m=delta.tolist(),
    source_xy_shift_from_prior_m=float(np.linalg.norm(delta[:2])),
    source_detection_confidences=[r['detection']['confidence'] for r in grasp['initial_observations']],
    grasp_success_verified=grasp['grasp_success_verified'],afterlift_tilts_deg=[m['tilt_deg'] for m in afterlift],
    object_palm_follow_error_m=[r['object_palm_follow_error_m'] for r in grasp['lift_verification']],
    retraction_verified=carry['retraction_verified'],destination_navigation_verified=carry['destination_navigation_verified'],
    placement_success_verified=place['placement_success_verified'],release_commands_sent=place['release_commands_sent'],
    robot_holding_can=False,hand_released=place['hand_released'],cleanup_completed=place['cleanup_completed'],
    robot_ready_for_next_task=place['robot_ready_for_next_task'],
    robot_state_at_verified_completion=place['last_fresh_robot_state'],
    actual_placement_verification=verification,final_placement_rows=place['final_placement_rows'],
    final_object_tilt_deg=[r['measurement'].get('tilt_deg') for r in place['final_placement_rows']],
    elapsed_s=result['elapsed_s'],stage_elapsed_s=durations,extra_seat_m=grasp['extra_seat_m'],
    finger_midline_base_y_offset_m=-.00005,lateral_alignment_tolerance_m=.004,
    tests_passed=previous['tests_passed'],latest_expected_hashes=previous['latest_expected_hashes'],
    navigation_parameters_changed=False,model_changed=False,task_finished_sent=False,
    statistical_success_rate_established=False,historical_state_is_not_new_motion_authorization=True,
    next=['The robot is EMPTY after this new actual placement. Never resume any old held-can context.',
        'Keep true finger-midline centering and original forward depth with zero extra seating.',
        'For another new transfer, reacquire fresh current streams and use a new output directory.',
        'Keep this successful configuration and regress further source poses or another object next.',
        'The initial pose was reconstructed with navigation. This is not evidence of an untouched Unity restart or an 80% statistical success rate.'])
path=base/'canned-initial-return-continuous-success-progress-20261024-09.json'
with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
print(json.dumps(dict(saved=str(path),status=value['status'],elapsed_s=value['elapsed_s'],stage_elapsed_s=durations,
    source_xy_shift_m=value['source_xy_shift_from_prior_m'],robot_holding_can=False,
    robot_ready_for_next_task=value['robot_ready_for_next_task'],final_captures=[r['capture'] for r in place['final_placement_rows']])) )
