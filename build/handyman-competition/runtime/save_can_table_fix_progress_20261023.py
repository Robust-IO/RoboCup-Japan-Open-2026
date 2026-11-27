"""Actual run02 outcome after its table-only correction; not continuous success."""
import hashlib
import json
from pathlib import Path
import time

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
root=base/'can-auto-live-20261023-02';final=root/'place-after-table-fix-01'
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
parent=read(root/'result.json');grasp=read(root/'grasp/result.json');carry=read(root/'carry/result.json')
failed=read(root/'place/result.json');result=read(final/'result.json');verified=read(final/'placement-verification.json')
if (parent['status']!='stopped_requires_actual_state_review' or parent['stage']!='place_verify'
        or grasp['status']!='can_grasp_and_lift_verified' or carry['status']!='can_retraction_and_destination_navigation_verified'
        or failed['stage']!='fresh_destination_and_held_can' or failed['release_commands_sent']!=0
        or result['status']!='reviewed_can_placement_verified' or not verified['placement_success_verified']
        or not result['robot_ready_for_next_task']):raise ValueError('actual_stage_history_and_final_success_required')
closed=[json.loads(s) for s in (root/'grasp/journal.jsonl').read_text().splitlines()]
released=[json.loads(s) for s in (final/'journal.jsonl').read_text().splitlines()]
if len([e for e in closed if e['event']=='can_closure_sent'])!=1 or len([e for e in released if e['event']=='can_release_sent'])!=1:
    raise ValueError('exactly_one_close_and_release_required')
latest=read(base/'canned-after-two-fixes-progress-20261023.json');deployment=read(base/'can-table-plane-v1-20261023-deployment.json')
hashes=dict(latest['latest_expected_hashes'],**deployment['hashes'])
roots=[base,Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
for code in roots[1:]:
    if any(sha(code/n)!=digest for n,digest in hashes.items()):raise ValueError('latest_code_copy_mismatch')
checks=verified['checks'];name='canned-after-table-plane-fix-progress-20261023.json'
if any((p/name).exists() for p in roots):raise ValueError('new_checkpoint_required')
value=dict(status='run02_physical_transfer_completed_after_table_plane_fix',saved_ns=time.time_ns(),
    branch='feat/handyman-phase5-grasping',boot_id=read(root/'run-meta.json')['boot_id'],
    object='canned_juice',layout='LayoutA',source='white_side_table#bedroom_1',destination='square_low_table#living_room',
    original_unified_trial='D:/handyman-training-runs/'+root.name,
    actual_final_placement='D:/handyman-training-runs/'+root.name+'/'+final.name,
    whole_run_before_table_fix_elapsed_s=parent['elapsed_s'],
    grasp_lift_retraction_and_round_trip_navigation_continuously_verified=True,
    tilt_tracking_fix_automatically_passed=True,release_time_frame_renewal_automatically_passed=True,
    table_plane_fix='Dominant depth-plane seed excludes near-height tabletop clutter before fitting. Original4mm residual limit retained.',
    original_failed_table_measurement_preserved=True,physical_transfer_completed=True,uninterrupted_full_transfer_verified=False,
    robot_ready_for_next_task=True,actual_close_commands=1,actual_release_commands=1,
    grasp_approaches=len(grasp['approaches']),placement_approaches=len(result.get('approaches',[])),
    automatic_release_frame_renewals=len([e for e in released if e['event']=='expired_release_frame_reacquire_without_action']),
    final_three_current_depth_frames_verified=True,final_can_center_map_m=checks[-1]['center_map_m'],
    final_center_spread_m=verified['center_spread_m'],final_outer_support_clearance_lower_estimate_m=checks[-1]['table_edge_safety']['outer_clearance_lower_estimate_m'],
    final_rgb_capture=result['final_placement_rows'][-1]['capture'].replace('/mnt/d/','D:/'),
    last_measured_robot_state=result['last_fresh_robot_state'],last_state_is_record_not_future_motion_authorization=True,
    latest_expected_hashes=hashes,tests=deployment['tests'],latest_deployment='D:/handyman-training-runs/can-table-plane-v1-20261023-deployment.json',
    original_apple_model_map_navigation_route_tolerances_unchanged=True,task_finished_sent=False,success_rate_not_measured=True,
    next=['Can is RELEASED on living_room square_low_table; robot is empty-handed and tucked. Never repeat held/release continuations.',
        'Run02 passed grasp/lift/retraction/both navigations continuously, then stopped at its first tabletop measurement. No placement action had started.',
        'New can-only dominant-plane extraction passed5 originally failed real frames plus3 older successful frames and571 tests. Only placement was then executed, without repeated grasp/navigation.',
        'Source-age renewals before release worked automatically; one physical release, final3 RGBD frames stable, cleanup complete.',
        'Read latest_expected_hashes here (table-plane update supersedes previous entry hashes).',
        'After user resets Unity session1, initial robot pose and upright canned_juice on bedroom white_side_table, run NEW can-auto-live-20261023-03 with the unified entry.',
        'Keep run01/run02 failed parent verdicts. Only a genuinely new complete whole run may claim uninterrupted automation.'])
data=json.dumps(value,indent=2,allow_nan=False).encode()
for p in roots:
    with (p/name).open('xb') as f:f.write(data)
print(json.dumps(dict(status=value['status'],checkpoint='D:/handyman-training-runs/'+name,sha256=hashlib.sha256(data).hexdigest(),
    tests=value['tests'],release_frame_renewals=value['automatic_release_frame_renewals'],
    final_rgb_capture=value['final_rgb_capture'],final_center=value['final_can_center_map_m'],ready=True,uninterrupted_full_transfer_verified=False)),flush=True)
