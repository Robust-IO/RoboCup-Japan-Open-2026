"""Summarize actual completed transport with two reviewed corrections."""
import hashlib
import json
from pathlib import Path
import time

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
first=base/'can-auto-live-20261023-01';continuation=base/'can-reviewed-carry-place-20261023-01';last=continuation/'place-release-only-01'
r0=read(first/'result.json');r1=read(continuation/'result.json');grasp=read(first/'grasp/result.json')
carry=read(continuation/'carry/result.json');original_place=read(continuation/'place/result.json');result=read(last/'result.json')
verification=read(last/'placement-verification.json')
if (result['status']!='reviewed_can_placement_verified' or not result['placement_success_verified']
        or not verification['placement_success_verified'] or not result['robot_ready_for_next_task']):raise ValueError('actual_final_placement_required')
if (carry['status']!='can_retraction_and_destination_navigation_verified' or original_place['release_commands_sent']!=0
        or result['release_commands_sent']!=1 or r0['status']!='stopped_requires_actual_state_review'
        or r1['status']!='stopped_requires_actual_state_review'):raise ValueError('actual_reviewed_stage_history_required')
close_events=[json.loads(line) for line in (first/'grasp/journal.jsonl').read_text().splitlines()]
if len([e for e in close_events if e['event']=='can_closure_sent'])!=1:raise ValueError('exactly_one_actual_grasp_closure_required')
merged={}
for name in ('can-continuous-entry-v1-20261023-deployment.json','can-cap-axis-v1-20261023-deployment.json','can-release-timing-v1-20261023-deployment.json'):
    merged.update(read(base/name)['hashes'])
roots=[base,Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
for root in roots[1:]:
    if any(sha(root/n)!=digest for n,digest in merged.items()):raise ValueError('latest_source_copy_mismatch')
checks=verification['checks'];context=read(continuation/'held-context.json')
value=dict(status='can_transfer_completed_after_tilt_and_release_timing_fixes',saved_ns=time.time_ns(),
    boot_id=read(first/'run-meta.json')['boot_id'],branch='feat/handyman-phase5-grasping',
    object='canned_juice',layout='LayoutA',source='white_side_table#bedroom_1',destination='square_low_table#living_room',
    first_unified_run='D:/handyman-training-runs/'+first.name,
    reviewed_carry_place='D:/handyman-training-runs/'+continuation.name,
    final_release_and_cleanup='D:/handyman-training-runs/'+continuation.name+'/'+last.name,
    physical_transfer_completed=True,uninterrupted_full_transfer_verified=False,
    supervised_development_corrections=['actual13.7degree_grasped_can_axis_and_side_depth_tracking',
        'reuse_just_acquired_feedback_and_renew_expired_release_observation_before_publication'],
    original_failed_verdicts_preserved=True,real_grasp_close_commands=1,real_release_commands=1,
    grasp_approaches=len(grasp['approaches']),placement_approaches=len(original_place.get('approaches',[])),
    grasp_navigation_preparation_approach_replayed_during_release_continuation=False,
    can_axis_base_m_at_grasp=context['can_axis_base_m'],can_surface_radius_m=context['can_footprint_radius_m'],
    can_projected_support_radius_m=context['can_support_radius_m'],
    radius_limits_relaxed=False,source_age_limit_s=.4,extra_joint_wait_before_release_removed=True,
    final_three_current_depth_frames_verified=True,final_can_center_map_m=checks[-1]['center_map_m'],
    final_center_spread_m=verification['center_spread_m'],
    final_outer_support_clearance_lower_estimate_m=checks[-1]['table_edge_safety']['outer_clearance_lower_estimate_m'],
    final_rgb_capture=result['final_placement_rows'][-1]['capture'].replace('/mnt/d/','D:/'),
    robot_ready_for_next_task=True,last_measured_robot_state=result['last_fresh_robot_state'],
    last_state_is_a_record_not_future_action_authorization=True,tests=566,latest_expected_hashes=merged,
    model_map_navigation_route_tolerances_unchanged=True,task_finished_sent=False,success_rate_not_measured=True,
    next=['Can is RELEASED on living_room square_low_table. Robot is empty-handed and tucked. Do NOT repeat any held/release continuation.',
        'Read latest_expected_hashes from this checkpoint; the original continuous-entry manifest is superseded for changed can modules.',
        'The first uninterrupted test remains FAILED; actual object transfer completed after two reviewed code corrections without new grasp/close/approach commands.',
        'After user resets Unity session1, initial robot pose and upright canned_juice on bedroom white_side_table, run a NEW whole trial with run_object_transfer.sh and NEW D output.',
        'Only success from that new whole run may establish uninterrupted automation; keep all earlier evidence and failures.'])
name='canned-after-two-fixes-progress-20261023.json'
if any((root/name).exists() for root in roots):raise ValueError('new_checkpoint_required')
data=json.dumps(value,indent=2,allow_nan=False).encode()
for root in roots:
    with (root/name).open('xb') as f:f.write(data)
print(json.dumps(dict(status=value['status'],checkpoint='D:/handyman-training-runs/'+name,
    sha256=hashlib.sha256(data).hexdigest(),final_center=value['final_can_center_map_m'],
    stable_spread_m=value['final_center_spread_m'],uninterrupted_full_transfer_verified=False,ready=True)),flush=True)
