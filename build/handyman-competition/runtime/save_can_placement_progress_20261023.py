"""Save genuine observed can placement; no ROS or commands."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
roots=[base,Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
trial=base/'canned-place-live-20261023-01';result=read(trial/'result.json');verified=read(trial/'placement-verification.json')
if result.get('status')!='reviewed_can_placement_verified' or not verified['placement_success_verified']:
    raise ValueError('physical_can_placement_success_required')
if result['release_commands_sent']!=1 or not result['hand_released'] or result['task_finished_sent']:
    raise ValueError('one_actual_release_without_protocol_success_required')
baseline=read(base/'canned-place-v1-20261023-baseline.json');deployed=read(base/'canned-place-v1-20261023-deployment.json')
update=read(base/'canned-place-v1-20261023-entry-update.json')
for label,root in zip(('runtime','training','module'),roots[1:]):
    for name,digest in baseline['frozen'][label].items():
        if sha(root/name)!=digest:raise ValueError('original_file_changed')
    for name,digest in deployed['hashes'].items():
        if name=='run_canned_place_trial.py':digest=update['source_sha256']
        if sha(root/name)!=digest:raise ValueError('new_addon_copy_changed')
for path,digest in baseline['external'].items():
    if sha(Path(path))!=digest:raise ValueError('original_map_model_changed')
events=[json.loads(line) for line in (trial/'journal.jsonl').read_text().splitlines()]
name='canned-transfer-verified-progress-20261023.json'
if any((root/name).exists() for root in roots):raise ValueError('new_checkpoint_targets_required')
checks=verified['checks'];last=result['last_fresh_robot_state']
checkpoint=dict(status='one_canned_juice_physical_transfer_verified',saved_ns=time.time_ns(),
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
    branch=subprocess.check_output(['git','-C',str(repo),'branch','--show-current'],text=True).strip(),
    object='canned_juice',source_table='white_side_table#bedroom_1',destination_table='square_low_table#living_room',
    actual_placement_trial='D:/handyman-training-runs/'+trial.name,
    actual_prior_navigation='D:/handyman-training-runs/tucked-can-navigation-live-20261023-01',
    actual_grasp_and_lift='D:/handyman-training-runs/canned-grasp-lift-live-progress-20261023.json',
    source_grasp_lift_verified=True,held_retraction_verified=True,original_route_navigation_verified=True,
    destination_placement_verified=True,one_full_measured_approach_used=len(result.get('approaches',[]))==1,
    approach_commands=len(result.get('approaches',[])),release_commands=1,hand_open_verified=True,
    cleanup_completed=result['cleanup_completed'],robot_ready_for_next_task=result['robot_ready_for_next_task'],
    final_can_center_map_m=checks[-1]['center_map_m'],final_bottom_gap_estimates_m=[c['estimated_bottom_gap_m'] for c in checks],
    final_center_spread_m=verified['center_spread_m'],
    final_outer_table_support_clearance_lower_estimates_m=[c['table_edge_safety']['outer_clearance_lower_estimate_m'] for c in checks],
    can_bottom_derived_from_measured_height_not_horizontal_radius=True,
    zero_depth_never_used_as_3d_position=True,final_actual_depth_observation_frames=3,
    final_rgb_capture=result['final_placement_rows'][-1]['capture'].replace('/mnt/d/','D:/'),
    placement_duration_estimate_s=(last['stamp_ns']-events[0]['time_ns'])/1e9,
    candidate_tests=deployed['candidate_tests'],final_entry_tests=update['tests'],
    original_apple_sources_model_map_route_navigation_parameters_unchanged=True,
    actual_place_code_entry_sha256=update['source_sha256'],
    task_finished_sent=False,uninterrupted_start_to_finish_automation_verified=False,
    successful_transfer_cases=1,success_rate_not_measured=True,
    last_measured_robot_state=last,last_state_is_record_not_future_motion_authorization=True,
    next=[
        'The can is now RELEASED on living_room square_low_table. Do NOT rerun held-can navigation or placement, close the empty hand, or repeat release.',
        'Use NEW images and joints for any next task. This checkpoint records actual success from a supervised staged development transfer, not one uninterrupted fresh-start automatic run.',
        'Next integrate the can grasp/carry/place adapters under the generic coordinator and perform one fresh-start continuous validation on the same route before changing objects or routes.',
        'Retain all earlier failed verdicts and images. One case does not establish80% grasp/85% placement success or30-class manipulation coverage.'
    ])
data=json.dumps(checkpoint,indent=2,allow_nan=False).encode()
for root in roots:
    with (root/name).open('xb') as f:f.write(data)
    if (root/name).read_bytes()!=data:raise ValueError('copy_mismatch')
print(json.dumps(dict(status=checkpoint['status'],checkpoint='D:/handyman-training-runs/'+name,
    sha256=hashlib.sha256(data).hexdigest(),copies=4,placement_duration_estimate_s=checkpoint['placement_duration_estimate_s'],
    final_center=checkpoint['final_can_center_map_m'],bottom_gaps=checkpoint['final_bottom_gap_estimates_m'],
    stable_spread_m=checkpoint['final_center_spread_m'],approaches=checkpoint['approach_commands'],
    cleanup_completed=checkpoint['cleanup_completed'],ready=checkpoint['robot_ready_for_next_task'])),flush=True)
