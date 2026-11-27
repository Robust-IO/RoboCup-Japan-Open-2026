"""Persist actual stop state and tested alignment correction; no robot commands."""
import hashlib,json,subprocess,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
roots=[base,Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
old=read(base/'canned-current-start-continuous-success-progress-20261023-05.json')
startup=read(base/'can-coherent-startup-v1-20261023-deployment.json');fix=read(base/'can-lateral-align-v1-20261024-deployment.json')
reset=read(base/'can07-empty-retreat-tuck-20261023-01/result.json');failed=read(base/'can-auto-live-20261023-07/result.json')
if (fix['status']!='can_lateral_alignment_deployed' or reset['status']!='empty_retreat_and_tuck_verified'
        or failed['status']!='stopped_requires_actual_state_review' or not reset['no_previous_closure']):
    raise ValueError('actual_failure_and_empty_retreat_required')
hashes=dict(old['latest_expected_hashes'],**startup['hashes']);hashes.update(fix['hashes'])
for root in roots[1:]:
    if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('tested_copy_mismatch')
value=dict(status='can_alignment_fix_deployed_waiting_for_upright_target',saved_ns=time.time_ns(),
    branch=subprocess.check_output(['git','-C',str(repo),'branch','--show-current'],text=True).strip(),
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
    object='canned_juice',layout='LayoutA',source='white_side_table#bedroom_1',destination='square_low_table#living_room',
    original_initial_start_run06='D:/handyman-training-runs/can-auto-live-20261023-06',
    original_initial_start_run07='D:/handyman-training-runs/can-auto-live-20261023-07',
    original_failed_verdicts_preserved=True,default_initial_pose_whole_run_verified=False,
    last_known_robot_state=reset['final_state'],robot_state_record_is_not_future_motion_authorization=True,
    current_arm_empty_tucked_verified_at_reset=True,current_hand_open_rad=reset['final_state']['joints'][5],
    no_grasp_closure_in_run07=True,robot_retreat_after_failure_m=.4,
    user_observation='Gripper was not facing/aligned with can; one side touched and tipped the can during approach.',
    original_failed_approach_delta_m=fix['failed_actual_plan'],
    correction='Align sideways while clear of can, then use one complete straight forward approach; if near and still laterally misaligned, retreat before a lateral correction.',
    forward_distance_subdivided=False,geometry_or_freshness_thresholds_relaxed=False,
    startup_race_corrected=True,actual_startup_windows_readonly_verified=5,
    alignment_fix_live_verified=False,physical_target_reset_confirmation_pending=True,
    tests=fix['tests'],latest_expected_hashes=hashes,
    model_map_navigation_route_tolerances_unchanged=True,apple_default_behavior_unchanged=True,
    checkpoints_are_not_live_held_contexts=True,task_finished_sent=False,
    next=['Robot was withdrawn40cm and tucked; no close was sent. Do not replay the failed diagonal approach.',
        'Wait for user confirmation that the tilted can has been put upright on the bedroom side table; Unity can stay running.',
        'Before moving reacquire current RGBD/joints/map state. Use a new output directory and task identity.',
        'Validate lateral alignment then straight approach first. Do not claim a complete initial-start success from a source-staging start.',
        'A full initial-pose run remains pending; run06 failed before arm command, run07 failed after contact approach before closure.'])
name='canned-alignment-fix-ready-progress-20261024.json'
if any((root/name).exists() for root in roots):raise ValueError('new_checkpoint_required')
data=json.dumps(value,indent=2,allow_nan=False).encode()
for root in roots:
    with (root/name).open('xb') as f:f.write(data)
print(json.dumps(dict(status=value['status'],checkpoint='D:/handyman-training-runs/'+name,
    sha256=hashlib.sha256(data).hexdigest(),tests=value['tests'],last_known_state=value['last_known_robot_state'],
    live_alignment_verified=False,full_initial_start_verified=False)),flush=True)
