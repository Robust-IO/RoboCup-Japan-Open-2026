"""Record the actual navigation/held result, without commands or success inflation."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
roots=[base,Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
trial=base/'tucked-can-navigation-live-20261023-01';result=read(trial/'result.json')
nav=read(trial/'destination-navigation.json');deploy=read(base/'held-can-navigation-v1-20261023-deployment.json')
if result['status']!='original_route_can_navigation_and_current_hold_verified' or nav['status']!='arrived':
    raise ValueError('actual_navigation_and_current_hold_success_required')
if not result['navigation_controller_verified'] or not result['destination_held_verified'] or result['hand_released']:
    raise ValueError('incorrect_actual_outcome')
for key in ('before_navigation_hold','after_navigation_hold'):
    if len(result[key]['rows'])!=3:raise ValueError('three_current_frames_required')
baseline=read(base/'held-can-navigation-v1-20261023-baseline.json')
for label,root in zip(('runtime','training','module'),roots[1:]):
    for name,digest in baseline['frozen'][label].items():
        if sha(root/name)!=digest:raise ValueError('original_file_changed:'+str(root/name))
    for name,digest in deploy['hashes'].items():
        if sha(root/name)!=digest:raise ValueError('addon_changed:'+str(root/name))
for path,digest in baseline['external'].items():
    if sha(Path(path))!=digest:raise ValueError('original_model_or_map_changed')
events=[json.loads(line) for line in (trial/'journal.jsonl').read_text().splitlines()]
stamps={e['stage']:e['time_ns'] for e in events if e['event']=='stage'}
post=result['after_navigation_hold']['rows'];state=result['last_fresh_robot_state']
checkpoint=dict(status='can_tucked_carry_to_living_room_verified_placement_pending',saved_ns=time.time_ns(),
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
    branch=subprocess.check_output(['git','-C',str(repo),'branch','--show-current'],text=True).strip(),
    actual_trial='D:/handyman-training-runs/'+trial.name,
    deployment='D:/handyman-training-runs/held-can-navigation-v1-20261023-deployment.json',
    candidate_tests=deploy['candidate_tests'],runtime_tests=deploy['runtime_tests'],recorded_frames=deploy['recorded_frames'],
    original_route_navigation_verified=True,holding_after_navigation_verified=True,
    held_observation_types=[row['evidence']['evidence_type'] for row in post],
    held_identity_source='previous_genuine_RGBD_grasp_followed_by_current_body_and_cap_appearance_tracking',
    fresh_hold_frames_at_destination=len(post),
    zero_depth_is_not_an_object_coordinate=True,
    current_held3d_position_measured=all(row['evidence'].get('measured_3d_object_position_available',False) for row in post),
    current_can_appearance_supports_no_new_object_search=True,
    physical_arm_retraction_completed=True,arm_commands_sent_this_step=False,gripper_commands_sent_this_step=False,
    grasp_retraction_or_close_replayed=False,hand_released=False,placement_started=False,placement_verified=False,
    task_finished_sent=False,uninterrupted_full_transfer_verified=False,
    navigation_duration_s=(stamps['verify_destination_tucked_can']-stamps['original_route_to_square_table'])/1e9,
    navigation_target=nav['target'],actual_navigation_final_pose=nav['final_pose'],
    navigation_position_error_m=nav['position_error_m'],navigation_precision=nav['current_precision'],
    waypoint_results=nav['reached'],speed_parameters_restored=nav['speed_parameters_restored'],
    original_code_model_map_route_tolerances_unchanged=True,
    last_measured_robot_state=state,last_state_is_a_record_not_future_action_authorization=True,
    last_rgb_capture=post[-1]['capture'].replace('/mnt/d/','D:/'),
    can_height_m=read(base/'canned-grasp-lift-live-progress-20261023.json')['can_height_m'],
    next=[
        'Robot is TUCKED and holding canned_juice in living_room near square_low_table. Do NOT repeat grasp, close, lift, retraction or navigation from the bedroom.',
        'Only resume from NEW current state/images. Maintain grip while measuring the actual destination tabletop and preparing inverse-grasp placement.',
        'Use actual can height/bottom-to-palm clearance for a vertical can; horizontal footprint radius is not its bottom offset.',
        'Near-clipped held RGB appearance is not measured3D and must not be used as a new target detector or a table measurement.',
        'No release before actual destination placement geometry is established; do not send Task_finished before physical placement verification.'
    ])
name='canned-carry-navigation-progress-20261023.json'
if any((root/name).exists() for root in roots):raise ValueError('new_checkpoint_targets_required')
data=json.dumps(checkpoint,indent=2,allow_nan=False).encode()
for root in roots:
    with (root/name).open('xb') as f:f.write(data)
    if (root/name).read_bytes()!=data:raise ValueError('checkpoint_copy_mismatch')
print(json.dumps(dict(status=checkpoint['status'],checkpoint='D:/handyman-training-runs/'+name,
    sha256=hashlib.sha256(data).hexdigest(),copies=len(roots),duration_s=checkpoint['navigation_duration_s'],
    final_pose=nav['final_pose'],position_error_m=nav['position_error_m'],held_evidence_types=checkpoint['held_observation_types'],
    hand_released=False,placement_verified=False)),flush=True)
