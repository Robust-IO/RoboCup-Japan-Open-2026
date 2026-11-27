"""Current empty robot state supersedes the prior held-can checkpoint."""
import hashlib,json,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
read=lambda p:json.loads(p.read_text());reset=read(base/'can08-return-tilted-can-to-source-20261024-01/result.json')
empty=read(base/'can08-empty-hand-normalized-20261024-01/result.json');old=read(base/'canned-deeper-grasp-verified-progress-20261024.json')
if reset['release_commands_sent']!=1 or not reset.get('empty_arm_tucked') or empty['status']!='empty_tucked_hand_normalized':
    raise ValueError('actual_release_retreat_and_empty_normalization_required')
value=dict(status='can_no_longer_held_empty_robot_waiting_upright_source_reset',saved_ns=time.time_ns(),
    supersedes_current_robot_state_from='D:/handyman-training-runs/canned-deeper-grasp-verified-progress-20261024.json',
    old_deeper_grasp_evidence_preserved=True,robot_holding_can=False,robot_empty_tucked=True,
    source_can_seen_lying_below_table_after_release=True,target_current_pose_requires_user_confirmation=True,
    original_transfer_completed=False,default_initial_pose_full_transfer_verified=False,
    reset_steps=['Lowered original5cm','Opened to0.85 once','Retreated40cm','Tucked empty arm','Normalized empty hand to0.65'],
    reset_image='D:/handyman-training-runs/can08-return-tilted-can-to-source-20261024-01/captures/empty-tucked-source-view/rgb.png',
    reset_support_failure='After releasing the tilted can at the prior source position, the can rolled off the table. Actual tilted support was not measured before release. Do not report source placement success.',
    user_steering='User asks why earlier can grasps were upright but this one is tilted. Current priority is upright-start grasp validation; do not continue tilted carry as an equivalent result.',
    actual_current_robot_state=empty['current_state'],state_is_record_not_new_motion_authorization=True,
    latest_expected_hashes=old['latest_expected_hashes'],production_code_changed_this_turn=False,
    gripper_range_release_adaptation_still_pending=True,grasp_depth_wide_opening_not_integrated=True,
    next=['Never resume the old held-can carry or release scripts: robot is now empty.',
        'Wait for user to return the can from below table to an upright supported position on bedroom white_side_table. Unity can remain running.',
        'Reacquire fresh current camera/TF/joints before motion; do not use old can coordinates as measurements.',
        'Investigate contact geometry and verify upright grasp from the start, rather than using recovery of an already tilted object as evidence for improved upright grasp.',
        'Original automatic run08 and reviewed diagnostic results remain preserved, with whole transfer still unverified.'])
name='canned-upright-reset-required-progress-20261024.json';roots=[base,Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
if any((p/name).exists() for p in roots):raise ValueError('new_checkpoint_required')
data=json.dumps(value,indent=2,allow_nan=False).encode()
for p in roots:
    with (p/name).open('xb') as f:f.write(data)
print(json.dumps(dict(status=value['status'],checkpoint='D:/handyman-training-runs/'+name,
    sha256=hashlib.sha256(data).hexdigest(),robot_holding_can=False,empty_hand_opening_rad=empty['current_state']['joints'][5])),flush=True)
