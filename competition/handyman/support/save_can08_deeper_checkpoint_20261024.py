"""Persist verified reviewed deeper grasp; not a full transfer or motion lease."""
import hashlib,json,subprocess,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
roots=[base,Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
failed=read(base/'can-auto-live-20261024-08/result.json')
advance=read(base/'can08-forward-four-cm-20261024-01/result.json')
closed=read(base/'can08-deeper-close-20261024-01/result.json')
lift=read(base/'can08-deeper-lift-probe-20261024-01/result.json')
independent=read(base/'can08-green-depth-final-verification-20261024-01/result.json')
current=read(base/'can08-current-green-held-verification-20261024-01/result.json')
now=read(base/'can08-deeper-held-now-20261024-01/result.json')
if (failed['status']!='stopped_requires_actual_state_review'
        or advance['status']!='one_four_cm_forward_completed_requires_new_grasp_review'
        or closed['status']!='deeper_close_contact_candidate_requires_lift_verification'
        or closed['close_commands_sent']!=1 or not lift['lift_command_sent']
        or not independent['current_grasp_follow_verified'] or not current['current_grasp_follow_verified']
        or len(independent['rows'])!=3 or len(current['rows'])!=3):raise ValueError('actual_independent_green_depth_and_current_hold_required')
previous=read(base/'canned-alignment-fix-ready-progress-20261024.json');hashes=previous['latest_expected_hashes']
for code in roots[1:]:
    if any(sha(code/n)!=digest for n,digest in hashes.items()):raise ValueError('baseline_changed_during_reviewed_trial')
opening=now['current_state']['joints'][5]
if abs(opening-closed['closure_evidence']['observed_opening_rad'])>.03:raise ValueError('current_grip_changed')
value=dict(status='reviewed_deeper_can_grasp_and_five_cm_lift_verified',saved_ns=time.time_ns(),
    branch=subprocess.check_output(['git','-C',str(repo),'branch','--show-current'],text=True).strip(),
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),object='canned_juice',layout='LayoutA',
    source='white_side_table#bedroom_1',intended_destination='square_low_table#living_room',
    original_failed_auto_run='D:/handyman-training-runs/can-auto-live-20261024-08',
    original_failed_auto_run_verdict_preserved=True,automatic_full_transfer_verified=False,
    default_initial_pose_full_run_verified=False,reviewed_deeper_grasp_verified=True,
    user_requested_extra_forward_before_closing=True,extra_forward_m=.04,extra_forward_commands=1,
    wider_opening_before_new_close_rad=.85,actual_new_deeper_close_commands=1,
    original_run08_close_commands=1,total_close_commands_since_run08=2,
    five_cm_lift_commands=1,new_grasp_command_not_replayed_during_lift=True,
    actual_green_surface_lift_m=[r['green_surface_delta_odom_m'][2] for r in independent['rows']],
    actual_green_depth_points_per_frame=[r['green_head_depth_features'] for r in independent['rows']],
    current_hold_three_frames_verified=True,robot_now_holding_can=True,can_pose_still_tilted=True,
    arm_retracted=False,can_carry_or_placement_verified=False,task_finished_sent=False,
    current_holding_opening_rad=opening,last_measured_robot_state=now['current_state'],
    last_state_is_record_not_future_motion_authorization=True,
    original_automation_regression_tests=previous['tests'],latest_expected_hashes=hashes,
    deeper_seating_and_wider_open_not_yet_integrated_in_production=True,
    production_can_adapter_maximum_holding_opening_rad=.62,production_release_opening_target_rad=.65,
    requires_grip_and_release_policy_adaptation_before_standard_carry_or_place=True,
    observation='Can-only green body features in registered depth follow the measured palm lift in three frames, with unchanged grip. Later current frames retain that displacement. Full object center/axis is not reconstructed here.',
    mixed_silver_probe_not_used_as_sole_grasp_evidence=True,
    hand_pixel_flow_outlier_warning_preserved=True,
    actual_evidence={
        'extra_forward':'D:/handyman-training-runs/can08-forward-four-cm-20261024-01/result.json',
        'new_close':'D:/handyman-training-runs/can08-deeper-close-20261024-01/result.json',
        'lift':'D:/handyman-training-runs/can08-deeper-lift-probe-20261024-01/result.json',
        'independent_green_depth':'D:/handyman-training-runs/can08-green-depth-final-verification-20261024-01/result.json',
        'current_hold':'D:/handyman-training-runs/can08-current-green-held-verification-20261024-01/result.json'},
    next=['Robot is currently holding the tilted can in the extended horizontal arm, lifted5cm. Do not reopen, repeat closure, repeat the4cm advance or run an empty-start entry.',
        'This reviewed trial verified one deeper grasp; no retraction, carry or placement was executed. Original automatic run08 stays failed.',
        'Before standard carry/place, adapt actual holding opening0.68848 and a release opening larger than that. The old0.65 release target would close further rather than open.',
        'Production guards also assume holding<=0.62 and a10cm lift; current real state is5cm above contact. Do not pass an invented compliant grip/pose or historical held context.',
        'Reacquire new current RGBD/joints before any further robot action. Grasp depth, opening and release policy require an end-to-end fresh regression before claiming full automation.'])
name='canned-deeper-grasp-verified-progress-20261024.json'
if any((p/name).exists() for p in roots):raise ValueError('new_checkpoint_required')
data=json.dumps(value,indent=2,allow_nan=False).encode()
for p in roots:
    with (p/name).open('xb') as f:f.write(data)
print(json.dumps(dict(status=value['status'],checkpoint='D:/handyman-training-runs/'+name,
    sha256=hashlib.sha256(data).hexdigest(),holding_opening_rad=opening,
    green_depth_points=value['actual_green_depth_points_per_frame'],lift_m=value['actual_green_surface_lift_m'],
    current_can_held=True,arm_retracted=False,full_transfer_verified=False)),flush=True)
