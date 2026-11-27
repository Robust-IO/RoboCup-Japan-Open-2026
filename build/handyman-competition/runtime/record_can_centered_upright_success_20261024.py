"""Persist the verified centered, original-depth can grasp and held state."""
import json
import time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=base/'can-centerline-baseline-depth-visible-20261024-07'
read=lambda p:json.loads(p.read_text())
result=read(root/'result.json');grasp=read(root/'grasp/result.json');context=read(root/'held-context.json')
if result['status']!='upright_grasp_and_lift_verified_can_remains_held':raise ValueError('upright_success_required')
previous=read(base/'canned-centerline-fix-ready-progress-20261024.json')
value=dict(status='centered_original_depth_upright_can_grasp_lift_verified',saved_ns=time.time_ns(),
    supersedes_robot_state_records=['canned-centerline-fix-ready-progress-20261024.json',
        'canned-extra-seat35-live-progress-20261024.json','canned-upright-calibrated-live-progress-20261024.json'],
    user_authorized_retry_after_stop=True,run_root=str(root),class_name='canned_juice',
    source='LayoutA bedroom white_side_table',grasp_success_verified=True,upright_grasp_verified=True,
    preclose_tilt_deg=result['preclose_tilt_deg'],
    postclose_before_lift_tilts_deg=[r['tilt_deg'] for r in result['postclose_pose_observations']],
    afterlift_tilt_degrees=result['afterlift_tilt_degrees'],
    observed_object_lift_m=[r['object_delta_odom_m'][2] for r in grasp['lift_verification']],
    object_palm_follow_error_m=[r['object_palm_follow_error_m'] for r in grasp['lift_verification']],
    held_opening_rad=result['holding_opening_rad'],held_context=str(root/'held-context.json'),
    held_context_run_id=context['run_id'],robot_holding_can_at_verified_trial_end=True,
    arm_retracted=False,robot_state_at_verified_trial_end=result['last_fresh_robot_state'],
    new_close_commands=1,lift_commands=1,release_commands=0,
    source_identity_from_this_run=True,old_world_target_coordinates_not_replayed=True,
    extra_seat_m=0.,extra_35mm_cancelled_by_user=True,
    finger_midline_base_y_offset_m=result['finger_midline_base_y_offset_m'],
    lateral_alignment_tolerance_m=result['lateral_alignment_tolerance_m'],
    default_can_entry_uses_corrected_midline_and_zero_extra_seating=True,
    viewing_pauses_s=6.,viewing_pauses_in_temporary_wrapper_only=True,
    physical_transfer_verified=False,placement_verified=False,initial_start_full_transfer_verified=False,
    statistical_grasp_success_rate_established=False,navigation_parameters_changed=False,model_changed=False,
    tests_passed=result['tests'],latest_expected_hashes=previous['latest_expected_hashes'],
    reset_before_trial=str(base/'can-centerline-open-retreat-20261024-02/result.json'),
    earlier_centered_35mm_trial=str(base/'can-centerline-seat35-visible-20261024-05'),
    evidence_images=dict(preclose=str(root/'grasp/captures/preclose/rgb.png'),
        postclose=str(root/'grasp/captures/postclose-before-lift-02/rgb.png'),
        afterlift=str(root/'grasp/captures/after-lift-02/rgb.png')),
    historical_state_is_not_current_motion_authorization=True,
    next=['Reacquire fresh RGBD/TF/joints before any continuation; do not repeat closure from this held state.',
        'If continuing carry/place, use only this run held-context; old root03 held contexts are obsolete.',
        'Keep actual two-finger midline alignment and zero additional seating; do not re-enable the rejected 35mm insertion.',
        'If releasing, measure actual current tabletop support and can bottom first.',
        'This verifies one grasp and lift from bedroom staging, not the full initial-position transfer or an 80% success rate.'])
path=base/'canned-centered-upright-grasp-success-progress-20261024.json'
with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
print(json.dumps(dict(saved=str(path),status=value['status'],lift_m=value['observed_object_lift_m'],
    preclose_tilt_deg=value['preclose_tilt_deg'],afterlift_tilt_degrees=value['afterlift_tilt_degrees'],
    robot_holding_can=True)))
