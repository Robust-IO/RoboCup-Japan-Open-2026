"""Record the finished trial without robot communication or motion."""
import json
import time
from pathlib import Path

base = Path('/__HANDYMAN_PACKAGE__/runtime')
root = base / 'can-upright-calibrated-live-20261024-01'
navroot = base / 'can-upright-calibrated-navigation-20261024-01'
read = lambda p: json.loads(p.read_text())
result = read(root / 'result.json')
grasp = read(root / 'grasp/result.json')
navigation = read(navroot / 'result.json')
value = dict(
    status='fresh_initial_source_navigation_grasp_lift_verified_upright_quality_pending',
    saved_ns=time.time_ns(), run_root=str(root), navigation_run_root=str(navroot),
    supersedes_robot_state_record='canned-upright-reset-required-progress-20261024.json',
    source='LayoutA bedroom white_side_table', class_name='canned_juice',
    initial_start_error_m=navigation['initial_position_error_m'],
    navigation_verified=navigation['status']=='source_staging_and_head_view_verified',
    grasp_success_verified=result['grasp_success_verified'],
    upright_grasp_verified=result['upright_grasp_verified'],
    preclose_tilt_deg=result['preclose_tilt_deg'],
    afterlift_tilt_degrees=result['afterlift_tilt_degrees'],
    holding_opening_rad=result['holding_opening_rad'],
    observed_lift_m=[row['object_delta_odom_m'][2] for row in grasp['lift_verification']],
    object_palm_follow_error_m=[row['object_palm_follow_error_m'] for row in grasp['lift_verification']],
    close_command_count=1, lift_command_count=1, release_command_count=0,
    approaches=grasp['approaches'],
    held_context=result['held_context'],
    robot_state_at_verified_trial_end=result['last_fresh_robot_state'],
    can_remained_held_at_trial_end=True, arm_retracted=False,
    full_transfer_completed=False, placement_verified=False,
    navigation_parameters_changed=False, detector_changed=False,
    tests_passed=594,
    evidence_images=dict(
        source=str(root/'grasp/captures/identity-02/rgb.png'),
        preclose=str(root/'grasp/captures/preclose/rgb.png'),
        afterlift=str(root/'grasp/captures/after-lift-02/rgb.png')),
    finding='Can was upright immediately before closure; the later verified lifted pose was tilted by about 11.3 degrees. No saved intermediate pose resolves closure versus lift as the exact onset.',
    historical_state_is_not_current_motion_authorization=True,
    next=[
        'Read fresh RGBD, joints and held pose before any further motion; never repeat the grasp from this held state.',
        'If continuing a transfer, use only this run held-context and a new current held-state audit.',
        'For the next upright-quality experiment, record current can pose after closing and before lifting to distinguish the tilt onset.',
        'Do not release over the source by assuming the old support position; measure actual support for the current object pose.',
    ],
)
path = base / 'canned-upright-calibrated-live-progress-20261024.json'
with path.open('x') as stream:
    json.dump(value, stream, indent=2, allow_nan=False)
print(json.dumps(dict(saved=str(path), grasp_success_verified=value['grasp_success_verified'],
    upright_grasp_verified=value['upright_grasp_verified'],
    preclose_tilt_deg=value['preclose_tilt_deg'], afterlift_tilt_degrees=value['afterlift_tilt_degrees'])))
