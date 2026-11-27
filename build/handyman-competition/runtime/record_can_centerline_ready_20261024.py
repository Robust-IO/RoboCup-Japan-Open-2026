"""Persist STOP state and the no-motion centering correction deployment."""
import json
import time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');read=lambda p:json.loads(p.read_text())
audit=read(base/'can-centerline-stop-audit-20261024-01/result.json')
manifest=read(base/'can-centerline-v1-20261024-deployment.json')
hashes=dict(read(base/'canned-upright-reset-required-progress-20261024.json')['latest_expected_hashes'])
for name in ('can-extra-seat35-20261024-deployment.json','can-cap-wrist-v1-20261024-deployment.json','can-centerline-v1-20261024-deployment.json'):
    hashes.update(read(base/name)['hashes'])
value=dict(status='user_stopped_true_finger_midline_fix_deployed_live_test_pending',saved_ns=time.time_ns(),
    supersedes_robot_state_records=['canned-extra-seat35-live-progress-20261024.json','canned-upright-calibrated-live-progress-20261024.json'],
    user_requested_stop=True,robot_commands_sent_after_user_stop=False,
    robot_holding_can=False,hand_open=True,empty_arm_horizontal=True,arm_tucked=False,
    robot_state_at_readonly_stop_audit=audit['actual_state'],current_capture=audit['current_capture'],
    can_remains_on_source_table_but_was_tipped_before_any_closure=True,
    prior_success_held_contexts_are_no_longer_current=True,
    most_recent_visible_attempt=str(base/'can-extra-seat35-visible-20261024-04'),
    most_recent_visible_attempt_sent_close=False,
    correction=dict(old_side_offset_base_m=.012144502421366304,
        new_side_offset_base_m=manifest['finger_midline_base_y_offset_m'],
        new_lateral_tolerance_m=manifest['lateral_alignment_tolerance_m'],
        finger_midline_audit=manifest['actual_midline_audit'],
        far_alignment_then_straight_approach=True,withdraw_first_if_already_close=True,
        forward_depth_and_height_not_changed_by_this_fix=True,
        extra_35mm_still_optional_experiment_not_default=True),
    latest_expected_hashes=hashes,tests_passed=manifest['tests'],live_centered_grasp_not_yet_verified=True,
    next=['User STOP remains in force: do not restart motion merely because the code is deployed.',
        'After the user resumes, reacquire fresh current RGBD, TF and joints before any motion.',
        'The current arm is EMPTY and HORIZONTAL with hand open; do not resume any old held-can carry/release.',
        'Before another upright trial, restore the source can upright and withdraw the open fingers straight backward before lateral correction.',
        'Use the actual two-finger midline and shared 4mm lateral bound for BOTH approach planning and closure acceptance.',
        'Default initial-source trial requires an empty tucked start; do not claim this current horizontal pose is a default initial pose.'])
path=base/'canned-centerline-fix-ready-progress-20261024.json'
with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
print(json.dumps(dict(saved=str(path),status=value['status'],tests=value['tests_passed'],robot_holding_can=False)))
