"""Record the operator's explicit success confirmation. No ROS imports/actions."""
import hashlib,json,time
from pathlib import Path

base=Path('/__HANDYMAN_PACKAGE__/runtime')
run=base/'horizontal-can-hand-grasp-20261101-04'
result=json.loads((run/'result.json').read_text())
if result['close_commands_sent']!=1 or result['release_commands_sent']!=0:
    raise ValueError('review_one_close_zero_release_trial_before_recording')
prior=json.loads((base/'phase5-horizontal-can-progress-20261101.json').read_text())
files=['horizontal_can_geometry.py','horizontal_can_grasp_plan.py','horizontal_can_trial.py',
       'horizontal_can_hand.py','horizontal_can_hand_trial.py']
checkpoint=dict(version=1,saved_ns=time.time_ns(),
    status='horizontal_can_grasp_and_lift_operator_confirmed',
    operator_confirmation='不用看了，你成功加到了',
    verification_source='explicit_user_observation',
    operator_confirmed_grasp_success=True,automatic_grasp_verification_completed=False,
    automatic_trial_report_retained_unchanged=True,
    automatic_verification_stop_reason=result['reason'],
    object='canned_juice',pose='horizontal',source_table='white_side_table#bedroom_1',
    source_room='bedroom',planned_destination='square_low_table#living_room',
    closed_commands_total=1,release_commands_total=0,
    held_state='keep_grip_do_not_replay_grasp_or_release',
    last_observed_robot_state=result['last_robot_state'],
    holding_opening_rad=result['holding_opening_rad'],
    successful_action_trial=str(run),
    predecessor_trials=[str(base/name) for name in ['horizontal-can-grasp-20261101-01',
        'horizontal-can-grasp-20261101-02','horizontal-can-hand-grasp-20261101-03']],
    scope='First operator-confirmed top-down horizontal-can grasp/lift. Not yet a complete transfer or an autonomous regression success.',
    tests_passed=16,
    latest_expected_hashes=prior['latest_expected_hashes'],
    candidate_sha256={name:hashlib.sha256((base/name).read_bytes()).hexdigest() for name in files},
    code_location='Standalone D-drive trial; not integrated into the canonical object transfer entry.',
    branch_at_last_launch=json.loads((base/'horizontal-can-hand-grasp-20261101-04-launch.json').read_text())['branch'],
    branch_name_confirmation_pending=True,
    next_work=['Preserve grip and obtain a current-state audit before any further motion.',
               'Extend held-can transport/placement to the top-down grasp without replaying the grasp.',
               'Make low-texture visual lift verification use appropriate geometric/appearance evidence; preserve this automatic false negative.'],
    robot_commands_sent_by_this_recorder=False)
for name in ('phase5-horizontal-can-held-progress-20261101.json','phase5-current-work-progress-20261101.json'):
    with (base/name).open('x') as stream:json.dump(checkpoint,stream,indent=2)
print(json.dumps(dict(status=checkpoint['status'],checkpoint=str(base/'phase5-horizontal-can-held-progress-20261101.json'),
                     original_trial_result_changed=False,robot_commands_sent=False)))
