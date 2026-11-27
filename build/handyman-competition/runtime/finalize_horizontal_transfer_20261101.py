"""Archive the staged verified result, leaving action journals untouched."""
import hashlib,json,time
from pathlib import Path

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
carry_root=base/'horizontal-can-carry-20261101-01'
place_root=base/'horizontal-can-place-20261101-01'
carry=json.loads((carry_root/'result.json').read_text())
place=json.loads((place_root/'result.json').read_text())
grasp=json.loads((base/'horizontal-can-hand-grasp-20261101-04/result.json').read_text())
confirmation=json.loads((base/'phase5-horizontal-can-held-progress-20261101.json').read_text())
if (not carry['carry_destination_verified'] or not place['placement_success_verified']
        or not place['robot_ready_for_next_task'] or place['release_commands_sent']!=1
        or grasp['close_commands_sent']!=1 or grasp['release_commands_sent']!=0):
    raise ValueError('confirmed_grasp_carry_actual_placement_and_cleanup_required')
files=['horizontal_can_geometry.py','horizontal_can_grasp_plan.py','horizontal_can_trial.py',
       'horizontal_can_hand.py','horizontal_can_hand_trial.py','horizontal_can_transport_geometry.py',
       'horizontal_can_carry.py','horizontal_can_place.py','test_horizontal_can_hand.py',
       'test_horizontal_can_transport.py','test_horizontal_can_geometry.py','test_horizontal_can_grasp_plan.py']
source=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/unity-reference/Assets/Competition/Handyman/Scripts/HandymanModeratorTool.cs')
script=source.read_text();lines=script.splitlines()
audit=dict(source=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    matching_lines=[dict(line=i+1,text=line.strip()) for i,line in enumerate(lines)
                    if 'graspablePositionInfo.eulerAngles' in line or 'pair.Value.transform.eulerAngles = pair.Key.eulerAngles' in line],
    interpretation='Candidate selection is shuffled, but the FULL candidate-point Euler orientation is copied, not just yaw. Horizontal initialization is possible when configured that way; not guaranteed in every session.',
    official_rule='https://github.com/RoboCupAtHomeJP/AtHome2026/blob/main/rules/S-OPL/hm_ja.md',
    rule_scope='2026 task rules initialize positions and poses; no always-upright promise was found in this page.')
checkpoint=dict(version=1,saved_ns=time.time_ns(),status='horizontal_can_staged_grasp_carry_place_verified',
    object='canned_juice',initial_object_pose='horizontal',final_object_pose='upright_on_destination',
    source_table='white_side_table#bedroom_1',destination='square_low_table#living_room',
    current_robot_state=place['final_state'],robot_ready_for_next_task=True,object_remains_held=False,
    close_commands_total=1,release_commands_total=1,
    grasp_success_verified_by_operator=True,automatic_grasp_verification_originally_completed=False,
    original_grasp_verification_false_negative='insufficient_same_can_texture_matches',
    carry_destination_verified=True,placement_success_verified=True,physical_transfer_verified=True,
    single_uninterrupted_run=False,grasp_human_confirmation_used=True,
    source_navigation=str(base/'horizontal-can-source-20261101-01'),
    grasp_trial=str(base/'horizontal-can-hand-grasp-20261101-04'),carry_trial=str(carry_root),place_trial=str(place_root),
    stage_times_s=dict(carry_including_retraction=carry['elapsed_s'],placement_and_cleanup=place['elapsed_s']),
    total_competition_runtime_not_established=True,
    placement_evidence=place['placement_verification'],
    navigation_profile='carry_fast_v1',navigation_route_changed=False,canonical_model_changed=False,
    tests_passed=21,task_finished_sent=False,
    latest_expected_hashes=confirmation['latest_expected_hashes'],
    candidate_sha256={f:hashlib.sha256((base/f).read_bytes()).hexdigest() for f in files},
    canonical_runtime_not_replaced=True,code_location='D:/handyman-training-runs; local patch mirror',
    new_branch_name_confirmation_pending=True,
    scope='One staged, operator-confirmed horizontal-can grasp followed by verified carry and independent actual RGBD placement. Not an autonomous one-command pass, statistical success rate, arbitrary-object proof or Moderator integration.',
    initialization_orientation_audit=audit,
    next_work=['Unify the tested head-to-hand transition and transport into a single entry without requiring human grasp confirmation.',
               'Retain explicit known-size RGB provenance; do not label it raw depth.',
               'Replace the low-texture SIFT-only lift false-negative with suitable temporal shape evidence.',
               'Test one flat representative object after this category is integrated; no need to exhaust all 30 classes.'])
with (base/'phase5-horizontal-can-complete-progress-20261101.json').open('x') as stream:json.dump(checkpoint,stream,indent=2)
current=base/'phase5-current-work-progress-20261101.json'
temporary=current.with_name(current.name+'.pending');temporary.write_text(json.dumps(checkpoint,indent=2));temporary.replace(current)
with (base/'horizontal-can-orientation-rule-audit-20261101.json').open('x') as stream:json.dump(audit,stream,indent=2)
print(json.dumps(dict(status=checkpoint['status'],robot_ready_for_next_task=True,
    carry_s=carry['elapsed_s'],place_s=place['elapsed_s'],close_commands_total=1,release_commands_total=1,
    robot_commands_sent_by_finalizer=False,checkpoint=str(base/'phase5-horizontal-can-complete-progress-20261101.json'))))
