"""Save completed offline deployment evidence; no ROS or robot connection."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

base = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
repo = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
previous_name = 'canned-initial-return-continuous-success-progress-20261024-09.json'
deployment_name = 'pepper-transfer-v1-20261024-deployment.json'
previous = read(base / previous_name)
deployment = read(base / deployment_name)
validation = base / 'pepper-transfer-v1-20261024-validation'
replay = read(validation / 'real-rgbd-replay.json')
if (deployment['status'] != 'ground_pepper_adapter_deployed_offline_verified_live_manipulation_pending'
        or deployment['tests_passed'] != 629
        or deployment['actual_saved_rgbd_frames_verified'] != 6
        or deployment['robot_commands_sent']):
    raise ValueError('completed_offline_deployment_evidence_required')
latest = dict(previous['latest_expected_hashes'], **deployment['hashes'])
roots = [base.parent / 'handyman-tools/apple-transfer-auto-v1', repo / 'training',
         repo / 'training/automation/apple_transfer_v1']
for root in roots:
    for name, digest in latest.items():
        if sha(root / name) != digest:
            raise ValueError('deployed_code_changed:' + str(root / name))

local_notes = read(base / 'pepper-local-full-suite-summary.json')
notes = dict(local_full_workspace=local_notes,
    local_workspace_result_is_not_all_green=True,
    unchanged_unity_exporter_source_assertions_failed=3,
    local_sandbox_unix_socket_permission_errors=4,
    scope_of_remote_green_suite='deployed manipulation runtime package',
    remote_runtime_tests_passed=629,
    new_pepper_tests_passed=27,
    targeted_local_pepper_and_can_tests_passed=73,
    runtime_socket_tests_passed_on_target_wsl=True,
    unrelated_unity_exporter_not_modified=True)
notes_path = validation / 'validation-notes.json'
with notes_path.open('x') as stream:
    json.dump(notes, stream, indent=2, ensure_ascii=False, allow_nan=False)

name = 'ground-pepper-adapter-ready-progress-20261024.json'
value = dict(
    status='ground_pepper_offline_adapter_ready_live_manipulation_pending',
    saved_ns=time.time_ns(),
    branch=subprocess.check_output(['git', 'branch', '--show-current'], cwd=repo, text=True).strip(),
    class_name='ground_pepper', canonical_class_id=11,
    source='LayoutA bedroom white_side_table#bedroom_1',
    destination='living_room square_low_table#living_room',
    deployment_file=str(base / deployment_name),
    validation_directory=str(validation),
    validation_notes=str(notes_path),
    archived_real_rgbd_frames=6,
    reference_manifest=str(base / 'pepper-offline-reference-v1-20261024/manifest.json'),
    reference_archive_sha256='d8923507eacfd1c01a8273516a8049f4dd6b3ec758cf01f02ac7ff9342947711',
    real_rgbd_replay_sha256=sha(validation / 'real-rgbd-replay.json'),
    observed_height_range_m=deployment['height_range_m'],
    observed_body_radius_range_m=deployment['body_radius_range_m'],
    source_observation_is_not_live_grasp_validation=True,
    geometry='separate actual-depth wide-body and cap fits; body center is grasp datum',
    zero_depth_is_not_xyz=True,
    near_clip_rgb_continuation_requires_preceding_actual_rgbd_lift_verification=True,
    finite_close_and_release_are_not_replayed=True,
    source_identity_confirmation_threshold=0.8,
    alignment_lateral_tolerance_m=0.004,
    extra_forward_seating_m=0.0,
    model_and_map_audit=deployment['model_and_map_audit'],
    navigation_parameters_changed=False,
    original_apple_can_profiles_unchanged=True,
    tests_passed=629,
    targeted_local_tests_passed=73,
    robot_commands_sent_during_this_development=False,
    unity_required_during_this_development=False,
    live_grasp_verified=False,
    live_carry_verified=False,
    live_placement_verified=False,
    statistical_success_rate_established=False,
    task_finished_sent=False,
    last_verified_robot_state_record=str(base / previous_name),
    last_verified_robot_state=previous['robot_state_at_verified_completion'],
    last_verified_robot_state_description='empty, open gripper, tucked in living room after successful can run09',
    current_live_robot_state_not_queried_during_offline_development=True,
    historical_state_is_not_new_motion_authorization=True,
    latest_expected_hashes=latest,
    first_live_preparation=[
        'Start or resume Session1/LayoutA; keep Unity Play running.',
        'Robot must be empty and tucked; initial or current living-room position is acceptable.',
        'Place ground_pepper upright and fully supported on bedroom white_side_table at the earlier can source position, with finger space.',
        'Acquire fresh readiness/localization/streams before motion; do not resume old held-can context.',
        'Use a new run directory and the ground_pepper entry; preserve current route and zero extra forward seating.',
        'If interrupted with an object held, review current actual state before continuing; do not rerun empty-start entry.'
    ],
    live_entry='bash /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/run_object_transfer.sh --object ground_pepper --run --confirm-layout LayoutA --confirm-localization --simulation-ignore-collision --output /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/NEW_PEPPER_AUTO_DIRECTORY',
    next='First actual ground_pepper grasp/lift/carry/place trial on the proven fixed route; live success remains unverified.')
content=json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n'
for target in [base/name,repo/'training'/name]:
    with target.open('x') as stream:
        stream.write(content)
print(json.dumps(dict(status=value['status'],branch=value['branch'],checkpoint=str(base/name),
    files_reverified_per_copy=len(latest),tests_passed=629,robot_commands_sent=False),ensure_ascii=False))
