"""Offline adapters: build invocation plans; never import ROS or launch commands."""
import math

SOURCE = 'white_side_table#living_room'
DESTINATION = 'square_low_table#living_room'
AUTOGRASP = '/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/workspace/training/session1/grasp_runtime/launch_autograsp.py'
NAVIGATION = '/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/navigate_carry_trial.py'


def furniture_candidates(environment, furniture_id):
    """Preserve importer priority; do not substitute Euclidean nearest for routing."""
    name, room = furniture_id.split('#', 1)
    candidates = []
    for item in environment.get('destinations', {}).get(name, []):
        if item.get('room') != room:
            continue
        pose = [float(item[k]) for k in ('x', 'y', 'yaw')]
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('nonfinite_navigation_pose')
        candidates.append(pose)
    if not candidates:
        raise ValueError('no_candidates_for_' + furniture_id)
    return candidates


def transfer_config(base, environment, environment_path, environment_sha256):
    """Copy shared configuration, replacing all apple/bedroom task fields."""
    from copy import deepcopy
    cfg = deepcopy(base)
    cfg.update(profile='session1_living_room_dog_to_square_table', layout='LayoutA',
               task=dict(object='dog_doll', source_room='living_room',
                         destination='square_low_table'),
               source_table_id=SOURCE, destination_id=DESTINATION,
               destination_room='living_room', environment=environment_path,
               environment_sha256=environment_sha256,
               source_staging_goal=furniture_candidates(environment, SOURCE)[0],
               destination_staging_goal=furniture_candidates(environment, DESTINATION)[0])
    return cfg


def navigation_argv(goal, output, *, holding_opening, waypoints=()):
    """Caller supplies current feedback and existing route waypoints, not history."""
    if holding_opening is None or not math.isfinite(float(holding_opening)):
        raise ValueError('current_gripper_feedback_required')
    argv = ['python3', NAVIGATION, '--run', '--goal', *map(str, goal),
            '--holding-opening', str(holding_opening), '--output', str(output)]
    for waypoint in waypoints:
        argv += ['--waypoint', *map(str, waypoint)]
    return argv


def grasp_argv(config, output, *, execute=False):
    argv = ['python3', AUTOGRASP, '--target', 'dog_doll', '--source-table', SOURCE,
            '--method', 'templates', '--config', str(config), '--output', str(output)]
    if execute:
        argv.append('--execute')
    return argv


def grasp_result(report,*,target='dog_doll'):
    """Translate real lift evidence; a selected plan is not a completed grasp.

    Live consumer must associate report path with its current operation token.
    This function alone does not establish freshness or operation ownership.
    """
    evidence = report.get('lift_evidence') or {}
    from grasp_runtime.autograsp.grip_confirmation import accepted
    success = report.get('status') == 'grasp_verified' and accepted(evidence)
    if success:
        return dict(status='ok', target=target, grasp_verified=True)
    return dict(status='incomplete', reason='grasp_not_verified', raw_status=report.get('status'))
