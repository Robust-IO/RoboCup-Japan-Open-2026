"""One-command reviewed apple transfer; default is a no-motion plan.

Not a Moderator client, map-localization service or generic object executor.
An interrupted journal is evidence for diagnosis, never a resume instruction.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import time
import uuid

from apple_transfer_auto_plan import file_hash, validate_config


STAGES = (
    'preflight', 'navigate_source', 'head_view', 'observe_source', 'prepare_grasp',
    'grasp_and_lift', 'verify_grasp', 'retract_holding', 'navigate_destination',
    'observe_surface', 'plan_placement', 'prepare_placement', 'approach_placement',
    'check_release', 'release', 'withdraw', 'verify_placement',
)
PLACEMENT_CONTINUATION_STAGES = ('continuation_preflight','approach_placement','check_release','release','withdraw','verify_placement')


def write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.pending')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


class RobotLock:
    """Serialize this automation even when callers choose different output paths."""
    def __init__(self, path='/tmp/handyman-apple-transfer-automation.lock'):
        self.path = path

    def __enter__(self):
        self.stream = open(self.path, 'a+')
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.stream.close()
            raise RuntimeError('another_transfer_automation_is_running') from None
        self.stream.seek(0)
        self.stream.truncate()
        self.stream.write(str(os.getpid()))
        self.stream.flush()
        return self

    def __exit__(self, *_):
        self.stream.close()


def check_inputs(config):
    """File-only checks. Never imports ROS or contacts any robot service."""
    import yaml
    from motion_speed import PROFILE_PATH
    validate_config(config)
    environment = yaml.safe_load(Path(config['environment']).read_text())
    if file_hash(config['environment']) != config['environment_sha256'] or environment['internal_name'] != config['layout']:
        raise ValueError('reviewed_environment_changed')
    semantic = json.loads(Path(config['semantic_map']).read_text())
    for key, room in (('source_table_id', 'bedroom'), ('destination_id', 'living_room')):
        items = [x for x in semantic['destinations'] if x['id'] == config[key] and x['room'] == room]
        if len(items) != 1:
            raise ValueError('missing_or_ambiguous_furniture:' + key)
    for source, destination in (('living_room', 'bedroom'), ('bedroom', 'living_room')):
        if not any(r['from'] == source and r['to'] == destination and r['waypoints'] for r in environment['routes']):
            raise ValueError('reviewed_doorway_route_missing')
    registry = json.loads(Path(config['registry']).read_text())
    if file_hash(registry['weights']) != registry['sha256']:
        raise ValueError('model_hash_mismatch')
    if not Path(config['model_python']).is_file():
        raise ValueError('model_python_missing')
    return dict(status='input_files_verified', environment_sha256=file_hash(config['environment']),
                semantic_map_sha256=file_hash(config['semantic_map']), model_sha256=registry['sha256'],
                near_grasp_roi_confirmation=config['policy'].get('near_grasp_roi_confirmation',False),
                motion_profile=config.get('motion_profile','baseline'),motion_profiles_sha256=file_hash(PROFILE_PATH),
                robot_commands_sent=False, scope='File checks only; live sensor and localization readiness not established')


def run(config, task_id, output, backend, *, placement_only=False):
    validate_config(config)
    if backend.mode not in ('offline_test', 'live'):
        raise ValueError('unsupported_backend_mode')
    if placement_only and getattr(backend,'placement_continuation_authorized',False) is not True:
        raise ValueError('reviewed_placement_context_required')
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,100}', task_id):
        raise ValueError('invalid_task_id')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    report = dict(status='running', task_id=task_id, profile=config['profile'], confirmed_stages=[],
                  task_finished_sent=False, does_not_exist_sent=False, automatic_action_retry=False,
                  automatic_resume=False, mode=backend.mode)
    report['run_kind']='reviewed_placement_continuation' if placement_only else 'full_transfer'
    with (output / 'journal.jsonl').open('x') as stream:
        def record(event):
            event = dict(time_ns=time.time_ns(), **event)
            stream.write(json.dumps(event, allow_nan=False) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        try:
            backend.bind(output, task_id, record)
            record(dict(event='begin', task_id=task_id, profile=config['profile'], mode=backend.mode))
            write_json(output / 'config.snapshot.json', config)
            for index, stage in enumerate(PLACEMENT_CONTINUATION_STAGES if placement_only else STAGES):
                operation_id = f'{task_id}:{index}:{stage}'
                report['stage'] = stage
                record(dict(event='intent', stage=stage, operation_id=operation_id))
                write_json(output / 'status.json', report)
                print(json.dumps(dict(stage=stage, status='starting')), flush=True)
                result = backend.perform(stage)
                if (stage=='withdraw' and 'release' in report['confirmed_stages']
                        and isinstance(result,dict) and result.get('verified') is False
                        and result.get('deferred_to_object_verification') is True
                        and result.get('cleanup_required') is True and result.get('evidence')):
                    report['cleanup_warning']=result.get('reason','withdrawal_incomplete')
                    record(dict(event='cleanup_incomplete',stage=stage,result=result))
                    continue
                if not isinstance(result, dict) or result.get('verified') is not True or not result.get('evidence'):
                    raise ValueError('stage_evidence_not_verified:' + stage)
                record(dict(event='confirmed', stage=stage, result=result))
                report['confirmed_stages'].append(stage)
                if stage=='verify_placement':
                    report['robot_ready_for_next_task']=bool(result.get('robot_ready_for_next_task',True)
                                                           and not report.get('cleanup_warning'))
            verified_status='reviewed_placement_continuation_verified' if placement_only else 'automatic_transfer_verified'
            ready=report['robot_ready_for_next_task']
            report.update(status=(verified_status if ready else verified_status+'_cleanup_required') if backend.mode == 'live' else 'offline_test_complete',
                          placement_verified=True,task_objective_completed=True,
                          requires_fresh_state_audit=not ready)
            record(dict(event='verified_complete', task_finished_sent=False))
        except (Exception, KeyboardInterrupt) as exc:
            report.update(status='stopped', reason=str(exc) or 'interrupted', placement_verified=False,task_objective_completed=False,
                          requires_fresh_state_audit=True)
            record(dict(event='stopped', stage=report.get('stage'), reason=report['reason'], automatic_retry=False))
        finally:
            try:
                backend.close()
            except Exception as exc:
                report['cleanup_error'] = str(exc)
                report.update(status='placement_verified_cleanup_required' if report.get('placement_verified') else 'stopped',
                              robot_ready_for_next_task=False,requires_fresh_state_audit=True)
            write_json(output / 'status.json', report)
            write_json(output / 'result.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).parent / 'config/apple_transfer_auto_v1.json')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check-inputs', action='store_true', help='File-only preflight; no ROS needed')
    mode.add_argument('--run', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--task-id')
    parser.add_argument('--confirm-layout')
    parser.add_argument('--confirm-localization', action='store_true')
    parser.add_argument('--simulation-ignore-collision', action='store_true')
    args = parser.parse_args()
    config = validate_config(json.loads(args.config.read_text()))
    if args.check_inputs:
        print(json.dumps(check_inputs(config), ensure_ascii=False))
        return
    if not args.run:
        print(json.dumps(dict(status='plan_only', profile=config['profile'], stages=STAGES,
                              motion_profile=config.get('motion_profile','baseline'),
                              robot_commands_sent=False, navigation_parameters_changed=False,
                              requires='explicit run, confirmed LayoutA/localization, fresh empty tucked robot, Unity domain71, new D-drive output'), ensure_ascii=False))
        return
    if (args.confirm_layout != config['layout'] or not args.confirm_localization or not args.simulation_ignore_collision
            or os.environ.get('ROS_DOMAIN_ID') != '71' or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        parser.error('run requires reviewed LayoutA/localization, explicit simulation override, domain71 and localhost wrapper')
    if args.output is None or not args.output.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs')):
        parser.error('run output must be a new directory under /mnt/d/handyman-training-runs')
    task_id = args.task_id or 'apple-auto-' + uuid.uuid4().hex
    from apple_transfer_auto_backend import LiveBackend
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupted)
    with RobotLock():
        inputs = check_inputs(config)
        result = run(config, task_id, args.output, LiveBackend(config, input_audit=inputs))
    print(json.dumps(result, ensure_ascii=False))
    if result.get('task_objective_completed') is not True:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
