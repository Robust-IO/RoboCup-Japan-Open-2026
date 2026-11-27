"""One fresh source navigation using the existing verified route and parameters."""
import hashlib
import json
import math
import signal
import sys
import time
from pathlib import Path

code = Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
sys.path.insert(0, str(code))
from apple_transfer_automation import RobotLock, write_json
from can_transfer_automation import configuration, live_backend

base = Path('/__HANDYMAN_PACKAGE__/runtime')
deployment = json.loads((base / 'can-extra-seat35-20261024-deployment.json').read_text())
for name, digest in deployment['hashes'].items():
    if hashlib.sha256((code / name).read_bytes()).hexdigest() != digest:
        raise ValueError('tested_calibration_changed:' + name)

root = base / 'can-extra-seat35-source-navigation-20261024-01'
config = configuration()
backend = live_backend(config)

def interrupted(*_):
    raise KeyboardInterrupt()

signal.signal(signal.SIGTERM, interrupted)
with RobotLock() as lock:
    root.mkdir(exist_ok=False)
    started = time.time_ns()
    write_json(root / 'run-meta.json', dict(version=1, run_id=root.name,
        run_root=str(root), object='canned_juice', mode='live', started_ns=started,
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        run_kind='source_navigation_only_no_grasp_or_release'))
    write_json(root / 'config.snapshot.json', config)
    report = dict(status='running', confirmed_stages=[], robot_commands_sent=False,
        navigation_parameters_changed=False, grasp_commands_sent=False,
        release_commands_sent=False, automatic_action_retry=False)
    with (root / 'journal.jsonl').open('x') as stream:
        def record(event):
            row = dict(time_ns=time.time_ns(), **event)
            stream.write(json.dumps(row) + '\n')
            stream.flush()
            print(json.dumps(row), flush=True)
        try:
            backend.bind_run(root, root.name, config, record, lock.stream.fileno())
            for stage in ('preflight', 'navigate_source', 'head_view'):
                report['stage'] = stage
                write_json(root / 'result.json', report)
                record(dict(event='intent', stage=stage))
                response = backend.perform(stage)
                if response.get('verified') is not True:
                    raise ValueError('source_stage_not_verified:' + stage)
                if stage == 'preflight':
                    report['initial_base_map_pose'] = backend.ready['base_map_pose']
                    pose = backend.ready['base_map_pose']
                    report['initial_position_error_m'] = math.hypot(pose[0]-1.184, pose[1])
                report['confirmed_stages'].append(stage)
                record(dict(event='confirmed', stage=stage, evidence=response['evidence']))
            report['status'] = 'source_staging_and_head_view_verified'
        except (Exception, KeyboardInterrupt) as exc:
            report.update(status='stopped_requires_actual_state_review', reason=str(exc) or 'interrupted')
            record(dict(event='stopped', reason=report['reason']))
        finally:
            backend.close()
            report.update(robot_commands_sent=bool(getattr(backend, 'action_intents', 0)),
                elapsed_s=(time.time_ns()-started)/1e9)
            write_json(root / 'result.json', report)
            print(json.dumps(report), flush=True)
if report['status'] != 'source_staging_and_head_view_verified':
    raise SystemExit(1)
