"""Navigate to the existing source table and record current target RGBD, without grasping."""
from competition_paths import is_run_path
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import uuid

code = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
sys.path.insert(0, str(code))
from apple_transfer_automation import RobotLock, check_inputs, write_json
from apple_transfer_auto_backend import LiveBackend
from apple_worker_client import Worker
from pepper_runtime import PepperRuntime
from outcome_transfer_runtime import OutcomeRuntime
from head_joint_feedback import TaskHeadFeedback
class SourceRuntime(OutcomeRuntime,TaskHeadFeedback,PepperRuntime):
    default_observation_profile="rgbd"

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--capture-only', action='store_true')
parser.add_argument('--object', choices=('cigarette',), required=True)
args = parser.parse_args()
root = args.output
base = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
if not is_run_path(root.resolve()):
    raise ValueError('D_drive_output_required')
expected = json.loads((base / 'phase5-current-work-progress-20261026.json').read_text())['latest_expected_hashes']
for name, sha in expected.items():
    if hashlib.sha256((code / name).read_bytes()).hexdigest() != sha:
        raise ValueError('reviewed_code_changed:' + name)
cfg = json.loads((code / 'config/apple_transfer_auto_v1.json').read_text())
audit = check_inputs(cfg)
# Same navigation feedback policy used by the successful pepper/penguin runs.
cfg.update(joint_feedback_max_age_s=.6, navigation_joint_feedback_max_age_s=.85,
           navigation_feedback_recovery=True)
report = dict(status='starting', object=args.object, requested_pose='flat_box',
              close_commands_sent=0, release_commands_sent=0,
              physical_transfer_verified=False, navigation_motion_profile=cfg['navigation_motion_profile'])
runtime = backend = worker = None
started = time.time_ns()
with RobotLock():
    root.mkdir(exist_ok=False)
    with (root / 'journal.jsonl').open('x') as log:
        def record(event):
            log.write(json.dumps(dict(time_ns=time.time_ns(), **event)) + '\n')
            log.flush()
        def stage(name):
            report['stage'] = name
            write_json(root / 'result.json', report)
        try:
            epoch = args.object + '-source-' + uuid.uuid4().hex
            backend = LiveBackend(cfg)
            backend.bind(root, epoch, record)
            backend.stage = 'preflight'
            write_json(root / 'input-audit.json', audit)
            import rclpy
            rclpy.init()
            runtime = SourceRuntime(root, epoch, record)
            if not args.capture_only:
                stage('prepare_navigation_from_current_state')
                runtime.prepare_navigation_from_current()
                stage('preflight')
                ready = backend.command([code / 'inspect_transfer_ready.py', '--output', root / 'preflight.json',
                    '--efficient-transfer', '--joint-feedback-max-age-s', '.6'], root / 'preflight.json', 'ready', True, None)
                backend.start_room = backend.room_at(ready['base_map_pose'])
                backend.initial_opening = ready['joints']['hand_motor_joint']
                report['initial_base_map_pose'] = ready['base_map_pose']
                stage('navigate_source')
                backend.stage = 'navigate_source'
                backend.navigate_source()
                stage('head_view')
                backend.stage = 'head_view'
                backend.head_view()
            stage('observe_target_at_source')
            worker = Worker(cfg['model_python'], cfg['registry'], root / 'model.json', imgsz=640)
            report['model'] = worker.ready
            report['frames'] = []
            for index in range(3):
                sample = runtime.sample(processing_reserve_ns=0)
                capture = runtime.save(sample, f'{index:03d}-source')
                result = worker.infer(capture, pump=runtime.pump)
                write_json(capture / 'detections.json', result)
                report['frames'].append(dict(capture=str(capture), inference=result,
                    depth_stamp_ns=sample['snapshot']['streams']['depth']['stamp_ns']))
                write_json(root / 'result.json', report)
            report.update(status='source_navigation_and_three_rgbd_observations_complete',
                          last_robot_state=runtime.state(), grip_unchanged=True)
        except Exception as exc:
            report.update(status='stopped_requires_actual_state_review', reason=str(exc))
        finally:
            if worker:
                worker.close()
            if runtime:
                runtime.shutdown()
                rclpy.shutdown()
            if backend:
                backend.close()
            report['elapsed_s'] = (time.time_ns() - started) / 1e9
            write_json(root / 'result.json', report)
            print(json.dumps(report))
