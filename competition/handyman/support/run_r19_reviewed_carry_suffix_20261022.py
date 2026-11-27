"""Explicit NEW suffix trial from visually rechecked holding state; not journal replay."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

BASE=Path('/__HANDYMAN_PACKAGE__/runtime')
CODE=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
CONTROL=BASE/'apple-r19-carry-suffix-control-20261022-01'
OUT=BASE/'apple-r19-carry-to-placement-20261022-01'
OLD=BASE/'automatic-apple-r18b-20261022-01'
PROBE=BASE/'apple-r19-carry-scene-check-20261022-01'
TASK='apple-r19-reviewed-held-suffix-20261022-01'
sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,check_inputs,write_json
from apple_transfer_auto_backend import LiveBackend,read
from apple_transfer_auto_plan import map_pose
from task_outcome_policy import carry_lower_from_current

parser=argparse.ArgumentParser()
parser.add_argument('--launch-after-camera-review',action='store_true')
parser.add_argument('--run-reviewed-suffix',action='store_true')
args=parser.parse_args()
if args.launch_after_camera_review==args.run_reviewed_suffix:raise ValueError('choose_explicit_launch_or_run')
if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('reviewed_domain_required')
cfg=read(CODE/'config/apple_transfer_auto_v1.json')
expected={}
for name in ('apple-outcome-r16b-20261018-deployment.json','apple-efficiency-r17-20261022-deployment.json',
             'apple-terminal-r18-20261022-deployment.json','apple-terminal-r18b-20261022-deployment.json',
             'apple-held-carry-r19-20261022-deployment.json'):
    expected.update(read(BASE/name)['hashes'])
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
if any(sha(CODE/name)!=digest for name,digest in expected.items()):raise ValueError('reviewed_runtime_changed')
old=read(OLD/'result.json')
if old['status']!='stopped' or old['stage']!='retract_holding':raise ValueError('specific_previous_holding_trial_required')
probe=read(PROBE/'result.json')
if not probe['ready_for_explicit_current_carry_completion']:raise ValueError('idle_holding_probe_required')
if Path('/proc/sys/kernel/random/boot_id').read_text().strip()!=probe['boot_id']:raise ValueError('new_boot_requires_new_scene_check')

if args.launch_after_camera_review:
    if CONTROL.exists() or OUT.exists():raise ValueError('new_trial_required_no_replay')
    CONTROL.mkdir()
    image=Path(probe['current_capture'])/'hand.png'
    review=dict(reviewed_by='assistant',reviewed_ns=time.time_ns(),image=str(image),sha256=sha(image),
                apple_visible_in_gripper=True,description='Fresh hand camera shows the red apple between the gripper fingers.',
                robot_commands_sent=False)
    write_json(CONTROL/'current-held-image-review.json',review)
    command=[sys.executable,str(Path(__file__).resolve()),'--run-reviewed-suffix']
    with (CONTROL/'automation.log').open('x') as stream:
        child=subprocess.Popen(command,env=dict(os.environ,ROS_LOG_DIR=str(CONTROL/'ros-logs')),
                               stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    report=dict(status='new_reviewed_carry_suffix_started',pid=child.pid,task_id=TASK,output=str(OUT),
                command=command,revision='held-carry-r19',run_kind='reviewed_held_carry_to_placement_continuation',
                execution_started_ns=time.time_ns(),old_actions_replayed=False,automatic_retry=False,
                is_full_transfer=False,log=str(CONTROL/'automation.log'))
    write_json(CONTROL/'startup.json',report);print(json.dumps(report),flush=True)
    raise SystemExit(0)

def interrupted(_signum,_frame):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,interrupted)
STAGES=('current_holding_preflight','finish_carry_lowering','navigate_destination','observe_surface',
        'plan_placement','prepare_placement','approach_placement','check_release','release','withdraw','verify_placement')
with RobotLock():
    if OUT.exists():raise ValueError('new_suffix_output_required')
    review=read(CONTROL/'current-held-image-review.json')
    if not review['apple_visible_in_gripper'] or sha(Path(review['image']))!=review['sha256']:
        raise ValueError('reviewed_current_held_image_required')
    OUT.mkdir()
    backend=LiveBackend(cfg,input_audit=check_inputs(cfg))
    report=dict(status='running',task_id=TASK,revision='held-carry-r19',
                run_kind='reviewed_held_carry_to_placement_continuation',is_full_transfer=False,
                old_trial_directory=str(OLD),confirmed_stages=[],task_finished_sent=False,
                automatic_action_retry=False,automatic_resume=False,old_actions_replayed=False,
                placement_verified=False,task_objective_completed=False)
    with (OUT/'journal.jsonl').open('x') as stream:
        def record(value):
            event=dict(time_ns=time.time_ns(),**value)
            stream.write(json.dumps(event,allow_nan=False)+'\n');stream.flush();os.fsync(stream.fileno())
        backend.bind(OUT,TASK,record)
        record(dict(event='begin',task_id=TASK,run_kind=report['run_kind'],is_full_transfer=False))
        write_json(OUT/'config.snapshot.json',cfg)
        try:
            for stage in STAGES:
                report['stage']=stage;backend.stage=stage
                record(dict(event='intent',stage=stage));write_json(OUT/'status.json',report)
                print(json.dumps(dict(stage=stage,status='starting')),flush=True)
                if stage=='current_holding_preflight':
                    capture=backend.capture('current-holding')
                    snap=read(capture/'snapshot.json');pose=map_pose(snap)
                    names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
                    held=read(OLD/'verified-held-geometry.json')
                    carry_lower_from_current([snap['joints'][k] for k in names],held['holding_opening_rad'])
                    previous_pose=probe['base_map_pose']
                    if (backend.room_at(pose)!='bedroom' or math.dist(pose[:2],previous_pose[:2])>.05
                            or abs(math.atan2(math.sin(pose[2]-previous_pose[2]),math.cos(pose[2]-previous_pose[2])))>.05):
                        raise ValueError('fresh_current_holding_scene_changed')
                    if not read(OLD/'grasp/close-lift/independent-lift-verification.json')['grasp_success_verified']:
                        raise ValueError('original_lift_not_verified')
                    backend.held=held;backend.holding=held['holding_opening_rad']
                    backend.grasp_plan=read(OLD/'measured-grasp-plan.json')
                    outcome=backend.evidence('current-holding-preflight',dict(ready=True,base_map_pose=pose,
                        capture=str(capture),joints=snap['joints'],current_image_review=review,
                        original_lift_evidence=str(OLD/'grasp/close-lift/independent-lift-verification.json'),
                        held_geometry=held,robot_commands_sent=False,old_action_receipts_authorize_no_motion=True))
                elif stage=='finish_carry_lowering':
                    capture=backend.capture('before-carry-lower');snap=read(capture/'snapshot.json')
                    _,evidence=backend.arm('current-carry-lower',map_pose(snap),[
                        '--measured-contact-start-lift',backend.grasp_plan['contact_start_lift_m'],
                        '--bedroom-side-table-grasp','--contact-finish','carry',
                        '--holding-opening',backend.holding,'--carry-lower-from-current'])
                    backend.start_model_service()
                    outcome=dict(verified=True,evidence=evidence)
                else:
                    outcome=backend.perform(stage)
                if stage=='withdraw' and outcome.get('deferred_to_object_verification') is True:
                    report['cleanup_warning']=outcome.get('reason','withdrawal_incomplete')
                    record(dict(event='cleanup_incomplete',stage=stage,result=outcome));continue
                if not isinstance(outcome,dict) or outcome.get('verified') is not True or not outcome.get('evidence'):
                    raise ValueError('stage_not_verified:'+stage)
                record(dict(event='confirmed',stage=stage,result=outcome))
                report['confirmed_stages'].append(stage)
                if stage=='verify_placement':report['robot_ready_for_next_task']=outcome.get('robot_ready_for_next_task',True) and not report.get('cleanup_warning')
            report.update(status='reviewed_carry_suffix_verified' if report['robot_ready_for_next_task'] else 'reviewed_carry_suffix_verified_cleanup_required',
                          placement_verified=True,task_objective_completed=True)
            record(dict(event='verified_complete',is_full_transfer=False,task_finished_sent=False))
        except (Exception,KeyboardInterrupt) as exc:
            report.update(status='stopped',reason=str(exc) or 'interrupted',requires_fresh_state_audit=True)
            record(dict(event='stopped',stage=report.get('stage'),reason=report['reason'],automatic_retry=False))
        finally:
            try:backend.close()
            except Exception as exc:report['cleanup_error']=str(exc)
            write_json(OUT/'status.json',report);write_json(OUT/'result.json',report)
    print(json.dumps(report),flush=True)
    if report.get('task_objective_completed') is not True:raise SystemExit(1)
