"""Fresh upright grasp experiment using validated relative calibration.

Owns the complete grasp stage; does not navigate, place or automatically repeat
a failed close. Successful grasp remains held for review and carry handoff.
"""
import hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
from can_transfer_context import build_context
from apple_transfer_auto_backend import stop_group
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
deployment=read(base/'can-upright-contact-v2-20261024-deployment.json')
previous=read(base/'canned-upright-reset-required-progress-20261024.json')
hashes=dict(previous['latest_expected_hashes'],**deployment['hashes'])
for root in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
    if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('tested_source_changed')
root=base/'can-upright-calibrated-live-20261024-01';root.mkdir(exist_ok=False)
meta=dict(version=1,run_id=root.name,mode='live',object='canned_juice',run_root=str(root),
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),started_ns=time.time_ns(),
    run_kind='upright_grasp_experiment_from_current_source_staging')
write_json(root/'run-meta.json',meta)
report=dict(status='starting',stage='grasp',grasp_success_verified=False,upright_grasp_verified=False,
    physical_transfer_verified=False,automatic_full_initial_transfer_verified=False,robot_commands_sent=False,
    task_finished_sent=False,maximum_close_commands=1,automatic_action_retry=False,tests=deployment['tests'])
active=None
def interrupted(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,interrupted)
with RobotLock() as lock,(root/'journal.jsonl').open('x') as journal:
    def record(event):
        row=dict(time_ns=time.time_ns(),**event);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
    try:
        write_json(root/'result.json',report)
        command=[sys.executable,str(code/'run_canned_grasp_trial.py'),'--run','--require-upright',
            '--simulation-ignore-collision','--output',str(root/'grasp'),
            '--owner-lock-fd',str(lock.stream.fileno()),'--owner-run-root',str(root)]
        record(dict(event='upright_grasp_intent',command=command))
        with (root/'grasp.log').open('x') as f:
            active=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,pass_fds=(lock.stream.fileno(),))
            result_code=active.wait();active=None
        grasp=read(root/'grasp/result.json');report['robot_commands_sent']=grasp.get('robot_commands_sent',bool(grasp.get('preparation')))
        if result_code or grasp['status']!='can_grasp_and_lift_verified':raise ValueError('upright_grasp_stopped:'+str(grasp.get('reason')))
        measurements=[read(Path(row['capture'])/'can-measurement.json') for row in grasp['lift_verification']]
        after_tilts=[m['tilt_deg'] for m in measurements]
        report.update(grasp_success_verified=True,preclose_tilt_deg=grasp['verified_preclose_tilt_deg'],
            afterlift_tilt_degrees=after_tilts,holding_opening_rad=grasp['held_opening_rad'],
            last_fresh_robot_state=grasp['last_fresh_robot_state'])
        context=build_context(root);report['held_context']=str(context)
        report.update(status='upright_grasp_and_lift_verified_can_remains_held' if max(after_tilts)<=8.
            else 'grasp_lift_verified_postlift_orientation_requires_review',upright_grasp_verified=max(after_tilts)<=8.,
            can_remains_held=True,arm_retracted=False)
        record(dict(event='upright_grasp_completed',preclose_tilt_deg=report['preclose_tilt_deg'],afterlift_tilt_degrees=after_tilts))
    except (Exception,KeyboardInterrupt) as exc:
        if active:stop_group(active);active=None
        report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
    finally:
        report['completed_ns']=time.time_ns();report['elapsed_s']=(report['completed_ns']-meta['started_ns'])/1e9
        write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if not report['grasp_success_verified']:raise SystemExit(1)
