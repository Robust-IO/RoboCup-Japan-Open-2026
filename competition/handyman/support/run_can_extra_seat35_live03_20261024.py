"""Fresh can grasp experiment with an additional 35mm forward seating.

Owns the complete grasp stage; does not navigate, place or automatically repeat
a failed close. Successful grasp remains held for review and carry handoff.
"""
import hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
from can_transfer_context import build_context
from can_transfer_automation import configuration,live_backend
from apple_transfer_auto_backend import stop_group
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
deployment=read(base/'can-extra-seat35-20261024-deployment.json')
previous=read(base/'canned-upright-reset-required-progress-20261024.json')
hashes=dict(previous['latest_expected_hashes'],**deployment['hashes'])
cap_deployment=read(base/'can-cap-wrist-v1-20261024-deployment.json')
hashes.update(cap_deployment['hashes'])
for root in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
    if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('tested_source_changed')
root=base/'can-extra-seat35-live-20261024-03';root.mkdir(exist_ok=False)
meta=dict(version=1,run_id=root.name,mode='live',object='canned_juice',run_root=str(root),
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),started_ns=time.time_ns(),
    run_kind='fresh_initial_navigation_and_extra_35mm_can_grasp_with_refined_cap_region')
write_json(root/'run-meta.json',meta)
report=dict(status='starting',stage='grasp',grasp_success_verified=False,upright_grasp_verified=False,
    physical_transfer_verified=False,automatic_full_initial_transfer_verified=False,robot_commands_sent=False,
    task_finished_sent=False,maximum_close_commands=1,automatic_action_retry=False,tests=cap_deployment['tests'],
    extra_seat_m=.035,postclose_pose_is_diagnostic_only=True)
active=None;backend=None
def interrupted(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,interrupted)
with RobotLock() as lock,(root/'journal.jsonl').open('x') as journal:
    def record(event):
        row=dict(time_ns=time.time_ns(),**event);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
    try:
        write_json(root/'result.json',report)
        config=configuration();write_json(root/'config.snapshot.json',config)
        backend=live_backend(config);backend.bind_run(root,root.name,config,record,lock.stream.fileno())
        report['confirmed_stages']=[]
        for stage in ('preflight','navigate_source','head_view'):
            report['stage']=stage;write_json(root/'result.json',report)
            record(dict(event='intent',stage=stage))
            response=backend.perform(stage)
            if response.get('verified') is not True:raise ValueError('source_stage_not_verified:'+stage)
            report['confirmed_stages'].append(stage)
            if stage=='preflight':report['initial_base_map_pose']=backend.ready['base_map_pose']
            record(dict(event='confirmed',stage=stage,evidence=response['evidence']))
        backend.close();backend=None
        report['stage']='grasp';write_json(root/'result.json',report)
        command=[sys.executable,str(code/'run_canned_grasp_trial.py'),'--run','--require-upright',
            '--extra-seat-m','.035','--record-close-pose',
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
            postclose_pose_observations=grasp.get('postclose_pose_observations',[]),
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
        if backend:backend.close()
        report['completed_ns']=time.time_ns();report['elapsed_s']=(report['completed_ns']-meta['started_ns'])/1e9
        write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if not report['grasp_success_verified']:raise SystemExit(1)
