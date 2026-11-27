"""Fresh can grasp experiment with an additional 35mm forward seating.

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
deployment=read(base/'can-centerline-v1-20261024-deployment.json')
previous=read(base/'canned-centerline-fix-ready-progress-20261024.json')
hashes=dict(previous['latest_expected_hashes'],**deployment['hashes'])
cap_deployment=deployment
hashes.update(cap_deployment['hashes'])
for root in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
    if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('tested_source_changed')
root=base/'can-centerline-seat35-visible-20261024-05';root.mkdir(exist_ok=False)
meta=dict(version=1,run_id=root.name,mode='live',object='canned_juice',run_root=str(root),
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),started_ns=time.time_ns(),
    run_kind='user_visible_true_finger_midline_regrasp_after_open_and_45cm_retreat')
write_json(root/'run-meta.json',meta)
report=dict(status='starting',stage='grasp',grasp_success_verified=False,upright_grasp_verified=False,
    physical_transfer_verified=False,automatic_full_initial_transfer_verified=False,robot_commands_sent=False,
    task_finished_sent=False,maximum_close_commands=1,automatic_action_retry=False,tests=cap_deployment['tests'],
    extra_seat_m=.035,postclose_pose_is_diagnostic_only=True,
    finger_midline_base_y_offset_m=deployment['finger_midline_base_y_offset_m'],
    lateral_alignment_tolerance_m=deployment['lateral_alignment_tolerance_m'])
active=None
def interrupted(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,interrupted)
with RobotLock() as lock,(root/'journal.jsonl').open('x') as journal:
    def record(event):
        row=dict(time_ns=time.time_ns(),**event);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
    try:
        write_json(root/'result.json',report)
        command=[sys.executable,str(base/'run_visible_can_grasp_trial_20261024.py'),'--run','--require-upright',
            '--extra-seat-m','.035','--record-close-pose',
            '--simulation-ignore-collision','--output',str(root/'grasp'),
            '--owner-lock-fd',str(lock.stream.fileno()),'--owner-run-root',str(root)]
        record(dict(event='upright_grasp_intent',command=command))
        with (root/'grasp.log').open('x') as f:
            active=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,
                start_new_session=True,pass_fds=(lock.stream.fileno(),))
            for line in active.stdout:
                f.write(line);f.flush()
                if 'human_viewing_pause' in line or '"event": "stage"' in line:print(line,end='',flush=True)
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
        report['completed_ns']=time.time_ns();report['elapsed_s']=(report['completed_ns']-meta['started_ns'])/1e9
        write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if not report['grasp_success_verified']:raise SystemExit(1)
