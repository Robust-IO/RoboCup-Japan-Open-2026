"""Reviewed carry/place continuation of the currently verified upright grasp.

Uses this run's held context and fresh adapter observations. No new grasp,
historical motion replay, navigation-parameter edits or automatic action retry.
"""
import hashlib
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_backend import stop_group
from can_transfer_context import load_context

base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'can-centerline-baseline-depth-visible-20261024-07'
out=root/'transfer-continuation';context=root/'held-context.json'
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
checkpoint=read(base/'canned-centered-upright-grasp-success-progress-20261024.json')
repo=Path('/__HANDYMAN_PACKAGE__/project')
for checked in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
    if any(sha(checked/n)!=digest for n,digest in checkpoint['latest_expected_hashes'].items()):
        raise ValueError('tested_source_changed')
load_context(context,root/'carry')
active=None
def interrupted(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,interrupted)
with RobotLock() as owner:
    if (root/'carry').exists() or (root/'place').exists():raise ValueError('carry_or_place_already_attempted_do_not_replay')
    out.mkdir(exist_ok=False);started=time.time_ns()
    report=dict(status='running',run_id=root.name,run_root=str(root),reviewed_continuation=True,
        new_grasp_commands=0,physical_transfer_verified=False,uninterrupted_initial_start_transfer_verified=False,
        navigation_parameters_changed=False,model_changed=False,task_finished_sent=False,
        automatic_action_retry=False,confirmed_stages=[])
    with (out/'journal.jsonl').open('x') as journal:
        def record(event):
            row=dict(time_ns=time.time_ns(),**event);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
        try:
            for stage,script,flags,expected in (
                ('carry','run_canned_carry_trial.py',['--navigate-destination'],'can_retraction_and_destination_navigation_verified'),
                ('place','run_canned_place_trial.py',[],'reviewed_can_placement_verified')):
                report['stage']=stage;write_json(out/'result.json',report)
                command=[sys.executable,str(code/script),'--run','--context',str(context),'--output',str(root/stage),
                    '--owner-lock-fd',str(owner.stream.fileno()),'--owner-run-root',str(root),*flags]
                record(dict(event='intent',stage=stage,command=command))
                with (out/(stage+'.log')).open('x') as log:
                    active=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,
                        start_new_session=True,pass_fds=(owner.stream.fileno(),))
                    for line in active.stdout:
                        log.write(line);log.flush()
                        if '"event": "stage"' in line:print(line,end='',flush=True)
                    return_code=active.wait();active=None
                result=read(root/stage/'result.json')
                if return_code or result.get('status')!=expected:
                    raise ValueError(stage+'_stopped:'+str(result.get('reason',result.get('status'))))
                if stage=='carry' and not (result.get('retraction_verified') and result.get('destination_navigation_verified')):
                    raise ValueError('carry_not_actually_verified')
                if stage=='place' and not (result.get('placement_success_verified') and result.get('release_commands_sent')==1):
                    raise ValueError('placement_not_actually_verified')
                report['confirmed_stages'].append(stage);record(dict(event='confirmed',stage=stage,result=str(root/stage/'result.json')))
            report.update(status='centered_can_grasp_carry_placement_completed_as_reviewed_continuation',
                physical_transfer_verified=True,placement_success_verified=True,
                robot_ready_for_next_task=result.get('robot_ready_for_next_task',False),
                cleanup_completed=result.get('cleanup_completed',False))
        except (Exception,KeyboardInterrupt) as exc:
            if active:stop_group(active);active=None
            report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
            record(dict(event='stopped',stage=report.get('stage'),reason=report['reason']))
        finally:
            report['elapsed_s']=(time.time_ns()-started)/1e9
            write_json(out/'result.json',report);print(json.dumps(report),flush=True)
if not report['physical_transfer_verified']:raise SystemExit(1)
