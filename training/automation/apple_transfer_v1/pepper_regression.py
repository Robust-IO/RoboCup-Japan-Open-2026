"""One automatic reverse reset and three forward regressions, no action replay.

Each leg launches the normal transfer entry with a new task snapshot/output.
A failed leg stops the sequence; a staged continuation never counts as a pass.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from pepper_tasks import DEFAULT_TASK,REVERSE_TASK,load_task,task_digest

STAGES=['preflight','navigate_source','head_view','grasp_lift','carry_destination','place_verify']


def sequence(rounds=3):
    if type(rounds) is not int or not 1<=rounds<=3:raise ValueError('one_to_three_rounds_required')
    legs=[dict(name='00-initial-reset',kind='automatic_reset',round=0,task_config=str(REVERSE_TASK))]
    for index in range(1,rounds+1):
        legs.append(dict(name=f'{len(legs):02d}-round-{index}',kind='forward_regression',round=index,task_config=str(DEFAULT_TASK)))
        if index<rounds:legs.append(dict(name=f'{len(legs):02d}-reset-for-{index+1}',kind='automatic_reset',round=index,task_config=str(REVERSE_TASK)))
    return legs


def verify_leg(result,task):
    if (result.get('status')!='automatic_ground_pepper_transfer_verified'
            or result.get('class_name')!='ground_pepper' or result.get('physical_transfer_verified') is not True
            or result.get('grasp_success_verified') is not True or result.get('placement_success_verified') is not True
            or result.get('robot_ready_for_next_task') is not True or result.get('cleanup_completed') is not True
            or result.get('object_remains_held') is not False or result.get('uninterrupted_transfer_verified') is not True
            or result.get('close_commands_sent')!=1 or result.get('release_commands_sent')!=1
            or result.get('confirmed_stages')!=STAGES or result.get('task_sha256')!=task_digest(task)
            or result.get('configured_task')!=task or result.get('task_finished_sent') is not False):
        raise ValueError('new_uninterrupted_complete_transfer_required_for_regression')
    return dict(elapsed_s=result['elapsed_s'],close_commands_sent=1,release_commands_sent=1,
        source=task['source']['table_id'],destination=task['destination']['table_id'],
        physical_transfer_verified=True,uninterrupted_transfer_verified=True,robot_ready_for_next_task=True)


def write(path,value):
    pending=path.with_suffix(path.suffix+'.pending')
    pending.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n');pending.replace(path)


def run(root,rounds):
    base=Path('/mnt/d/handyman-training-runs');code=Path(__file__).parent
    if not root.resolve().is_relative_to(base) or root.exists():raise ValueError('new_D_drive_output_required')
    legs=sequence(rounds);active=None
    report=dict(status='starting',requested_rounds=rounds,passed_rounds=0,legs=[],
        manual_object_reposition_required=False,automatic_manipulation_retry=False,navigation_feedback_recovery_enabled=True,task_finished_sent=False,
        untouched_unity_spawn_start_verified=False,start_kind='fresh_current_empty_pose_after_automatic_object_reset')
    lock=(base/'.pepper-regression.lock').open('a+')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:lock.close();raise ValueError('another_pepper_regression_is_running')
    started=time.time_ns()
    root.mkdir(exist_ok=False);(root/'tasks').mkdir();(root/'legs').mkdir()
    for leg in legs:
        task=load_task(leg['task_config']);path=root/'tasks'/(leg['name']+'.json')
        write(path,task);leg['task_config']=str(path);leg['task']=task
    write(root/'sequence.json',legs);write(root/'result.json',report)
    def interrupted(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    try:
        for leg in legs:
            output=root/'legs'/leg['name'];report.update(status='running',current_leg=leg['name'],current_kind=leg['kind'])
            write(root/'result.json',report)
            command=['bash',str(code/'run_object_transfer.sh'),'--object','ground_pepper','--run',
                '--task-config',leg['task_config'],'--confirm-layout','LayoutA','--confirm-localization',
                '--simulation-ignore-collision','--output',str(output)]
            with (root/(leg['name']+'.console.log')).open('x') as log:
                active=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                print(json.dumps(dict(event='leg_started',leg=leg['name'],kind=leg['kind'],output=str(output))),flush=True)
                previous=None;last_print=0
                while active.poll() is None:
                    try:
                        current=json.loads((output/'result.json').read_text())
                        progress={k:current.get(k) for k in ('stage','status','close_commands_sent','release_commands_sent','grasp_success_verified','reason')}
                        if progress!=previous or time.monotonic()-last_print>=25:
                            print(json.dumps(dict(event='leg_progress',leg=leg['name'],**progress)),flush=True)
                            previous=progress;last_print=time.monotonic()
                    except (FileNotFoundError,json.JSONDecodeError):pass
                    time.sleep(.5)
                returncode=active.returncode;active=None
            result=json.loads((output/'result.json').read_text())
            if returncode:
                report['failed_leg_result']=dict(output=str(output),stage=result.get('stage'),reason=result.get('reason'),
                    physical_transfer_verified=result.get('physical_transfer_verified'),last_fresh_robot_state=result.get('last_fresh_robot_state'))
                raise ValueError('leg_stopped:'+leg['name']+':'+str(result.get('reason')))
            verified=verify_leg(result,leg['task'])
            report['legs'].append(dict(leg=leg['name'],kind=leg['kind'],round=leg['round'],output=str(output),**verified))
            if leg['kind']=='forward_regression':report['passed_rounds']+=1
            write(root/'result.json',report)
            print(json.dumps(dict(event='leg_verified',leg=leg['name'],passed_rounds=report['passed_rounds'],**verified)),flush=True)
        report.update(status='three_forward_rounds_verified' if rounds==3 else 'requested_forward_rounds_verified',
            all_requested_rounds_verified=True,robot_ready_for_next_task=True,
            object_at_destination=legs[-1]['task']['destination']['table_id'],statistical_success_rate_established=False)
    except (Exception,KeyboardInterrupt) as exc:
        if active:
            from apple_transfer_auto_backend import stop_group
            stop_group(active);active=None
        report.update(status='stopped_requires_current_state_review',reason=str(exc) or 'interrupted',
            all_requested_rounds_verified=False)
    finally:
        report['elapsed_s']=(time.time_ns()-started)/1e9;write(root/'result.json',report)
        lock.close();print(json.dumps(report,ensure_ascii=False),flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true')
    p.add_argument('--rounds',type=int,choices=(1,2,3),default=3);p.add_argument('--output',type=Path)
    a=p.parse_args()
    if not a.run:
        print(json.dumps(dict(robot_commands_sent=False,legs=sequence(a.rounds),manual_object_reposition_required=False)));return
    if a.output is None or not a.output.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs')):
        p.error('new D-drive output required')
    result=run(a.output,a.rounds)
    if not result.get('all_requested_rounds_verified'):raise SystemExit(1)

if __name__=='__main__':main()
