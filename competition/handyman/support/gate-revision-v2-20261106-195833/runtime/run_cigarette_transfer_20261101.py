"""Sequence existing automatic stages for this fresh, furniture-specific task.

No grasp/release replay and no historic target coordinates or identity fallback.
Each child owns the ordinary robot lock and fresh sensor acquisition.
"""
from competition_paths import is_run_path
import argparse,fcntl,json,os,signal,subprocess,time
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--readiness-run',type=Path,required=True);p.add_argument('--resume-at-source',action='store_true');p.add_argument('--tracked-source',type=Path);args=p.parse_args()
b=Path('/__HANDYMAN_PACKAGE__/runtime');frozen=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');root=args.output
if not is_run_path(root.resolve()) or not is_run_path(args.readiness_run.resolve()):raise ValueError('D_paths_required')
lock=Path('/tmp/handyman-cigarette-delivery-sequence.lock').open('a+');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
root.mkdir(exist_ok=False);start=time.time_ns();active=None;report=dict(status='running',request='卧室白桌香烟→客厅矮方桌',stages=[],task_finished_sent=False,automatic_stage_replay=False,operator_identity_fallback_used=False)
def save():
    report['elapsed_s']=(time.time_ns()-start)/1e9
    tmp=root/'result.tmp';tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2));tmp.replace(root/'result.json')
def running(pid):
    try:return Path('/proc',str(pid),'stat').read_text().split()[2]!='Z'
    except FileNotFoundError:return False
def run_stage(kind,launcher,extra,success):
    global active
    attempt=1
    while any((b/(f'cigarette-{kind}-20261101-{attempt:02d}'+suffix)).exists() for suffix in ('','-console.log','-launch.json')):attempt+=1
    name=f'cigarette-{kind}-20261101-{attempt:02d}';path=b/name
    report['active_stage']=kind;report[kind]=str(path);save()
    command=['/usr/bin/python3',str(b/launcher)]+([] if kind in ('source','grasp') else [kind])+['--attempt',str(attempt)]+extra
    launched=subprocess.run(command,check=True,capture_output=True,text=True)
    (root/(kind+'-launcher.log')).write_text(launched.stdout+launched.stderr)
    active=json.loads((b/(name+'-launch.json')).read_text())['pid']
    while running(active):time.sleep(.5)
    active=None;r=json.loads((path/'result.json').read_text())
    report['stages'].append(dict(stage=kind,run=str(path),status=r.get('status'),reason=r.get('reason'),elapsed_s=r.get('elapsed_s'),
        close_commands_sent=r.get('close_commands_sent',0),release_commands_sent=r.get('release_commands_sent',0)));save()
    if not success(r):raise RuntimeError(kind+':'+str(r.get('reason',r.get('status'))))
    return path,r
def stop(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop);save()
try:
    ready=json.loads((args.readiness_run/'result.json').read_text())
    if ready.get('status')!='fresh_state_observed':raise ValueError('fresh_readiness_not_observed')
    if any(row['publishers'] or row['subscribers']!=1 for row in ready['command_topics'].values()):raise ValueError('control_channels_not_exclusive')
    cfg=json.loads((frozen/'config/apple_transfer_auto_v1.json').read_text())
    if cfg['source_table_id']!='white_side_table#bedroom_1' or cfg['destination_id']!='square_low_table#living_room':raise ValueError('task_configuration_does_not_match_requested_furniture')
    report['initial_state']=ready['state'];save()
    entry=ready.get('entry',dict(mode='new_pick'))
    if entry['mode']=='continue_held':
        grasp=Path(entry['grasp']);g=json.loads((grasp/'result.json').read_text())
        report.update(resumed_with_current_verified_hold=True,grasp=str(grasp),close_commands_sent_this_run=0);save()
        if entry.get('carry'):
            carry=Path(entry['carry']);report['carry']=str(carry);save()
        else:
            carry,_=run_stage('carry','launch_flat_box_stage_20261101.py',['--grasp',str(grasp),'--destination-id',cfg['destination_id']],lambda r:r.get('carry_destination_verified') is True)
    else:
        if not args.resume_at_source:
            source,_=run_stage('source','launch_cigarette_source_20261101.py',[],lambda r:r.get('status')=='source_navigation_and_three_rgbd_observations_complete')
        else:report['resumed_at_current_source_pose']=True;save()
        grasp,g=run_stage('grasp','launch_flat_box_grasp_20261101.py',['--source-table-id',cfg['source_table_id'],'--gripper-geometry',str(b/'hsr-gripper-colliders-20261101-171937.json')]+(['--tracked-source',str(args.tracked_source)] if args.tracked_source else []),lambda r:r.get('grasp_success_verified') is True)
        report['close_commands_sent_this_run']=g['close_commands_sent'];save()
        carry,_=run_stage('carry','launch_flat_box_stage_20261101.py',['--grasp',str(grasp),'--destination-id',cfg['destination_id']],lambda r:r.get('carry_destination_verified') is True)
    place,z=run_stage('place','launch_flat_box_stage_20261101.py',['--held-run',str(carry),'--gripper-geometry',str(b/'hsr-gripper-colliders-20261101-171937.json')],lambda r:r.get('placement_success_verified') is True)
    report.update(status='completed',physical_transfer_verified=True,robot_ready_for_next_task=z.get('robot_ready_for_next_task'),close_commands=g['close_commands_sent'],release_commands=z['release_commands_sent'])
except (KeyboardInterrupt,Exception) as exc:
    if active and running(active):os.killpg(active,signal.SIGINT)
    report.update(status='stopped_requires_state_review',reason=str(exc) or 'interrupted')
finally:save();print(json.dumps(report,ensure_ascii=False))
