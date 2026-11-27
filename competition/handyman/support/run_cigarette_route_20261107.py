"""Reuse verified pickup/grasp/carry/place stages with a new explicit destination.

No global task/config edits. Source-ready confirms only the operator's object
placement; the object still requires actual detection and physical lift proof.
"""
from competition_paths import is_run_path
import argparse,fcntl,json,os,signal,subprocess,sys,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--source-ready',action='store_true');p.add_argument('--task',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b),str(f)]
from cigarette_route_plan_20261107 import plan
import yaml
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());task=json.loads(a.task.read_text())
spec=plan(task,json.loads(Path(cfg['semantic_map']).read_text()),yaml.safe_load(Path(cfg['environment']).read_text()),cfg)
if not a.run:print(json.dumps(spec,ensure_ascii=False));raise SystemExit(0)
if not a.source_ready:raise ValueError('source_placement_confirmation_required')
if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('isolated_domain71_required')
if not is_run_path(a.output.resolve()) or a.output.exists():raise ValueError('new_D_output_required')
lock=Path('/tmp/handyman-cigarette-delivery-sequence.lock').open('a+');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
root=a.output;root.mkdir();started=time.time_ns();active=None
report=dict(status='running',task=task,plan=spec,stages=[],close_commands=0,release_commands=0,task_finished_sent=False,automatic_stage_replay=False)
def save():
    report['elapsed_s']=(time.time_ns()-started)/1e9;tmp=root/'result.tmp';tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2));tmp.replace(root/'result.json')
def running(pid):
    try:return Path('/proc',str(pid),'stat').read_text().split()[2]!='Z'
    except FileNotFoundError:return False
def stage(kind,launcher,extra,success):
    global active
    attempt=1
    while any((b/(f'cigarette-{kind}-20261101-{attempt:02d}'+suffix)).exists() for suffix in ('','-console.log','-launch.json')):attempt+=1
    name=f'cigarette-{kind}-20261101-{attempt:02d}';out=b/name
    report.update(active_stage=kind,**{kind:str(out)});save()
    cmd=['/usr/bin/python3',str(b/launcher)]+([] if kind in ('source','grasp') else [kind])+['--attempt',str(attempt)]+extra
    launched=subprocess.run(cmd,check=True,capture_output=True,text=True);(root/(kind+'-launcher.log')).write_text(launched.stdout+launched.stderr)
    active=json.loads((b/(name+'-launch.json')).read_text())['pid']
    while running(active):time.sleep(.25)
    active=None;result=json.loads((out/'result.json').read_text())
    report['stages'].append(dict(stage=kind,run=str(out),status=result.get('status'),reason=result.get('reason'),elapsed_s=result.get('elapsed_s')));save()
    if not success(result):raise RuntimeError(kind+':'+str(result.get('reason',result.get('status'))))
    return out,result
def stop(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop);save()
try:
    from apple_transfer_automation import RobotLock,check_inputs
    with RobotLock():
        check_inputs(cfg)
        import rclpy,uuid
        from flat_box_place import TaskPlaceRuntime
        from table_localization_client import localize
        from current_task_entry import inspect_entry
        from canned_grasp_runtime import TOPICS
        intake=root/'intake';intake.mkdir();rclpy.init();runtime=TaskPlaceRuntime(intake,'route-intake-'+uuid.uuid4().hex,lambda _:None)
        try:
            runtime.sample(processing_reserve_ns=0);runtime.ownership();report['localization']=localize(runtime,intake,'route-start')
            entry=inspect_entry(runtime,b)
            if entry['mode']!='new_pick':raise ValueError('new_route_starts_with_new_pick_not_old_held_context')
            report['initial_state']=runtime.state();report['initial_capture']=str(runtime.save(runtime.sample(),'initial'))
            if any(runtime.node.count_publishers(t) or runtime.node.count_subscribers(t)!=1 for t in TOPICS):raise ValueError('control_channels_not_exclusive')
        finally:runtime.shutdown();rclpy.shutdown()
    source,_=stage('source','launch_cigarette_source_20261101.py',[],lambda r:r.get('status')=='source_navigation_and_three_rgbd_observations_complete')
    grasp,g=stage('grasp','launch_flat_box_grasp_20261101.py',['--source-table-id',task['source_table_id'],'--gripper-geometry',str(b/'hsr-gripper-colliders-20261101-171937.json')],lambda r:r.get('grasp_success_verified') is True)
    report['close_commands']=g['close_commands_sent'];save()
    carry,c=stage('carry','launch_flat_box_stage_20261101.py',['--grasp',str(grasp),'--destination-id',task['destination_id']],lambda r:r.get('carry_destination_verified') is True)
    # Explicitly pass the destination again; the old baseline default remains
    # square_low_table for its original regression task.
    place,z=stage('place','launch_flat_box_stage_20261101.py',['--held-run',str(carry),'--destination-id',task['destination_id'],'--gripper-geometry',str(b/'hsr-gripper-colliders-20261101-171937.json')],lambda r:r.get('placement_success_verified') is True)
    if z['placement_verification']['destination_id']!=task['destination_id']:raise ValueError('actual_destination_verification_mismatch')
    report.update(status='completed',physical_transfer_verified=True,robot_ready_for_next_task=z.get('robot_ready_for_next_task'),release_commands=z['release_commands_sent'],final_state=z['final_state'])
except BaseException as exc:
    if active and running(active):os.killpg(active,signal.SIGINT)
    report.update(status='stopped_requires_state_review',reason=str(exc) or type(exc).__name__)
finally:save();print(json.dumps(report,ensure_ascii=False))
