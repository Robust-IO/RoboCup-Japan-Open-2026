"""One authorized repeat, with physical restoration excluded from measured run."""
import hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
import yaml
BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
REPO=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project');ROOT=BASE/'penguin-resident-repeat-20261026-01'
sys.path.insert(0,str(CODE));sys.path.insert(0,str(BASE))
from resident_inference import connect
from apple_transfer_automation import RobotLock
from transfer_stage_timing import summarize


def main():
    ROOT.mkdir(exist_ok=False);active=None;started=time.time_ns()
    status=dict(status='running',stage='verify_current_code',started_ns=started,
        restore_elapsed_s=None,official_timer_started=False,prior_run=str(BASE/'penguin-resident-fast-transfer-20261026-01'))
    def save():
        p=ROOT/'status.pending';p.write_text(json.dumps(status,indent=2));p.replace(ROOT/'status.json')
    def stage(name):status['stage']=name;status['stage_started_ns']=time.time_ns();save()
    def command(args,name):
        nonlocal active
        with (ROOT/(name+'.console.log')).open('xb') as out:
            active=subprocess.Popen(list(map(str,args)),stdout=out,stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,start_new_session=True)
            code=active.wait();active=None
        if code:raise RuntimeError(name+'_failed_review_current_state')
    def transfer(name,task=None):
        output=ROOT/name
        args=[sys.executable,CODE/'run_object_transfer.py','--object','toy_penguin','--run',
              '--confirm-layout','LayoutA','--confirm-localization','--simulation-ignore-collision',
              '--navigation-profile','carry_fast_v1','--output',output]
        if task:args+=['--task-config',task]
        command(args,name)
        result=json.loads((output/'result.json').read_text())
        if not result.get('physical_transfer_verified') or not result.get('robot_ready_for_next_task'):
            raise RuntimeError(name+'_not_verified_review_actual_state')
        status[name]=dict(result=str(output/'result.json'),elapsed_s=result['elapsed_s'],
            close_commands_sent=result['close_commands_sent'],release_commands_sent=result['release_commands_sent'])
        save();return result
    def interrupt(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:
        checkpoint=json.loads((BASE/'resident-model-fast-navigation-trial-progress-20261026.json').read_text())
        for tree in (CODE,REPO/'training',REPO/'training/automation/apple_transfer_v1'):
            for name,expected in checkpoint['latest_expected_hashes'].items():
                if hashlib.sha256((tree/name).read_bytes()).hexdigest()!=expected:raise ValueError('source_changed:'+str(tree/name))
        cfg=json.loads((CODE/'config/apple_transfer_auto_v1.json').read_text())
        model=connect(cfg['model_python'],cfg['registry'],imgsz=640)
        status['model_before']=model.ready;model.close();save()
        stage('restore_object_to_bedroom')
        restored=transfer('restore-object',CODE/'config/penguin_living_room_to_bedroom.json')
        stage('restore_robot_initial_position')
        first=json.loads((BASE/'penguin-resident-fast-transfer-20261026-01/result.json').read_text())
        initial=first['initial_base_map_pose'];status['requested_initial_pose']=initial;save()
        environment=yaml.safe_load(Path(cfg['environment']).read_text())
        route=next(r['waypoints'] for r in environment['routes'] if r['from']=='bedroom' and r['to']=='living_room')
        nav=[sys.executable,CODE/'navigate_carry_trial.py','--run','--goal',*initial,
             '--holding-opening',str(restored['last_fresh_robot_state']['joints'][5]),'--verify-current-precision',
             '--motion-profile','carry_fast_v1','--joint-feedback-max-age-s','.85','--recover-feedback-interruption',
             '--output',ROOT/'restore-initial-navigation.json']
        for w in route:nav+=['--waypoint',w['x'],w['y'],w['yaw']]
        with RobotLock():
            command(nav,'restore-initial-navigation')
            navigation=json.loads((ROOT/'restore-initial-navigation.json').read_text())
            if navigation.get('status')!='arrived' or navigation.get('speed_parameters_restored') is not True:
                raise ValueError('initial_pose_not_restored')
            stage('restore_initial_head_view')
            import rclpy
            from pepper_runtime import PepperRuntime
            folder=ROOT/'restore-head';folder.mkdir(exist_ok=False);rclpy.init();runtime=None
            try:
                with (folder/'journal.jsonl').open('x') as journal:
                    def record(e):journal.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');journal.flush()
                    runtime=PepperRuntime(folder,'repeat-restore-head',record)
                    runtime.face_source([0.,0.]);status['restored_robot_state']=runtime.state()
            finally:
                if runtime:runtime.shutdown()
                rclpy.shutdown()
        status['restore_elapsed_s']=(time.time_ns()-started)/1e9
        stage('timed_forward_transfer');status['official_timer_started']=True;save()
        measured=transfer('timed-transfer')
        second=summarize(ROOT/'timed-transfer');previous=summarize(BASE/'penguin-resident-fast-transfer-20261026-01')
        worker=json.loads((ROOT/'timed-transfer/grasp/model-worker.log').read_text())
        comparison=dict(previous=previous,repeat=second,total_delta_s=second['elapsed_s']-previous['elapsed_s'],
            repeat_model=worker,first_model=json.loads((BASE/'penguin-resident-fast-transfer-20261026-01/grasp/model-worker.log').read_text()),
            restoration_excluded=True,restoration_elapsed_s=status['restore_elapsed_s'],
            note='Both runs use the resident model. This is warm-to-warm timing, not a cold-vs-warm A/B. Returned object location and moving guests can differ.')
        (ROOT/'comparison.json').write_text(json.dumps(comparison,indent=2))
        status.update(status='repeat_full_transfer_verified',stage='complete',elapsed_s=measured['elapsed_s'],
            total_delta_s=comparison['total_delta_s'],physical_transfer_verified=True,
            current_object_location='square_low_table#living_room',model_after=worker,
            final_robot_state=measured.get('last_fresh_robot_state'))
    except (Exception,KeyboardInterrupt) as exc:
        status.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
    finally:
        if active and active.poll() is None:
            os.killpg(active.pid,signal.SIGTERM)
            try:active.wait(timeout=15)
            except subprocess.TimeoutExpired:status['child_shutdown_incomplete']=True
        status['completed_ns']=time.time_ns();save();print(json.dumps(status),flush=True)
    if status['status']!='repeat_full_transfer_verified':raise SystemExit(1)


if __name__=='__main__':main()
