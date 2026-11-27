"""Wait for fresh handshakes, then run Session1 through Session6 once."""
import argparse,datetime,json,os,signal,subprocess,time,sys
from console_exit import ConsoleExit
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from round_sequence import RoundSequence
ROOT=Path(__file__).resolve().parent
PROJECT=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
RUNS=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runs')

def processes():
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:a=[v.decode(errors='replace') for v in (p/'cmdline').read_bytes().split(b'\0') if v]
        except OSError:continue
        yield int(p.name),a

def process_running(pid):
    try:
        state=(Path('/proc')/str(pid)/'stat').read_text().split(') ',1)[1].split()[0]
        return state not in ('Z','X')
    except (OSError,IndexError):return False

def retire_services():
    groups=set()
    for pid,args in processes():
        anchor=args and Path(args[0]).name=='handyman_initial_pose_localizer'
        services=any(a.endswith('/session_services.launch.py') or a.endswith('/session1_services.launch.py') for a in args)
        if anchor or services:
            group=os.getpgid(pid)
            if group in (1,os.getpgrp()):raise RuntimeError('unexpected_service_process_group')
            groups.add(group)
    for group in groups:
        try:os.killpg(group,signal.SIGINT)
        except ProcessLookupError:pass
    def still_running():
        for pid,_ in processes():
            try:
                if os.getpgid(pid) in groups and process_running(pid):return True
            except ProcessLookupError:pass
        return False
    # Restrict escalation to the service groups discovered above. A terminated
    # but unreaped child cannot publish commands and must not block the next map.
    for sig,seconds in ((None,15),(signal.SIGTERM,5),(signal.SIGKILL,3)):
        if sig is not None:
            for group in groups:
                try:os.killpg(group,sig)
                except ProcessLookupError:pass
        deadline=time.monotonic()+seconds
        while still_running() and time.monotonic()<deadline:time.sleep(.1)
        if not still_running():return
    raise RuntimeError('old_services_not_retired')

def spawn(args,log):
    with log.open('x') as f:
        return subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)

def round_spec(index):
    if index not in (0,1,2,3,4,5):raise ValueError('unsupported_session_index')
    return dict(layout=('LayoutA','LayoutB','LayoutA','LayoutC','LayoutD','LayoutD')[index],
        config=ROOT/('session3-search-config.json' if index==2 else f'session{index+1}-live-config.json'),
        runner=ROOT/('run_session3_search.py' if index==2 else 'run_live_session1.py'))

def task_command(index,config,out,*,fresh=True):
    return ['/usr/bin/python3',str(round_spec(index)['runner']),'--config',str(config),
            '--execute']+(['--prepare-initial-empty'] if fresh else [])+['--output',str(out/'task')]

def setup(index,out):
    import yaml
    retire_services()
    spec=round_spec(index);layout=spec['layout'];config=spec['config']
    cfg=json.loads(config.read_text());env=yaml.safe_load(Path(cfg['environment']).read_text())
    service=spawn(['ros2','launch',str(ROOT/'session_services.launch.py'),'layout:='+layout],out/'services.log')
    clock=None;clock_log=None;started=time.monotonic()
    try:
        pose=env['initial_pose']
        args=['ros2','run','handyman_rebuild_ros2','handyman_initial_pose_localizer','--ros-args',
              '-p','map_frame:=map','-p','odom_frame:=odom','-p','base_frame:=base_footprint']
        for k in ('x','y','yaw'):args+=['-p',f'initial_pose.{k}:={float(pose[k])}']
        anchor=spawn(args,out/'anchor.log')
        # Clock settling and Nav2 initialization are independent read-only readiness work.
        clock_log=(out/'clock-ready.log').open('x')
        clock=subprocess.Popen(['/usr/bin/python3',str(ROOT/'startup_clock_ready.py'),'--output',str(out/'clock-ready.json')],
            stdin=subprocess.DEVNULL,stdout=clock_log,stderr=subprocess.STDOUT)
        # ros2 service call waits for the service itself; no fixed startup sleep.
        if service.poll() is not None or anchor.poll() is not None:raise RuntimeError('round_services_exited')
        # Leave velocity ownership with manipulation until the first navigation.
        result=subprocess.run(['ros2','service','call','/lifecycle_manager_navigation/manage_nodes',
            'nav2_msgs/srv/ManageLifecycleNodes','{command: 3}'],capture_output=True,text=True,timeout=30)
        (out/'nav-reset.log').write_text(result.stdout+result.stderr)
        if result.returncode or 'success=True' not in result.stdout:raise RuntimeError('nav_reset_failed')
        nav_ready=time.monotonic()
        remaining=max(.1,45-(nav_ready-started))
        if clock.wait(timeout=remaining)!=0:raise RuntimeError('startup_clock_check_failed')
        if service.poll() is not None or anchor.poll() is not None:raise RuntimeError('round_services_exited')
        (out/'startup-timing.json').write_text(json.dumps(dict(
            nav_ready_s=nav_ready-started,setup_ready_s=time.monotonic()-started,
            clock_parallel=True,empty_hand_prepared_in_task=True)))
        return config
    except BaseException:
        retire_services();raise
    finally:
        if clock is not None and clock.poll() is None:
            clock.terminate()
            try:clock.wait(timeout=3)
            except subprocess.TimeoutExpired:clock.kill();clock.wait()
        if clock_log is not None:clock_log.close()

def parse_args(argv=None):
    p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true',required=True)
    initial=p.add_mutually_exclusive_group(required=True)
    initial.add_argument('--from-session',type=int,choices=range(1,7),help='Start at this Unity session initial pose and continue through Session6')
    for n in (2,4,5):initial.add_argument(f'--from-session{n}-initial',action='store_true')
    initial.add_argument('--from-session1-initial',action='store_true')
    initial.add_argument('--from-session6-initial',action='store_true',help='Wait for fresh Session6 readiness and run Session6 only')
    initial.add_argument('--from-session3-initial',action='store_true',help='Wait for Session3 readiness, then run Session3 through Session6')
    initial.add_argument('--resume-session2-initial',action='store_true',help='Wait for actual LayoutB readiness at the new-round initial pose')
    initial.add_argument('--resume-session3-search-from',type=Path,help='Same live round, resume stopped search without resetting localization')
    p.add_argument('--dynamic-instructions',action='store_true',help='Parse actual task and resolve source from inventory and live search')
    args=p.parse_args(argv)
    if args.dynamic_instructions and args.resume_session3_search_from:
        p.error('dynamic mode does not support the fixed-mode search checkpoint')
    return args

def initial_index(args):
    if args.from_session is not None:return args.from_session-1
    if args.resume_session3_search_from:return 2
    if args.resume_session2_initial:return 1
    for n in range(1,7):
        if getattr(args,f'from_session{n}_initial',False):return n-1
    raise ValueError('initial_session_required')

def main():
    args=parse_args()
    for pid,a in processes():
        if pid!=os.getpid() and any(Path(v).name in ('run_live_session1.py','run_dynamic_session.py','run_sessions12.py','run_sessions123.py','run_session3_search.py','start_session3_round.py','start_session4_round.py','start_session5_round.py','start_session6_round.py') for v in a):
            raise RuntimeError('another_task_or_sequence_running:'+str(pid))
    from ensure_clock_bridge import ensure
    ensure()
    import rclpy
    from handyman_msgs.msg import HandymanMsg
    rclpy.init();node=rclpy.create_node('handyman_sessions123456_supervisor')
    out=RUNS/('sessions123456-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));out.mkdir()
    (RUNS/'sessions12-latest.txt').write_text(str(out))
    (RUNS/'sessions123-latest.txt').write_text(str(out))
    (RUNS/'sessions1234-latest.txt').write_text(str(out))
    (RUNS/'sessions12345-latest.txt').write_text(str(out))
    (RUNS/'sessions123456-latest.txt').write_text(str(out))
    events=(out/'events.jsonl').open('x');seq=RoundSequence();task=None;future=None;round_dir=None;stopping=False;interrupted=False;retirement=None;startup_failures={};preparation_failures={}
    seq.index=initial_index(args)
    def record(**row):
        row['time']=time.time();events.write(json.dumps(row)+'\n');events.flush();print(json.dumps(row),flush=True)
    def receive(msg):
        record(event='moderator',message=msg.message,detail=msg.detail)
        seq.message(msg.message,msg.detail,time.monotonic())
        if msg.message in ('Task_failed','Mission_complete') and task is not None and task.poll() is None:
            os.killpg(task.pid,signal.SIGINT)
    node.create_subscription(HandymanMsg,'/handyman/message/to_robot',receive,10)
    def interrupt(*_):
        nonlocal interrupted
        interrupted=True
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    console=ConsoleExit(sys.stdin)
    print('退出：输入 q 或 exit 后按回车（也可 Ctrl+C）。将停止当前任务并清理本轮导航服务。',flush=True)
    pool=ThreadPoolExecutor(max_workers=1)
    record(event='waiting_for_session'+str(seq.index+1),expected_layout=seq.layouts[seq.index],robot_commands_sent=False)
    prewarm=None
    try:
        from model_prewarm import start as start_prewarm
        try:
            prewarm=start_prewarm(round_spec(seq.index)['config'],PROJECT.parent,out/'model-prewarm.log')
            record(event='model_prewarm_started',pid=prewarm.pid,robot_commands_sent=False)
        except OSError as exc:
            record(event='model_prewarm_deferred_to_task',reason=str(exc))
        if args.resume_session3_search_from:
            seq.launch(2);seq.round_engaged=True
            round_dir=out/'session3';round_dir.mkdir()
            cfg=round_spec(2)['config']
            task=spawn(task_command(2,cfg,round_dir,fresh=False)+['--resume-room-from',str(args.resume_session3_search_from)],round_dir/'live.log')
            record(event='resumed_session3_search',pid=task.pid,checkpoint=str(args.resume_session3_search_from),localization_reset=False)
        while True:
            rclpy.spin_once(node,timeout_sec=.1)
            if console.poll():
                record(event='console_exit_requested');interrupted=True
            if interrupted:seq.finished=True
            transition=seq.pending is not None and seq.started and seq.pending!=seq.index
            if task is not None:
                if (seq.finished or transition) and not stopping and task.poll() is None:
                    from round_retirement import RoundRetirement
                    retirement=RoundRetirement(task.pid,record);stopping=True
                    record(event='retiring_old_task',pid=task.pid)
                retired=retirement.poll() if retirement is not None else True
                code=task.poll()
                if not retired:code=None
                if code is not None:
                    record(event='round_process_exited',session=seq.index+1,returncode=code)
                    task=None;stopping=False;retirement=None
                    if code!=0 and not seq.round_engaged and not seq.finished and seq.pending in (None,seq.index):
                        count=startup_failures.get(seq.index,0)+1;startup_failures[seq.index]=count
                        if count>=3:
                            record(event='startup_retry_exhausted',session=seq.index+1,attempts=count,logs=str(round_dir))
                            raise RuntimeError('round_initialization_failed_after_3_attempts:'+str(round_dir))
                        archived=round_dir.with_name(round_dir.name+f'-startup-failed-{count}')
                        round_dir.rename(archived)
                        seq.started=False;seq.done=False;seq.pending=None;seq.ready_at=None;seq.environment_at=None
                        record(event='startup_retry_waiting_fresh_handshake',session=seq.index+1,attempt=count,logs=str(archived))
                    else:seq.exited()
            if future is not None and future.done():
                try:cfg=future.result()
                except Exception as exc:
                    record(event='round_preparation_failed',session=seq.index+1,error=str(exc),logs=str(round_dir))
                    count=preparation_failures.get(seq.index,0)+1;preparation_failures[seq.index]=count
                    retryable=isinstance(exc,subprocess.TimeoutExpired) or str(exc) in ('startup_clock_check_failed','nav_reset_failed','round_services_exited')
                    if retryable and count<3 and not seq.finished:
                        archived=round_dir.with_name(round_dir.name+f'-preparation-failed-{count}')
                        round_dir.rename(archived);future=None
                        seq.started=False;seq.done=False;seq.pending=None;seq.ready_at=None;seq.environment_at=None;seq.round_engaged=False
                        record(event='round_preparation_retry_waiting_fresh_handshake',session=seq.index+1,attempt=count,logs=str(archived))
                        continue
                    (out/'supervisor-state.json').write_text(json.dumps(dict(session=seq.index+1,preparing=False,finished=True,error=str(exc),logs=str(round_dir))))
                    raise
                future=None
                if not seq.finished and not seq.done and seq.pending in (None,seq.index):
                    command=task_command(seq.index,cfg,round_dir)
                    if args.dynamic_instructions:command[1]=str(ROOT/'run_dynamic_session.py')
                    task=spawn(command,round_dir/'live.log')
                    record(event='round_task_started',session=seq.index+1,pid=task.pid)
            if seq.finished and task is None and future is None:break
            if seq.pending is not None and task is None and future is None and not seq.finished:
                index=seq.pending;seq.launch(index)
                round_dir=out/f'session{index+1}';round_dir.mkdir()
                record(event='preparing_round',session=index+1,layout=seq.layouts[index])
                future=pool.submit(setup,index,round_dir)
            (out/'supervisor-state.json').write_text(json.dumps(dict(session=seq.index+1,
                waiting_for_handshake=not seq.started or seq.done,preparing=future is not None,
                task_pid=None if task is None else task.pid,finished=seq.finished)))
        record(event='sequence_stopped',session=seq.index+1)
    finally:
        from model_prewarm import stop_waiter
        stop_waiter(prewarm)
        if task is not None and task.poll() is None:
            os.killpg(task.pid,signal.SIGINT)
            task.wait() # never start a replacement while command ownership is outstanding
        pool.shutdown(wait=True)
        try:
            if interrupted:
                retire_services()
                record(event='console_exit_cleanup_complete')
        finally:
            events.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
