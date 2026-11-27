"""Start a fresh user-confirmed Session1 round using existing live services.

Run in WSL. Bridges/map/Nav2 must already be up; this does not start old task nodes.
It reanchors only the existing initial localizer, prepares the empty hand, then
launches the complete protocol-driven task. Logs are always new directories on D.
"""
import argparse,json,os,signal,subprocess,time,datetime
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--confirmed-session1-initial',action='store_true',required=True);p.parse_args()
    processes=[]
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():continue
        try:args=[v.decode(errors='replace') for v in (item/'cmdline').read_bytes().split(b'\0') if v]
        except (OSError,PermissionError):continue
        processes.append((int(item.name),args))
    for pid,args in processes:
        if pid!=os.getpid() and any(Path(a).name in ('run_live_session1.py','check_source_view.py','prepare_empty_hand.py') for a in args):
            raise RuntimeError('Task process already exists: '+str(pid))
    anchors=[]
    for pid,args in processes:
        if args and Path(args[0]).name=='handyman_initial_pose_localizer':anchors.append((pid,os.getpgid(pid)))
    for pid,pgid in anchors:
        if pgid in (os.getpgrp(),1):raise RuntimeError('Unexpected localizer process group')
        os.killpg(pgid,signal.SIGINT)
    deadline=time.monotonic()+5
    while any(Path('/proc',str(pid)).exists() for pid,_ in anchors):
        if time.monotonic()>deadline:raise RuntimeError('Old initial anchor did not exit')
        time.sleep(.1)
    runs=Path('/__HANDYMAN_PACKAGE__/runtime')
    transport=Path(json.loads((runs/'demo-background-latest.json').read_text())['logs'])
    old=transport/'initial-anchor.log'
    if old.exists():old.rename(old.with_name('initial-anchor-previous-'+str(time.time_ns())+'.log'))
    subprocess.run(['python3',str(runs/'start_demo_initial_anchor_20261106.py')],check=True)
    root=Path(__file__).resolve().parent
    out=runs/('session1-round-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));out.mkdir()
    project=Path('/__HANDYMAN_PACKAGE__/project')
    env='''source /opt/ros/humble/setup.bash
source /__HANDYMAN_PACKAGE__/project/install/setup.bash
export PYTHONPATH=/__HANDYMAN_PACKAGE__/workspace/training/session1:/__HANDYMAN_PACKAGE__/runtime/generic-grasp-v1:/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:${PYTHONPATH:-}
exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- "$@"
'''
    with (out/'prepare.log').open('x') as log:
        subprocess.run(['bash','-c',env,'session1-prepare','python3',str(root/'prepare_empty_hand.py'),'--config',str(root/'session1-live-config.json'),'--output',str(out/'prepare'),'--user-confirmed-initial-empty'],stdout=log,stderr=subprocess.STDOUT,timeout=35,check=True)
    with (out/'live.log').open('x') as log:
        task=subprocess.Popen(['bash',str(root/'start_session1.sh'),'--execute','--output',str(out/'task')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    print(json.dumps(dict(task_pid=task.pid,round_directory=str(out),mode='fresh_full_automation_no_checkpoint')))
if __name__=='__main__':main()
