"""Continue this request from grasp09 through carry04 and place06 exactly once."""
import json,os,signal,subprocess,time,fcntl,argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--grasp-attempt',type=int,default=9);p.add_argument('--chain-attempt',type=int,default=1);p.add_argument('--grasp-run');args=p.parse_args()
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');name='cigarette-delivery-outcome-20261101-'+format(args.chain_attempt,'02d')
report_path=b/(name+'.json');lock=Path('/tmp/'+name+'.lock').open('a+');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
if report_path.exists():raise ValueError('existing_delivery_chain_must_be_reviewed')
report=dict(status='running',request='卧室白桌香烟→客厅矮方桌',source_run='cigarette-source-20261101-03',
    grasp=args.grasp_run or 'cigarette-grasp-20261101-'+format(args.grasp_attempt,'02d'),carry='cigarette-carry-20261101-04',place='cigarette-place-20261101-06',stages=[])
def save():
    tmp=report_path.with_suffix('.tmp');tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2));tmp.replace(report_path)
def running(pid):
    try:return Path('/proc',str(pid),'stat').read_text().split()[2]!='Z'
    except FileNotFoundError:return False
def wait(root,success):
    report['active_stage']=root;save();pid=json.loads((b/(root+'-launch.json')).read_text())['pid']
    while running(pid):time.sleep(.5)
    r=json.loads((b/root/'result.json').read_text())
    report['stages'].append(dict(run=root,status=r.get('status'),elapsed_s=r.get('elapsed_s'),reason=r.get('reason')));save()
    if not r.get(success):raise RuntimeError(root+':'+str(r.get('reason',r.get('status'))))
    return r
def stop(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,stop);save()
try:
    grasp=wait(report['grasp'],'grasp_success_verified')
    subprocess.run(['/usr/bin/python3',str(b/'launch_flat_box_stage_20261101.py'),'carry','--attempt','4','--grasp',str(b/report['grasp'])],check=True)
    carry=wait(report['carry'],'carry_destination_verified')
    subprocess.run(['/usr/bin/python3',str(b/'launch_flat_box_stage_20261101.py'),'place','--attempt','6','--held-run',str(b/report['carry']),
        '--gripper-geometry',str(b/'hsr-gripper-colliders-20261101-171937.json')],check=True)
    place=wait(report['place'],'placement_success_verified')
    report.update(status='completed',physical_transfer_verified=True,robot_ready_for_next_task=place.get('robot_ready_for_next_task'),
        close_commands=grasp['close_commands_sent'],release_commands=place['release_commands_sent'],task_finished_sent=False)
except KeyboardInterrupt:
    active=report.get('active_stage');receipt=b/(str(active)+'-launch.json')
    if receipt.exists():
        pid=json.loads(receipt.read_text())['pid']
        if running(pid):os.killpg(pid,signal.SIGINT)
    report.update(status='interrupted',automatic_stage_replay=False)
except Exception as exc:report.update(status='stopped_requires_state_review',reason=str(exc),automatic_stage_replay=False)
finally:save();print(json.dumps(report,ensure_ascii=False))
