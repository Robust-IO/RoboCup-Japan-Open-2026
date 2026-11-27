"""Snapshot runtime + navigation configs before the efficiency change; no ROS."""
import fcntl,hashlib,json,shutil,subprocess,tarfile,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');repo=Path('/__HANDYMAN_PACKAGE__/project')
lock=Path('/tmp/handyman-apple-transfer-automation.lock').open('a+')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
active={'run_cigarette_transfer_20261101.py','flat_box_grasp.py','flat_box_carry.py','flat_box_place.py','outcome_finite_adjustment.py','navigate_carry_trial.py','observe_grasp_object_20261101.py','start_fresh_cigarette_session_20261101.py'}
processes=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        args=[v.decode(errors='replace') for v in (p/'cmdline').read_bytes().split(b'\0') if v]
        state=(p/'stat').read_text().split()[2]
    except OSError:continue
    if args:processes.append(dict(pid=p.name,args=args,state=state))
    if state!='Z' and args and Path(args[0]).name.startswith('python') and any(Path(v).name in active for v in args[1:3]):raise RuntimeError('active_robot_task:'+p.name)
root=b/'backups'/('before-navigation-efficiency-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir()
entries=[]
def save(path,label,relative):
    if not path.is_file():return
    target=root/label/relative;target.parent.mkdir(parents=True,exist_ok=True);data=path.read_bytes();target.write_bytes(data)
    entries.append(dict(path=str(path),relative=str(target.relative_to(root)),sha256=hashlib.sha256(data).hexdigest()))
for path in list(b.glob('*.py'))+list(b.glob('*.json')):save(path,'runtime',path.name)
for path in f.rglob('*'):
    if path.is_file() and '__pycache__' not in path.parts and path.suffix in ('.py','.json','.yaml','.yml','.sh','.txt','.md','.xml'):save(path,'frozen',path.relative_to(f))
for path in [repo/'src/handyman_ros2/param/nav2_params.yaml',repo/'install/handyman_ros2/share/handyman_ros2/param/nav2_params.yaml',repo/'src/handyman_ros2/behavior_trees/navigate_w_recovery.xml',repo/'install/handyman_ros2/share/handyman_ros2/behavior_trees/navigate_w_recovery.xml',repo/'src/handyman_rebuild_ros2/config/environments/layout_a.yaml']:
    save(path,'repository',path.relative_to(repo))
status=subprocess.run(['git','status','--short'],cwd=repo,capture_output=True,text=True)
manifest=dict(status='backup_complete',root=str(root),files=entries,processes=processes,git_status=status.stdout,robot_commands_sent=False,created_ns=time.time_ns())
(root/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2));archive=root.with_suffix('.tar.gz')
with tarfile.open(archive,'w:gz') as t:t.add(root,arcname='baseline')
receipt=dict(root=str(root),archive=str(archive),files=len(entries),sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),robot_commands_sent=False)
(b/'navigation-efficiency-backup-latest.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
