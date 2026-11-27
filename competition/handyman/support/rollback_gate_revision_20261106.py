"""Preview by default. Restore only this revision, refusing unrelated changes."""
import argparse,fcntl,hashlib,json,shutil,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');args=p.parse_args()
b=Path('/__HANDYMAN_PACKAGE__/runtime');receipt=b/'gate-revision-deployment-latest.json';deployment=json.loads(receipt.read_text())
lease=Path('/tmp/handyman-apple-transfer-automation.lock').open('a+')
fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
if deployment.get('rollback_completed'):print(json.dumps(dict(status='already_rolled_back')));raise SystemExit(0)
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None
conflicts=[]
for row in deployment['files']:
    if sha(Path(row['target']))!=row['sha256']:conflicts.append(row['target'])
    if sha(Path(row['backup']))!=row['before_sha256']:conflicts.append('backup:'+row['backup'])
if conflicts:raise ValueError('Unrelated changes or invalid backup; no files restored: '+repr(conflicts))
if not args.run:
    print(json.dumps(dict(status='rollback_preview_passed',files=len(deployment['files']),backup=deployment['backup'],robot_commands_sent=False)));raise SystemExit(0)
active={'run_cigarette_transfer_20261101.py','flat_box_grasp.py','flat_box_place.py','flat_box_carry.py','outcome_finite_adjustment.py','navigate_carry_trial.py','start_fresh_cigarette_session_20261101.py','observe_grasp_object_20261101.py'}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:parts=[v.decode(errors='replace') for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    except (FileNotFoundError,PermissionError):continue
    if len(parts)>1 and Path(parts[0]).name.startswith('python') and any(Path(v).name in active for v in parts[1:3]) and (proc/'stat').read_text().split()[2]!='Z':raise ValueError('Stop the active transfer before rollback; no files restored')
for row in reversed(deployment['files']):
    target=Path(row['target']);saved=Path(row['backup'])
    if saved.exists():
        temp=target.with_name(target.name+'.rollback-new');shutil.copy2(saved,temp);temp.replace(target)
    else:target.unlink()
deployment.update(rollback_completed=True,rolled_back_ns=time.time_ns())
receipt.write_text(json.dumps(deployment,indent=2));print(json.dumps(dict(status='rollback_complete',files=len(deployment['files']),robot_commands_sent=False)))
