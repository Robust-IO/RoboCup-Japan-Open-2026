"""Preview or restore this common layer; no robot command is ever sent."""
import argparse,fcntl,hashlib,json,shutil,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args()
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');path=b/'common-placement-deployment-latest.json';receipt=json.loads(path.read_text())
if receipt.get('rollback_completed'):raise ValueError('already_rolled_back')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
for row in receipt['files']:
    if sha(Path(row['target']))!=row['sha256']:raise ValueError('newer_changes_restore_them_first:'+row['target'])
    if row['backup'] is not None and sha(Path(row['backup']))!=row['before_sha256']:raise ValueError('backup_hash_mismatch')
if not a.run:
    print(json.dumps(dict(status='rollback_preview_verified',files=len(receipt['files']),backup=receipt['backup'],robot_commands_sent=False)));raise SystemExit(0)
leases=[]
for name in ('/tmp/handyman-apple-transfer-automation.lock','/tmp/handyman-cigarette-delivery-sequence.lock'):
    h=Path(name).open('a+');fcntl.flock(h,fcntl.LOCK_EX|fcntl.LOCK_NB);leases.append(h)
# Recheck after ownership, never restore stale or subsequently edited files.
for row in receipt['files']:
    if sha(Path(row['target']))!=row['sha256']:raise ValueError('target_changed_before_restore')
for row in reversed(receipt['files']):
    target=Path(row['target'])
    if row['backup'] is None:target.unlink()
    else:
        tmp=target.with_name(target.name+'.common-restore');shutil.copy2(row['backup'],tmp);tmp.replace(target)
for row in receipt['files']:
    if sha(Path(row['target']))!=row['before_sha256']:raise ValueError('restored_hash_mismatch')
receipt.update(rollback_completed=True,rollback_ns=time.time_ns());path.write_text(json.dumps(receipt,indent=2))
print(json.dumps(dict(status='common_placement_rolled_back',files=len(receipt['files']),robot_commands_sent=False)))
