"""Preview by default; restore exact pre-install files with --run while idle."""
import argparse,fcntl,hashlib,json,shutil,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args()
b=Path('/__HANDYMAN_PACKAGE__/runtime');receipt_path=b/'navigation-efficiency-deployment-latest.json';d=json.loads(receipt_path.read_text())
if d.get('rollback_completed'):raise ValueError('already_rolled_back')
lease=Path('/tmp/handyman-apple-transfer-automation.lock').open('a+');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
sequence=Path('/tmp/handyman-cigarette-delivery-sequence.lock').open('a+');fcntl.flock(sequence,fcntl.LOCK_EX|fcntl.LOCK_NB)
active={'run_cigarette_transfer_20261101.py','flat_box_grasp.py','flat_box_carry.py','flat_box_place.py','navigate_carry_trial.py','efficient_navigation.py','observe_grasp_object_20261101.py'}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        args=[v.decode(errors='replace') for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
        state=(proc/'stat').read_text().split()[2]
    except OSError:continue
    if state!='Z' and args and Path(args[0]).name.startswith('python') and any(Path(v).name in active for v in args[1:3]):raise ValueError('active_robot_task_no_rollback')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
for row in d['files']:
    target=Path(row['target'])
    if sha(target)!=row['sha256']:raise ValueError('file_changed_after_install_refusing_to_overwrite:'+str(target))
    if row['before_sha256'] is not None and sha(Path(row['backup']))!=row['before_sha256']:raise ValueError('backup_missing_or_changed')
report=dict(status='rollback_preview_ready',files=len(d['files']),backup=d['backup'],robot_commands_sent=False,run=a.run)
if a.run:
    # No goals, trajectories, or lifecycle changes. Existing loaded parameters
    # are restored by the navigation runner on exit; next run uses old files.
    for row in reversed(d['files']):
        target=Path(row['target'])
        if row['before_sha256'] is None:target.unlink()
        else:
            temp=target.with_name(target.name+'.nav-rollback');shutil.copy2(row['backup'],temp);temp.replace(target)
    for row in d['files']:
        if sha(Path(row['target']))!=row['before_sha256']:raise RuntimeError('rollback_readback_failed')
    d.update(rollback_completed=True,rolled_back_ns=time.time_ns());receipt_path.write_text(json.dumps(d,indent=2))
    report['status']='rolled_back'
    (b/'navigation-efficiency-rollback-result.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
