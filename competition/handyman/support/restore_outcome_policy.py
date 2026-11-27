"""Restore original active files; archive new files rather than deleting them."""
import argparse,fcntl,json,time,zipfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');args=p.parse_args()
base=Path('/__HANDYMAN_PACKAGE__/runtime');deployment=json.loads((base/'outcome-policy-deployment.json').read_text())
if not args.run:print(json.dumps(dict(status='restore_plan',backup=deployment['backup'],files=len(deployment['installed']),robot_commands_sent=False)));raise SystemExit(0)
lock=Path('/tmp/handyman-apple-transfer-automation.lock').open('a+');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
saved=base/'backups'/('saved-outcome-version-before-restore-'+time.strftime('%Y%m%d-%H%M%S'));saved.mkdir(exist_ok=False)
original=zipfile.ZipFile(deployment['backup']['archive']);extra_path=Path(deployment['backup']['root'])/'supplemental-before-activation.zip'
extra=zipfile.ZipFile(extra_path) if extra_path.exists() else None
for item in deployment['installed']:
    target=base/item['name']
    if target.exists():target.replace(saved/item['name'])
    if item['previous_archive_key']:target.write_bytes(original.read(item['previous_archive_key']))
    elif item['supplemental']:target.write_bytes(extra.read(item['name']))
original.close()
if extra:extra.close()
report=dict(status='original_active_version_restored',saved_outcome_version=str(saved),robot_commands_sent=False)
(saved/'restore-receipt.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
