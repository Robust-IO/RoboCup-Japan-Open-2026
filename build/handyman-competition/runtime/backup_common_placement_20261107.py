import fcntl,hashlib,json,shutil,tarfile,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
lease=Path('/tmp/handyman-apple-transfer-automation.lock').open('a+');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
root=b/'backups'/('before-common-placement-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir()
shutil.copytree(f,root/'frozen',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
entries=[]
for p in b.iterdir():
    if p.is_file() and p.suffix in ('.py','.json','.txt'):
        target=root/'runtime'/p.name;target.parent.mkdir(exist_ok=True);shutil.copy2(p,target)
for label in ('frozen','runtime'):
    for p in (root/label).rglob('*'):
        if p.is_file():entries.append(dict(relative=str(p.relative_to(root)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
(root/'manifest.json').write_text(json.dumps(dict(files=entries,robot_commands_sent=False),indent=2))
archive=root.with_suffix('.tar.gz')
with tarfile.open(archive,'w:gz') as t:t.add(root,arcname='baseline')
receipt=dict(root=str(root),archive=str(archive),sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),files=len(entries),robot_commands_sent=False)
(b/'common-placement-backup-latest.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
