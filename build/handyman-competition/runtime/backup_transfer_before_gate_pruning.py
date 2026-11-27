"""Verified D-drive snapshot before changing active transfer code. No robot commands."""
import hashlib,json,subprocess,time,zipfile
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');frozen=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
root=base/'backups'/('before-outcome-policy-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir(parents=True,exist_ok=False)
files={}
for path in base.glob('*.py'):files['active/'+path.name]=path
for path in frozen.rglob('*'):
    if path.is_file() and path.suffix in ('.py','.json','.yaml','.yml','.xml','.sh','.md','.txt'):
        files['frozen/'+str(path.relative_to(frozen))]=path
for name in ('active_detection_model.json','hsr-gripper-colliders-20261101-171937.json',
             'cigarette-bedroom-white-table-to-living-square-low-table-20261101-completed.json'):
    files['evidence/'+name]=base/name
for name in ('cigarette-grasp-20261101-08','cigarette-contact-20261101-02','cigarette-carry-20261101-03','cigarette-place-20261101-05'):
    for path in (base/name).glob('*.json'):files['evidence/'+name+'/'+path.name]=path
manifest={};archive=root/'transfer-before-changes.zip'
with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as z:
    for name,path in sorted(files.items()):
        data=path.read_bytes();z.writestr(name,data);manifest[name]=dict(source=str(path),bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
with zipfile.ZipFile(archive) as z:
    if z.testzip() is not None:raise ValueError('backup_crc_failure')
    for name,item in manifest.items():
        if hashlib.sha256(z.read(name)).hexdigest()!=item['sha256']:raise ValueError('backup_hash_failure:'+name)
repo='/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project'
report=dict(status='verified_backup',robot_commands_sent=False,created_ns=time.time_ns(),archive=str(archive),files=manifest,
    branch=subprocess.run(['git','-C',repo,'branch','--show-current'],capture_output=True,text=True,check=True).stdout.strip(),
    working_tree=subprocess.run(['git','-C',repo,'status','--short'],capture_output=True,text=True,check=True).stdout)
(root/'manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
(base/'outcome-policy-backup-latest.json').write_text(json.dumps(dict(status='verified_backup',root=str(root),archive=str(archive),files=len(manifest)),indent=2))
print(json.dumps(dict(status='verified_backup',root=str(root),archive=str(archive),files=len(manifest),archive_bytes=archive.stat().st_size)))
