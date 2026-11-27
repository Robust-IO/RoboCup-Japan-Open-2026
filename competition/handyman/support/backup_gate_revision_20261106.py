"""Back up authoritative runtime sources and deployment metadata, without ROS."""
import hashlib,json,tarfile,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
active={'run_cigarette_transfer_20261101.py','flat_box_grasp.py','flat_box_carry.py','flat_box_place.py','outcome_finite_adjustment.py','navigate_carry_trial.py'}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:args=[v.decode(errors='replace') for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    except (FileNotFoundError,PermissionError):continue
    if len(args)>1 and Path(args[0]).name.startswith('python') and any(Path(v).name in active for v in args[1:3]):
        state=(proc/'stat').read_text().split()[2]
        if state!='Z':raise RuntimeError('Active robot task; no deployment: '+str(proc.name))
stamp=time.strftime('%Y%m%d-%H%M%S');root=b/'backups'/('before-gate-revision-'+stamp);root.mkdir()
entries=[]
for base,label in ((b,'runtime'),(f,'frozen')):
    paths=list(base.glob('*.py'))+list(base.glob('*.json')) if label=='runtime' else [p for p in base.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix in ('.py','.json','.yaml','.yml','.sh','.txt','.md')]
    for path in paths:
        relative=Path(label)/path.relative_to(base);target=root/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(path.read_bytes())
        entries.append(dict(path=str(path),relative=str(relative),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
manifest=dict(status='backup_complete',created_ns=time.time_ns(),root=str(root),robot_commands_sent=False,files=entries)
(root/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
archive=root.with_suffix('.tar.gz')
with tarfile.open(archive,'w:gz') as tar:tar.add(root,arcname='baseline')
receipt=dict(root=str(root),archive=str(archive),files=len(entries),sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),robot_commands_sent=False)
(b/'gate-revision-backup-latest.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
