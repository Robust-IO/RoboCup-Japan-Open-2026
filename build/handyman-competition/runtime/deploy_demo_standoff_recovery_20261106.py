"""Back up and narrowly patch tested D runtime; do not install offline v1."""
import ast,hashlib,json,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');target=b/'flat_box_place.py'
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:parts=[v.decode() for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    except (FileNotFoundError,PermissionError,UnicodeError):continue
    if len(parts)>1 and any(Path(v).name in ('flat_box_place.py','flat_box_grasp.py','flat_box_carry.py','run_cigarette_transfer_20261101.py') for v in parts[1:3]):raise RuntimeError('active_robot_task_no_patch')
source=target.read_text();updated=source
replacements=[
('                if not already_extended and not clearance[\'ready\']:\n',
 '                standoff_corrections=0\n                while not already_extended and not clearance[\'ready\']:\n                    if standoff_corrections>=policy[\'maximum_corrections\']:\n                        raise ValueError(\'preextension_standoff_corrections_exhausted\')\n                    standoff_corrections+=1\n'),
("                    backend.adjust('preextension-standoff',row['state']['poses']['map'],[-retreat,0.,0.],row['state']['joints'][:5],row['state']['joints'][5])\n                    row,sample=observe('table-after-standoff')\n",
 "                    label='preextension-standoff' if standoff_corrections==1 else f'preextension-standoff-{standoff_corrections}'\n                    backend.adjust(label,row['state']['poses']['map'],[-retreat,0.,0.],row['state']['joints'][:5],row['state']['joints'][5])\n                    row,sample=observe(f'table-after-standoff-{standoff_corrections}')\n"),
("                    if not clearance['ready']:raise ValueError('extension_standoff_not_observed')\n",
 "                    record(dict(event='preextension_standoff_remeasured',correction=standoff_corrections,clearance=clearance))\n                report['preextension_standoff_corrections']=standoff_corrections\n                write_json(root/'extension-clearance-final.json',clearance)\n")]
for old,new in replacements:
    if updated.count(old)!=1:raise RuntimeError('tested_runtime_block_changed')
    updated=updated.replace(old,new)
ast.parse(updated)
root=b/'backups'/('demo-standoff-recovery-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir()
(root/target.name).write_text(source)
temp=target.with_suffix('.demo-new');temp.write_text(updated);temp.replace(target)
report=dict(status='narrow_standoff_recovery_deployed',target=str(target),backup=str(root/target.name),before_sha256=hashlib.sha256(source.encode()).hexdigest(),after_sha256=hashlib.sha256(updated.encode()).hexdigest(),maximum_corrections=5,clearance_threshold_changed=False,offline_package_activated=False,robot_commands_sent=False)
(b/'demo-standoff-recovery-deployment.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
