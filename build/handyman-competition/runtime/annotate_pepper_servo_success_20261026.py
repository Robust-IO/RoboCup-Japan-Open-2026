import hashlib,json,shutil
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project');roots=(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1')
name='phase5-pepper-visual-servo-success-progress-20261026.json';d=json.loads((base/name).read_text())
if not d['physical_transfer_verified'] or not d['robot_ready_for_next_task']:raise ValueError('actual_success_required')
for tree in roots:
 for n,h in d['latest_expected_hashes'].items():
  if hashlib.sha256((tree/n).read_bytes()).hexdigest()!=h:raise ValueError('source_changed:'+str(tree/n))
source=base/'object-transfer-pepper-servo-success-20261026.txt';content=source.read_bytes()
if not content.startswith((roots[0]/'OBJECT_TRANSFER.txt').read_bytes()):raise ValueError('prior_notes_must_be_retained')
for tree in roots:
 pending=tree/'OBJECT_TRANSFER.txt.success-pending';pending.write_bytes(content);pending.replace(tree/'OBJECT_TRANSFER.txt')
d['executed_code_expected_hashes']=dict(d['latest_expected_hashes'])
d['latest_expected_hashes']['OBJECT_TRANSFER.txt']=hashlib.sha256(content).hexdigest();d['success_note_metadata_only']=True
for directory in (base,repo/'training'):
 for n in (name,'phase5-current-work-progress-20261026.json'):
  p=directory/n;temp=p.with_suffix('.pending');temp.write_text(json.dumps(d,indent=2));temp.replace(p)
print(json.dumps(dict(status=d['status'],metadata_only=True,actual_transfer_verified=True,tests=d['tests_passed'])))
