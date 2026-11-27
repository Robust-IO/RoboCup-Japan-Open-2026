import hashlib,json
from pathlib import Path
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
candidate=base/'can-precontact-axis-v1-20261023-validation/candidate'
roots=[Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
names=['held_can_appearance.py','run_canned_grasp_trial.py','canned_grasp_runtime.py','test_tracked_can_cap_geometry.py','test_canned_grasp_geometry.py','OBJECT_TRANSFER.txt']
for root in roots:
    print('TARGET',str(root),json.dumps({n:dict(current=sha(root/n),candidate=sha(candidate/n)) for n in names}))
    diffs=[]
    for p in candidate.rglob('*'):
        rel=p.relative_to(candidate)
        if not p.is_file() or '__pycache__' in p.parts or p.name.endswith('.pyc') or str(rel) in names:continue
        other=root/rel
        if other.is_file() and sha(other)!=sha(p):diffs.append(str(rel))
    print('OTHER_DIFFERENCES',diffs[:100])
print('BACKUP_EXISTS',(base/'can-precontact-axis-v1-20261023-validation/previous-files.tar.gz').exists())
