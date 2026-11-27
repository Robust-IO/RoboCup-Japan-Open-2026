"""Read-only new-scene initial-pose check before the continuous run06."""
import hashlib,json,math,subprocess,sys
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
prior=json.loads((base/'canned-current-start-continuous-success-progress-20261023-05.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for root in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
    if any(sha(root/n)!=v for n,v in prior['latest_expected_hashes'].items()):raise ValueError('tested_code_changed')
if (base/'can-auto-live-20261023-06').exists():raise ValueError('run06_already_exists_do_not_relaunch')
path=base/'can06-prestart-ready-20261023.json'
proc=subprocess.run([sys.executable,str(code/'inspect_transfer_ready.py'),'--output',str(path),'--efficient-transfer'],timeout=25)
r=json.loads(path.read_text())
if proc.returncode or not r.get('ready'):raise ValueError('new_scene_not_ready:'+str(r.get('reason')))
pose=r['base_map_pose']
if math.dist(pose[:2],[1.184,0.])>.05 or abs(pose[2])>.05:raise ValueError('not_default_initial_pose:'+str(pose))
print(json.dumps(dict(status='new_scene_initial_pose_and_tested_code_verified',actual_start_map_pose=pose,
    robot_commands_sent=False,original_navigation_tolerances=r['navigation_tolerances'],
    previous_staged_checkpoint_not_used_for_motion=True)),flush=True)
