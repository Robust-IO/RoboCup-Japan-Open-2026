"""Start one new can alignment/transfer trial after user restored upright can."""
import hashlib,json,os,subprocess
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
checkpoint=json.loads((base/'canned-alignment-fix-ready-progress-20261024.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for root in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
    if any(sha(root/n)!=digest for n,digest in checkpoint['latest_expected_hashes'].items()):
        raise ValueError('alignment_tested_code_copy_changed:'+str(root))
branch=subprocess.check_output(['git','-C',str(repo),'branch','--show-current'],text=True).strip()
if branch!='feat/handyman-phase5-grasping':raise ValueError('unexpected_branch')
output=base/'can-auto-live-20261024-08'
if output.exists():raise ValueError('run08_exists_do_not_relaunch')
print(json.dumps(dict(status='alignment_tested_sources_verified_start_new_run08',
    user_confirmed_upright_target=True,tested_regressions=checkpoint['tests'],
    output=str(output),historical_robot_state_not_used_to_authorize_motion=True)),flush=True)
os.execv('/bin/bash',['/bin/bash',str(code/'run_object_transfer.sh'),
    '--object','canned_juice','--run','--confirm-layout','LayoutA','--confirm-localization',
    '--simulation-ignore-collision','--output',str(output),'--task-id',output.name])
