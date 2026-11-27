"""Capture deployment baselines; no ROS, motion or mutation of source files."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

RUNS=Path('/__HANDYMAN_PACKAGE__/runtime')
REPO=Path('/__HANDYMAN_PACKAGE__/project')
ROOTS=dict(runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),
           training=REPO/'training',module=REPO/'training/automation/apple_transfer_v1')
FILES=['apple_transfer_auto_backend.py','apple_transfer_auto_plan.py','config/apple_transfer_auto_v1.json',
       'capture_grasp_scene.py','apple_worker_client.py','apple_inference_worker.py','inspect_table_apple.py',
       'pregrasp_raised_trial.py','inspect_transfer_ready.py','apple_approach_lease.py',
       'run_bedroom_apple_regrasp.py','run_apple_close_lift_trial.py','reviewed_carry_adjustment.py',
       'transfer_efficiency.py','apple_shared_inference.py','test_transfer_efficiency.py',
       'APPLE_TRANSFER_AUTOMATION.txt']
FROZEN=['navigate_carry_trial.py','motion_speed.py','config/motion_speed_profiles.json',
        'apple_transfer_automation.py','inverse_table_place_plan.py','task_outcome_policy.py',
        'table_release_gate.py','verify_table_placement.py']
OUT=RUNS/'apple-efficiency-r17-20261022-baseline.json'
BACKUP=RUNS/'apple-before-efficiency-r17-20261022.tar.gz'
if OUT.exists() or BACKUP.exists():raise RuntimeError('baseline_already_exists')
for root in [Path('/home'),REPO.parent,REPO,REPO/'training',REPO/'training/automation',ROOTS['module'],ROOTS['runtime']]:
    if (root/'AGENTS.md').exists():
        print('AGENTS '+str(root/'AGENTS.md')+'\n'+(root/'AGENTS.md').read_text())
        raise RuntimeError('read_existing_agent_instructions_before_deployment')
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
report=dict(files=FILES,roots={k:str(v) for k,v in ROOTS.items()},
            hashes={k:{f:digest(root/f) for f in FILES} for k,root in ROOTS.items()},
            frozen={k:{f:digest(root/f) for f in FROZEN} for k,root in ROOTS.items()},
            environment_sha256=digest(REPO/'src/handyman_rebuild_ros2/config/environments/layout_a.yaml'),
            branch=subprocess.check_output(['git','-C',str(REPO),'branch','--show-current'],text=True).strip(),
            robot_commands_sent=False)
with tarfile.open(BACKUP,'x:gz') as archive:
    for key,root in ROOTS.items():
        for name in FILES:
            if (root/name).is_file():archive.add(root/name,arcname=key+'/'+name)
with OUT.open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps(report,indent=2))
