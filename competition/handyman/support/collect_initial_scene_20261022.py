"""Capture fresh images/laser/odometry; no map anchor or command publisher."""
import json
import os
from pathlib import Path
import subprocess
import sys

base=Path('/__HANDYMAN_PACKAGE__/runtime');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
root=base/'apple-r16b-live-initial-20261022-01';root.mkdir(exist_ok=False)
commands={
    'cameras':[sys.executable,str(code/'capture_grasp_scene.py'),str(root/'cameras'),
        '--hand-geometry','--tracking-epoch','r16b-initial-20261022-01','--duration','7','--interval','1'],
    'scan':[sys.executable,str(base/'source-table-boundary-audit-20261017-01/capture_table_localization_audit.py'),
        '--output',str(root/'scans.json')],
    'inputs':[sys.executable,str(code/'apple_transfer_automation.py'),'--check-inputs']}
children={}
for name,command in commands.items():
    with (root/(name+'.log')).open('x') as stream:
        children[name]=subprocess.Popen(command,env=dict(os.environ,ROS_LOG_DIR=str(root/'ros-logs')),
            stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
codes={name:child.wait(timeout=35) for name,child in children.items()}
result=dict(status='fresh_initial_evidence_collected' if not any(codes.values()) else 'initial_evidence_incomplete',
    exit_codes=codes,commands=commands,robot_commands_sent=False)
(root/'collection.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
if any(codes.values()):raise SystemExit(1)
