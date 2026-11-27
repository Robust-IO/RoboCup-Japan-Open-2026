"""Fresh localization and idle Nav2 only. No movement or task sender."""
import json
import os
from pathlib import Path
import subprocess
import sys

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=base/'canned-initial-20261023-01'
root.mkdir(exist_ok=False)
reports=[]
for name in ('start_canned_idle_navigation_20261023.py','initialize_canned_idle_controller_20261023.py',
             'capture_canned_initial_laser_20261023.py','initialize_canned_scene_20261023.py'):
    with (root/(name+'.log')).open('x') as stream:
        child=subprocess.run([sys.executable,str(base/name)],stdout=stream,stderr=subprocess.STDOUT,
            env=dict(os.environ,ROS_LOG_DIR=str(root/'ros-logs')),timeout=70)
    reports.append(dict(script=name,exit_code=child.returncode))
    print(json.dumps(reports[-1]),flush=True)
    if child.returncode:
        print((root/(name+'.log')).read_text(errors='replace')[-5000:],flush=True)
        raise SystemExit(child.returncode)
print(json.dumps(dict(status='new_scene_context_restored',robot_commands_sent=False,reports=reports)),flush=True)
