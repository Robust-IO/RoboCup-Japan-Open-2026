"""After terminal automation, receive-only cameras/readiness; no cleanup motion."""
import json
import os
from pathlib import Path
import subprocess
import sys

base=Path('/__HANDYMAN_PACKAGE__/runtime');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock
trial=base/'automatic-apple-r16b-20261022-01';root=base/'apple-r16b-post-full-live-20261022-01'
if not (trial/'result.json').exists() or json.loads((trial/'result.json').read_text())['status']=='running':
    raise ValueError('terminal_trial_required')
with RobotLock():
    root.mkdir(exist_ok=False)
    commands={
        'cameras':[sys.executable,str(code/'capture_grasp_scene.py'),str(root/'cameras'),
            '--hand-geometry','--map-geometry','--tracking-epoch','r16b-post-full-live-20261022-01','--duration','7','--interval','1'],
        'ready':[sys.executable,str(code/'inspect_transfer_ready.py'),'--output',str(root/'ready.json')]}
    children={}
    for name,cmd in commands.items():
        with (root/(name+'.log')).open('x') as stream:
            children[name]=subprocess.Popen(cmd,env=dict(os.environ,ROS_LOG_DIR=str(root/'ros-logs')),
                stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    codes={name:p.wait(timeout=30) for name,p in children.items()}
    captures=sorted((root/'cameras').glob('*/snapshot.json'))
    result=dict(status='terminal_current_state_collected',trial_result=json.loads((trial/'result.json').read_text()),
        child_exit_codes=codes,last_capture=str(captures[-1].parent) if captures else None,
        readiness=json.loads((root/'ready.json').read_text()) if (root/'ready.json').exists() else None,
        robot_commands_sent=False)
    with (root/'result.json').open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result),flush=True)
