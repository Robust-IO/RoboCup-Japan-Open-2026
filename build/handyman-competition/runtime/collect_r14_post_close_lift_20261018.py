"""New bounded idle cameras/readiness after the runner exits; NO control."""
import json
from pathlib import Path
import subprocess
import sys
code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
out=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/apple-r14-post-close-lift-20261018-01')
with RobotLock():
    out.mkdir(exist_ok=False);children={};logs={}
    commands=dict(
        cameras=[sys.executable,str(code/'capture_grasp_scene.py'),str(out/'cameras'),'--hand-geometry','--map-geometry',
            '--tracking-epoch',out.name,'--duration','7','--interval','1'],
        ready=[sys.executable,str(code/'inspect_transfer_ready.py'),'--output',str(out/'ready.json')])
    for name,command in commands.items():
        logs[name]=(out/(name+'.log')).open('x')
        children[name]=subprocess.Popen(command,stdout=logs[name],stderr=subprocess.STDOUT,start_new_session=True)
    exits={name:child.wait(timeout=25) for name,child in children.items()}
    for log in logs.values():log.close()
    captures=sorted((out/'cameras').glob('*/snapshot.json'))
    result=dict(status='current_post_run_idle_evidence_collected',child_exit_codes=exits,
        last_capture=str(captures[-1].parent) if captures else None,
        readiness=json.loads((out/'ready.json').read_text()) if (out/'ready.json').exists() else None,
        robot_commands_sent=False)
    write_json(out/'result.json',result);print(json.dumps(result),flush=True)
