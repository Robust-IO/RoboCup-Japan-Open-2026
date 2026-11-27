import json,subprocess,time
from pathlib import Path
root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/camera-browser-viewer-20261101')
root.mkdir(exist_ok=True)
receipt=root/'server-launch.json'
if receipt.exists():raise ValueError('existing_camera_page_server_must_be_reviewed')
cmd=['/usr/bin/python3','/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/camera_browser_server_20261101.py']
with (root/'server.log').open('xb') as log:
    child=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
row=dict(pid=child.pid,started_ns=time.time_ns(),url='http://127.0.0.1:8766',command=cmd,
         robot_commands_sent=False,ros_services_started_or_stopped=False)
receipt.write_text(json.dumps(row,indent=2));print(json.dumps(row))
