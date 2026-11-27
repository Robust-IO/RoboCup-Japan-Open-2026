"""Restore only this task's bridges and idle map services after display restart."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args()
if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp':
    raise ValueError('explicit_domain71_cyclone_run_required')
root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/restart-after-graphics-20261018-01')
if root.exists():raise ValueError('new_recovery_output_required')
for port in (9090,50001):
    with socket.socket() as sock:sock.bind(('0.0.0.0',port))
if subprocess.run(['pgrep','-x','map_server'],capture_output=True).returncode==0:
    raise ValueError('existing_map_server_no_duplicate')
saved=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/restart-camera-bridges-20261018-01/startup.json').read_text())
commands=saved['commands']
commands['idle_map_navigation']=['ros2','launch','/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/restart_idle_map_navigation_20261018.launch.py']
root.mkdir();children={}
try:
    for name,cmd in commands.items():
        with (root/(name+'.log')).open('x') as log:
            children[name]=subprocess.Popen(cmd,env=dict(os.environ,ROS_LOG_DIR=str(root/'ros-logs')),
                stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+20
    while time.monotonic()<deadline:
        if any(proc.poll() is not None for proc in children.values()):raise RuntimeError('owned_service_exited_see_logs')
        listening=subprocess.check_output(['ss','-ltn'],text=True)
        if all(':'+str(port)+' ' in listening for port in (9090,50001)):break
        time.sleep(.2)
    else:raise RuntimeError('bridge_listener_timeout')
    result=dict(status='services_restored_after_desktop_first_boot',pids={k:v.pid for k,v in children.items()},
        commands=commands,map_anchor_started=False,navigation_autostart=False,robot_motion_or_task_protocol_started=False,
        logs=str(root))
    (root/'startup.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
except BaseException:
    for proc in children.values():
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
    raise
