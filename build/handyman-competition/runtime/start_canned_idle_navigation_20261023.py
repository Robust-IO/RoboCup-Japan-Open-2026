"""Restore the already-reviewed idle map/Nav2 servers without any anchor or goal."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp':
    raise ValueError('reviewed_domain71_environment_required')
root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/idle-map-navigation-canned-20261023-01')
launch=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/restart_idle_map_navigation_20261018.launch.py')
if root.exists() or not launch.is_file():raise ValueError('new_output_and_reviewed_launch_required')
for name in ('map_server','controller_server','planner_server','bt_navigator'):
    if subprocess.run(['pgrep','-x',name],capture_output=True).returncode==0:raise ValueError('existing_service:'+name)
root.mkdir();command=['ros2','launch',str(launch)]
with (root/'launch.log').open('x') as stream:
    child=subprocess.Popen(command,env=dict(os.environ,ROS_LOG_DIR=str(root/'ros-logs')),
        stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(2)
if child.poll() is not None:raise RuntimeError('idle_map_navigation_exited')
result=dict(status='idle_map_navigation_started',pid=child.pid,command=command,
    navigation_autostart=False,map_anchor_started=False,robot_commands_sent=False,
    task_or_protocol_node_started=False,navigation_tolerances=[.06,.08,.06])
(root/'startup.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
