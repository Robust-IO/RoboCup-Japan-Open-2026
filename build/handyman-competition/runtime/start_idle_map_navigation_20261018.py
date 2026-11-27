"""Start idle map/navigation services only, preserving all previous session logs."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args()
if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71':raise ValueError('explicit_simulation_run_required')
root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/restart-idle-map-navigation-20261018-01')
launch=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/restart_idle_map_navigation_20261018.launch.py')
if root.exists() or not launch.is_file():raise ValueError('existing_output_or_missing_launch')
existing=subprocess.run(['pgrep','-x','map_server'],capture_output=True,text=True)
if existing.returncode==0:raise ValueError('existing_map_server_no_duplicate')
root.mkdir()
with (root/'launch.log').open('x') as log:
    proc=subprocess.Popen(['ros2','launch',str(launch)],env=dict(os.environ,ROS_LOG_DIR=str(root/'ros-logs')),
                          stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(2)
if proc.poll() is not None:raise RuntimeError('idle_launch_exited_see_log')
result=dict(status='idle_map_navigation_launch_started',pid=proc.pid,navigation_autostart=False,
            map_anchor_started=False,initial_pose_localizer_started=False,robot_motion_or_protocol_node_started=False,
            navigation_precision=[.06,.08,.06],logs=str(root))
(root/'startup.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
