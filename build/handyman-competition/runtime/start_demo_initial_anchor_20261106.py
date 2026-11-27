"""Fresh LayoutA initial anchor after user confirmed restarted initial pose."""
import json
from pathlib import Path
import subprocess
import time
import yaml

b = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
repo = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
root = Path(json.loads((b/'demo-background-latest.json').read_text())['logs'])
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit(): continue
    try: args = (proc/'cmdline').read_bytes().split(b'\0')
    except (FileNotFoundError, PermissionError): continue
    if args and Path(args[0].decode(errors='replace')).name == 'handyman_initial_pose_localizer':
        raise RuntimeError('Initial anchor already running; inspect')
    if any(Path(v.decode(errors='replace')).name == 'table_localization_service.py' for v in args if v):
        raise RuntimeError('Corrector already running; inspect')
cfg = yaml.safe_load((repo/'src/handyman_rebuild_ros2/config/environments/layout_a.yaml').read_text())
pose = cfg['initial_pose']
command = 'ros2 run handyman_rebuild_ros2 handyman_initial_pose_localizer --ros-args -p map_frame:=map -p odom_frame:=odom -p base_frame:=base_footprint'
for k in ('x','y','yaw'): command += ' -p initial_pose.'+k+':='+str(float(pose[k]))
cmd = ['bash','-lc','source /opt/ros/humble/setup.bash; source '+str(repo/'install/setup.bash')+'; export ROS_LOG_DIR='+str(root/'roslogs')+'; exec /usr/bin/python3 '+str(repo/'training/scripts/cyclone_test_env.py')+' --domain 71 --dds-config '+str(repo/'training/config/cyclone_search_stack.xml')+' --run -- '+command]
with (root/'initial-anchor.log').open('x') as stream:
    child = subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(2)
if child.poll() is not None: raise RuntimeError('Initial anchor failed; see log')
print(json.dumps(dict(pid=child.pid,initial_pose=pose,robot_commands_sent=False)))
