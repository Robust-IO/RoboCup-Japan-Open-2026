"""Start the reviewed idle navigation launch; no goals or map anchor."""
import json
from pathlib import Path
import subprocess
import time

b = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
receipt = json.loads((b/'demo-background-latest.json').read_text())
root = Path(receipt['logs'])
names = {'map_server', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator'}
existing = []
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        args = (proc/'cmdline').read_bytes().split(b'\0')
        if args and Path(args[0].decode()).name in names:
            existing.append(int(proc.name))
    except (FileNotFoundError, PermissionError):
        pass
if existing:
    raise RuntimeError('Existing navigation processes; inspect before starting: '+repr(existing))
script = b/'restart_idle_map_navigation_20261018.launch.py'
cmd = ['bash', '-lc',
    'source /opt/ros/humble/setup.bash; '
    'source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/setup.bash; '
    'export ROS_LOG_DIR='+str(root/'roslogs')+'; '
    'exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py '
    '--domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml '
    '--run -- ros2 launch '+str(script)]
with (root/'idle-navigation.log').open('x') as stream:
    child = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=stream,
        stderr=subprocess.STDOUT, start_new_session=True)
time.sleep(3)
if child.poll() is not None:
    raise RuntimeError('Idle navigation exited; inspect log')
result = dict(pid=child.pid, log=str(root/'idle-navigation.log'),
    navigation_autostart=False, map_anchor_started=False,
    robot_commands_sent=False, launch=str(script))
(b/'demo-idle-navigation-latest.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result))
