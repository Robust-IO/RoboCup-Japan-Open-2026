"""Launch the existing dual RGB viewer; never publish robot commands."""
import json
import os
from pathlib import Path
import subprocess
import time

base = Path('/__HANDYMAN_PACKAGE__/runtime')
repo = Path('/__HANDYMAN_PACKAGE__/project')
root = base / ('dual-camera-20261101-' + str(time.time_ns()))
root.mkdir()
runtime = Path('/tmp/rviz-runtime-local')
runtime.mkdir(exist_ok=True)
runtime.chmod(0o700)
env = dict(os.environ, DISPLAY=':0', XDG_RUNTIME_DIR=str(runtime),
           QT_QPA_PLATFORM='xcb', LIBGL_ALWAYS_SOFTWARE='1',
           ROS_LOG_DIR=str(root / 'roslogs'))
command = ['bash', '-lc',
    'source /opt/ros/humble/setup.bash; '
    'exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py '
    '--domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml '
    '--run -- rviz2 -d /__HANDYMAN_PACKAGE__/project/training/config/live_head_hand_rgb.rviz '
    '--geometry 1200x760+30+30']
with (root / 'viewer.log').open('xb') as log:
    child = subprocess.Popen(command, env=env, stdin=subprocess.DEVNULL,
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
receipt = dict(pid=child.pid, root=str(root), command=command, robot_commands_sent=False)
(root / 'launch.json').write_text(json.dumps(receipt, indent=2))
print(json.dumps(receipt))
