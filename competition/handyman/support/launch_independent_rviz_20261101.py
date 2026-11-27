"""Start only the RViz viewer on the separate X display. No robot commands."""
import json,os,subprocess,time
from pathlib import Path
root=Path('/__HANDYMAN_PACKAGE__/runtime/rviz-independent-display-20261101-01')
if (root/'viewer-launch.json').exists():raise ValueError('viewer_already_launched')
if not Path('/mnt/wslg/.X11-unix/X1').exists():raise ValueError('independent_X1_display_required')
env=dict(os.environ,DISPLAY=':1',QT_QPA_PLATFORM='xcb',LIBGL_ALWAYS_SOFTWARE='1',
    XDG_RUNTIME_DIR='/tmp/rviz-runtime-local',ROS_LOG_DIR=str(root/'roslogs'))
Path(env['XDG_RUNTIME_DIR']).mkdir(exist_ok=True);Path(env['XDG_RUNTIME_DIR']).chmod(0o700)
cmd=['bash','-lc','source /opt/ros/humble/setup.bash; '
    'exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py '
    '--domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml '
    '--run -- rviz2 -d /__HANDYMAN_PACKAGE__/project/training/config/live_head_hand_rgb.rviz '
    '--geometry 1200x740+20+20']
with (root/'viewer.log').open('xb') as log:
    child=subprocess.Popen(cmd,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
row=dict(pid=child.pid,command=cmd,display=':1',robot_commands_sent=False,ros_background_changed=False)
(root/'viewer-launch.json').write_text(json.dumps(row,indent=2));print(json.dumps(row))
