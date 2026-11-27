"""Run only after operator has stopped Play; restart the known bridge owner.

Preserves ROS navigation, map, model and camera web-page server. No robot motion.
"""
import argparse,json,os,signal,socket,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--unity-stopped',action='store_true');a=p.parse_args()
if not a.unity_stopped:raise ValueError('stop_Unity_Play_before_restarting_connections')
b=Path('/__HANDYMAN_PACKAGE__/runtime');receipt=b/'cigarette-task-bridge-restart-20261101-01.json'
if receipt.exists():raise ValueError('restart_receipt_already_exists')
owners=[]
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:cmd=proc.joinpath('cmdline').read_bytes().split(bytes([0]))
 except FileNotFoundError:continue
 if len(cmd)>1 and cmd[1]==b'/__HANDYMAN_PACKAGE__/runtime/hold_canned_bridges_20261023.py':owners.append(int(proc.name))
if len(owners)!=1:raise ValueError('unique_known_old_bridge_owner_required')
new=b/'hold_cigarette_bridges_20261101.py'
old=(b/'hold_canned_bridges_20261023.py').read_text()
with new.open('x') as f:f.write(old.replace('hold-camera-bridges-canned-20261023-01','hold-camera-bridges-thin-20261101-01'))
os.kill(owners[0],signal.SIGTERM)
deadline=time.monotonic()+20
while time.monotonic()<deadline:
 free=True
 for port in (9090,50001):
  with socket.socket() as sock:
   sock.settimeout(.3)
   if sock.connect_ex(('127.0.0.1',port))==0:free=False
 if free:break
 time.sleep(.2)
else:raise ValueError('old_bridge_ports_not_released')
command=['bash','-lc','source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; '
 'export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/cigarette-bridge-restart-20261101-01-roslogs; '
 'exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 '
 '--dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 '+str(new)]
with (b/'cigarette-bridge-restart-20261101-01-console.log').open('x') as log:
 child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
out=dict(started_ns=time.time_ns(),old_owner_pid=owners[0],new_owner_launcher_pid=child.pid,command=command,robot_commands_sent=False,
 task=dict(object='cigarette',source_room='bedroom',destination='square_low_table'),next_action='wait_until_bridges_listen_then_enter_Unity_Play_and_check_unique_command_receivers')
receipt.write_text(json.dumps(out,indent=2));print(json.dumps(out))
