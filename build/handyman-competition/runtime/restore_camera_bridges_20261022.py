"""Start only camera/control transports; no motion/protocol publishers or task node."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp':
    raise ValueError('reviewed_domain71_cyclone_environment_required')
root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/restart-camera-bridges-20261022-01')
binary=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/bridge-framing-install-20261010/sigverse_ros_bridge/lib/sigverse_ros_bridge/sigverse_ros_bridge')
if root.exists() or not binary.is_file():raise ValueError('new_output_and_existing_bridge_required')
for port in (9090,50001):
    with socket.socket() as sock:sock.bind(('0.0.0.0',port))
root.mkdir();env=dict(os.environ,ROS_LOG_DIR=str(root/'ros-logs'))
commands={'rosbridge':['ros2','launch','rosbridge_server','rosbridge_websocket_launch.xml','port:=9090'],
          'sigverse':[str(binary),'50001','65535']}
children={}
try:
    for name,cmd in commands.items():
        with (root/(name+'.log')).open('x') as stream:
            children[name]=subprocess.Popen(cmd,env=env,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+20
    while time.monotonic()<deadline:
        if any(proc.poll() is not None for proc in children.values()):raise RuntimeError('owned_bridge_exited')
        listening=subprocess.check_output(['ss','-ltn'],text=True)
        if all(':'+str(port)+' ' in listening for port in (9090,50001)):break
        time.sleep(.2)
    else:raise RuntimeError('bridge_listener_timeout')
    value=dict(status='bridges_listening',pids={k:v.pid for k,v in children.items()},commands=commands,
        domain=71,robot_motion_or_protocol_node_started=False,unity_data_verified=False,logs=str(root))
    (root/'startup.json').write_text(json.dumps(value,indent=2));print(json.dumps(value),flush=True)
except BaseException:
    for child in children.values():
        if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
    raise
