"""Restore tested transports and browser page, without control or protocol nodes."""
import json,socket,subprocess,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');stamp=time.strftime('%Y%m%d-%H%M%S');root=b/('demo-background-'+stamp);root.mkdir();report=dict(robot_commands_sent=False,logs=str(root))
def listening(port):
 with socket.socket() as sock:sock.settimeout(.3);return sock.connect_ex(('127.0.0.1',port))==0
if not listening(9090) and not listening(50001):
 src=(b/'hold_cigarette_bridges_20261101.py').read_text();old="root = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/hold-camera-bridges-thin-20261101-02')"
 if src.count(old)!=1:raise ValueError('reviewed_bridge_supervisor_changed')
 target=root/'hold_demo_bridges.py';target.write_text(src.replace(old,'root = Path('+repr(str(root/'bridge-logs'))+')'))
 cmd=['bash','-lc','source /opt/ros/humble/setup.bash; source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/setup.bash; export ROS_LOG_DIR='+str(root/'roslogs')+'; exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 '+str(target)]
 with (root/'bridges.log').open('x') as stream:child=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
 report['bridge_supervisor_pid']=child.pid
if not listening(8766):
 with (root/'camera-page.log').open('x') as stream:child=subprocess.Popen(['/usr/bin/python3',str(b/'camera_browser_server_20261101.py')],stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
 report['camera_page_pid']=child.pid
deadline=time.monotonic()+15
while time.monotonic()<deadline and not all(listening(p) for p in (9090,50001,8766)):time.sleep(.2)
report['ports']={str(p):listening(p) for p in (9090,50001,8766)}
report['startup_candidates']=[p.name for p in b.glob('*.py') if any(v in p.name for v in ('boot','startup','nav_stack','start_nav','launch_nav','runtime'))]
report['runtime_folders']=[p.name for p in b.glob('*runtime*') if p.is_dir()]
(b/'demo-background-latest.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
