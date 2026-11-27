"""Own the existing two transports, with the native startup clock service enabled."""
import argparse,json,os,signal,socket,subprocess,time
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp':raise ValueError('domain71_cyclone_required')
 if not a.binary.is_file():raise ValueError('clock_bridge_binary_missing')
 for port in (9090,50001):
  with socket.socket() as s:s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);s.bind(('0.0.0.0',port))
 a.output.mkdir(parents=True,exist_ok=False);children={};env=dict(os.environ,ROS_LOG_DIR=str(a.output/'ros-logs'))
 commands={'rosbridge':['ros2','launch','rosbridge_server','rosbridge_websocket_launch.xml','port:=9090'],
           'sigverse':[str(a.binary),'50001','65535']}
 def stop(*_):raise KeyboardInterrupt()
 signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
 try:
  for key,args in commands.items():
   with (a.output/(key+'.log')).open('x') as f:children[key]=subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
  (a.output/'startup.json').write_text(json.dumps(dict(pids={k:p.pid for k,p in children.items()},commands=commands,robot_commands_sent=False,boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()),indent=2))
  while True:
   if any(p.poll() is not None for p in children.values()):raise RuntimeError('owned_bridge_exited')
   time.sleep(.5)
 except KeyboardInterrupt:pass
 finally:
  for p in children.values():
   if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  for p in children.values():
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:
    os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
if __name__=='__main__':main()
