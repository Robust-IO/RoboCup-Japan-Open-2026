"""Start/reuse the reviewed native-time-sync bridge before competition starts.

Only retire bridge PIDs recorded by the existing bridge supervisor. Never kill
arbitrary ROS processes, change system clocks, or send robot commands.
"""
import datetime,hashlib,json,os,signal,socket,subprocess,time
from pathlib import Path
RUNS=Path('/__HANDYMAN_PACKAGE__/runtime')

def process_args(pid):
 try:return [x.decode(errors='replace') for x in (Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0') if x]
 except FileNotFoundError:return []

def ports_free():
 for port in (9090,50001):
  try:
   with socket.socket() as sock:sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);sock.bind(('0.0.0.0',port))
  except OSError:return False
 return True

def ensure():
 config=json.loads((RUNS/'clock-bridge-active.json').read_text());binary=Path(config['binary'])
 if hashlib.sha256(binary.read_bytes()).hexdigest()!=config['sha256']:raise ValueError('clock_bridge_binary_changed')
 pointer=RUNS/'bridge-clean-latest.json';old=json.loads(pointer.read_text()) if pointer.exists() else None
 if old:
  root=Path(old['root']);startup=root/'bridge-logs/startup.json'
  record=json.loads(startup.read_text()) if startup.exists() else {}
  same_boot=record.get('boot_id')==Path('/proc/sys/kernel/random/boot_id').read_text().strip()
  args=process_args(record.get('pids',{}).get('sigverse',0)) if same_boot else []
  ws_args=process_args(record.get('pids',{}).get('rosbridge',0)) if same_boot else []
  if str(binary) in args and ws_args and process_args(old['pid']):
   print(json.dumps(dict(event='native_clock_bridge_reused',root=str(root),robot_commands_sent=False)),flush=True);return old
  owner=process_args(old['pid']) if same_boot else []
  approved=[str(root/'hold_demo_bridges.py'),str(Path(__file__).with_name('hold_clock_bridges.py'))]
  if owner:
   if not any(p in owner for p in approved):raise ValueError('recorded_bridge_pid_no_longer_owned')
   os.kill(old['pid'],signal.SIGTERM)
   deadline=time.monotonic()+20
   while process_args(old['pid']) and time.monotonic()<deadline:time.sleep(.1)
   if process_args(old['pid']):raise RuntimeError('old_bridge_supervisor_not_retired')
 deadline=time.monotonic()+10
 while not ports_free() and time.monotonic()<deadline:time.sleep(.1)
 if not ports_free():raise RuntimeError('bridge_ports_owned_by_unrecorded_or_remaining_process')
 root=RUNS/('native-clock-bridge-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));root.mkdir()
 with (root/'supervisor.log').open('x') as f:
  child=subprocess.Popen(['/usr/bin/python3',str(Path(__file__).with_name('hold_clock_bridges.py')),'--binary',str(binary),'--output',str(root/'bridge-logs')],stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
 result=dict(pid=child.pid,root=str(root),previous_supervisor=None if old is None else old['pid'],robot_commands_sent=False)
 if old:(root/'previous-bridge.json').write_text(json.dumps(old,indent=2))
 pointer.write_text(json.dumps(result,indent=2))
 deadline=time.monotonic()+20
 while time.monotonic()<deadline:
  if child.poll() is not None:raise RuntimeError('native_clock_bridge_start_failed:'+str(root))
  if (root/'bridge-logs/startup.json').exists():
   # Both listeners must accept connections; close immediately without BSON or
   # WebSocket data. No robot/competition message is produced.
   try:
    for port in (9090,50001):
     with socket.create_connection(('127.0.0.1',port),timeout=.2):pass
    print(json.dumps(dict(event='native_clock_bridge_ready',**result)),flush=True);return result
   except OSError:pass
  time.sleep(.2)
 raise RuntimeError('native_clock_bridge_listener_timeout:'+str(root))
if __name__=='__main__':ensure()
