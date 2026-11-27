"""Retire identified Handyman owners before starting a new competition."""
import argparse,fcntl,json,os,signal,socket,time
from pathlib import Path
PORTS={9090,50001}
ROOTS=('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/','/home/crazylearner/handyman-worktrees/','/home/crazylearner/RoboCup-Japan-Open-2026/','/mnt/d/handyman-training-runs/','/mnt/d/handyman-tools/')
SCRIPTS={'apple_shared_inference.py','apple_inference_worker.py','run_sessions12.py','run_sessions123.py','run_live_session1.py','run_dynamic_session.py','run_session3_search.py','start_session1_round.py','start_session3_round.py','start_session4_round.py','start_session5_round.py','start_session6_round.py','session_services.launch.py','session1_services.launch.py','hold_clock_bridges.py','hold_demo_bridges.py'}
def snapshot():
 rows={}
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   raw=(p/'stat').read_text().rsplit(') ',1)[1].split()
   if raw[0]=='Z':continue
   args=[x.decode(errors='replace') for x in (p/'cmdline').read_bytes().split(b'\0') if x]
   env=(p/'environ').read_bytes().split(b'\0')
   rows[int(p.name)]=dict(pid=int(p.name),ppid=int(raw[1]),start=raw[19],uid=p.stat().st_uid,args=args,domain71=b'ROS_DOMAIN_ID=71' in env)
  except (OSError,ValueError,IndexError):continue
 return rows

def listeners(rows):
 inodes={}
 for table in ('tcp','tcp6'):
  for line in Path('/proc/net',table).read_text().splitlines()[1:]:
   a=line.split();port=int(a[1].split(':')[1],16)
   if port in PORTS and a[3]=='0A':inodes[a[9]]=port
 owners={port:set() for port in PORTS}
 for pid in rows:
  try:
   for fd in Path('/proc',str(pid),'fd').iterdir():
    try:target=os.readlink(fd)
    except OSError:continue
    if target.startswith('socket:[') and target[8:-1] in inodes:owners[inodes[target[8:-1]]].add(pid)
  except OSError:continue
 return owners

def trusted(row,port_owner=False):
 if row['uid']!=os.getuid():return False
 args=row['args']
 for a in args:
  if a.startswith(ROOTS) and Path(a).name in SCRIPTS:return True
 if args and args[0].startswith(ROOTS) and Path(args[0]).name in ('sigverse_ros_bridge','handyman_initial_pose_localizer'):return True
 # System ROS executables are accepted only as actual bridge listeners in domain71.
 return bool(port_owner and row['domain71'] and any(a.startswith('/opt/ros/humble/') and Path(a).name in ('rosbridge_websocket','rosbridge_websocket.py','sigverse_ros_bridge') for a in args))

def targets(rows,owners):
 selected={pid for pid,r in rows.items() if trusted(r,pid in set().union(*owners.values()))}
 while True:
  children={pid for pid,r in rows.items() if r['ppid'] in selected and r['uid']==os.getuid()}
  if children<=selected:break
  selected|=children
 # Never target this checker or any of its ancestors.
 pid=os.getpid()
 while pid in rows:selected.discard(pid);pid=rows[pid]['ppid']
 unknown={port:sorted(pids-selected) for port,pids in owners.items() if pids-selected}
 return {pid:rows[pid] for pid in selected},unknown

def alive(row):
 now=snapshot().get(row['pid'])
 return now is not None and now['start']==row['start'] and now['uid']==row['uid']

def lock_holders(path):
 target=Path(path).stat();rows=snapshot();holders=[]
 for pid,row in rows.items():
  try:
   for fd in Path('/proc',str(pid),'fd').iterdir():
    try:st=fd.stat()
    except OSError:continue
    if (st.st_dev,st.st_ino)==(target.st_dev,target.st_ino):
     holders.append(dict(pid=pid,args=row['args']));break
  except OSError:continue
 return holders

def wait_control_lock(path='/tmp/handyman-apple-transfer-automation.lock',timeout=5.):
 # Opening an existing lock inode is essential; never unlink or replace it.
 deadline=time.monotonic()+timeout;announced=False
 with open(path,'a') as stream:
  while True:
   try:
    fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
    return
   except BlockingIOError as exc:
    if not announced:
     print('等待旧进程释放机器人控制锁……',flush=True);announced=True
    if time.monotonic()>=deadline:
     raise RuntimeError('机器人控制锁仍被占用，实际持有者：'+json.dumps(lock_holders(path),ensure_ascii=False)) from exc
    time.sleep(.1)

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');ap.add_argument('--grace-seconds',type=float,default=15);a=ap.parse_args()
 if not 0<=a.grace_seconds<=30:ap.error('grace-seconds must be between 0 and 30')
 lock=open('/tmp/handyman-competition-cleanup.lock','a')
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 rows=snapshot();owners=listeners(rows);selected,unknown=targets(rows,owners)
 print(json.dumps(dict(event='startup_occupancy',ports={k:sorted(v) for k,v in owners.items()},identified_processes=[dict(pid=p,args=r['args']) for p,r in selected.items()],unknown_port_owners=unknown,execute=a.execute),ensure_ascii=False),flush=True)
 if unknown:raise RuntimeError('端口由无关程序占用，未自动结束：'+str(unknown))
 if not a.execute:return
 # SIGINT lets task owners cancel commands and perform their normal cleanup.
 for sig,seconds in ((signal.SIGINT,a.grace_seconds),(signal.SIGTERM,5),(signal.SIGKILL,3)):
  current=snapshot();remaining={p:r for p,r in selected.items() if p in current and current[p]['start']==r['start']}
  if not remaining:break
  print(json.dumps(dict(event='retire_old_handyman',signal=sig.name,pids=sorted(remaining))),flush=True)
  for pid,row in remaining.items():
   if alive(row):
    try:os.kill(pid,sig)
    except ProcessLookupError:pass
  deadline=time.monotonic()+seconds
  while time.monotonic()<deadline:
   current=snapshot()
   if not any(p in current and current[p]['start']==r['start'] for p,r in remaining.items()):break
   time.sleep(.2)
 current=snapshot()
 if any(p in current and current[p]['start']==r['start'] for p,r in selected.items()):raise RuntimeError('旧任务未退出，取消新启动')
 # Recheck OS binds, including listeners that could not be read from /proc.
 for port in sorted(PORTS):
  with socket.socket() as sock:
   sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
   try:sock.bind(('0.0.0.0',port))
   except OSError as exc:raise RuntimeError(f'端口 {port} 仍被占用，取消新启动') from exc
 # Do not unlink a flock file: that could allow two independent lock owners.
 wait_control_lock()
 print('旧 Handyman 任务已清理，9090/50001 端口和机器人控制锁可用。',flush=True)
if __name__=='__main__':main()
