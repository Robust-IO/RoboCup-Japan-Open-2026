import os,socket,ctypes,json,re,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');previous=json.loads((base/'rosbridge-stale-connections-cleanup-20261025.json').read_text())
if b'rosbridge_websocket' not in Path('/proc/48/cmdline').read_bytes():raise ValueError('bridge_process_changed')
number=int(re.search(r'#define __NR_pidfd_getfd (\d+)',Path('/usr/include/x86_64-linux-gnu/asm/unistd_64.h').read_text()).group(1))
libc=ctypes.CDLL(None,use_errno=True);pidfd=os.pidfd_open(48);rows=[]
try:
 for row in previous['old_idle_connections']:
  if row['peer']=='192.168.0.1:52570':continue
  fd=int(re.search(r'fd=(\d+)',row['owner']).group(1));host,port=row['peer'].rsplit(':',1)
  cloned=libc.syscall(number,pidfd,fd,0)
  if cloned<0:raise OSError(ctypes.get_errno(),os.strerror(ctypes.get_errno()))
  with socket.socket(fileno=cloned) as s:
   if s.getpeername()!=(host,int(port)) or s.getsockname()[1]!=9090:raise ValueError('endpoint_changed:'+row['peer'])
   s.shutdown(socket.SHUT_RDWR);rows.append(row['peer'])
finally:os.close(pidfd)
result=dict(saved_ns=time.time_ns(),disconnected_old_connections=rows,previous_single_disconnected='192.168.0.1:52570',
 protected_current_connections=[r['peer'] for r in previous['protected_other_connections']],robot_commands_sent=False)
with (base/'rosbridge-old-connection-shutdown-verified-20261025.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result),flush=True)
