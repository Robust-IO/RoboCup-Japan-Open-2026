import os,socket,ctypes,json,re
from pathlib import Path
checkpoint=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/rosbridge-stale-connections-cleanup-20261025.json').read_text())
row=next(r for r in checkpoint['old_idle_connections'] if r['peer']=='192.168.0.1:52570')
fd=int(re.search(r'fd=(\d+)',row['owner']).group(1));pid=48
if b'rosbridge_websocket' not in Path('/proc/48/cmdline').read_bytes():raise ValueError('bridge_process_changed')
header=Path('/usr/include/x86_64-linux-gnu/asm/unistd_64.h').read_text()
number=int(re.search(r'#define __NR_pidfd_getfd (\d+)',header).group(1))
libc=ctypes.CDLL(None,use_errno=True);pidfd=os.pidfd_open(pid);cloned=libc.syscall(number,pidfd,fd,0)
if cloned<0:raise OSError(ctypes.get_errno(),os.strerror(ctypes.get_errno()))
try:
 s=socket.socket(fileno=cloned)
 if s.getpeername()!=('192.168.0.1',52570) or s.getsockname()[1]!=9090:raise ValueError('socket_endpoint_changed')
 s.shutdown(socket.SHUT_RDWR)
 print(json.dumps(dict(disconnected_old_peer=list(s.getpeername()),robot_commands_sent=False)))
finally:
 try:s.close()
 finally:os.close(pidfd)
