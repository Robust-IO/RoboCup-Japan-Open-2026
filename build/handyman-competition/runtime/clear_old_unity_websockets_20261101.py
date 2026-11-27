"""Close only the eight observed pre-restart ROS websocket connections.

No bridge process, sensor TCP connection, browser connection or robot command
is changed. Exact endpoints, idle-age separation and task lock are checked.
"""
import fcntl,json,re,subprocess,time
from pathlib import Path
old={57102,57105,57107,57108,57113,57123,57125,57126}
current={64938,64939,64940,64941,64942,64943,64945,64946}
output=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/old-unity-websocket-cleanup-20261101-01.json')
if output.exists():raise ValueError('cleanup_already_recorded')
def inspect():
 text=subprocess.check_output(['ss','-tinp','sport = :9090'],text=True);rows={};last=None
 for line in text.splitlines():
  if line.startswith('ESTAB'):
   m=re.search(r'192\.168\.0\.2:9090\s+192\.168\.0\.1:(\d+)',line);last=int(m[1]) if m else None
  elif last is not None:
   m=re.search(r'\blas trcv:(\d+)'.replace(' ',''),line)
   if m:rows[last]=int(m[1])
 return rows
with open('/tmp/handyman-apple-transfer-automation.lock','r') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 before=inspect()
 if not current.issubset(before):raise ValueError('current_unity_connection_group_changed')
 chosen=old&before.keys()
 if not chosen or min(before[x] for x in chosen)<max(before[x] for x in current)+300000:
  raise ValueError('old_connection_age_not_separated_from_current_play')
 result=dict(started_ns=time.time_ns(),before_idle_ms=before,current_ports_preserved=sorted(current),closed=[],robot_commands_sent=False)
 for port in sorted(chosen):
  cmd=['ss','-K','src','192.168.0.2','sport','=','9090','dst','192.168.0.1','dport','=',str(port)]
  p=subprocess.run(cmd,capture_output=True,text=True,check=True)
  result['closed'].append(dict(peer_port=port,result=p.stdout.strip()))
 result['after_idle_ms']=inspect()
 with output.open('x') as f:json.dump(result,f,indent=2)
 print(json.dumps(result))
