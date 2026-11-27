import argparse,subprocess,re,json,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--apply',action='store_true');a=p.parse_args()
raw=subprocess.run(['/usr/bin/ss','-tinp','sport','=',':9090'],capture_output=True,text=True,check=True).stdout
rows=[];last=None
for line in raw.splitlines():
 if line.startswith('ESTAB'):
  fields=line.split();last=dict(local=fields[3],peer=fields[4],owner=' '.join(fields[5:]));rows.append(last)
 elif last is not None:
  last['stats']={k:int(v) for k,v in re.findall(r'(lastsnd|lastrcv|lastack|bytes_received|bytes_sent):(\d+)',line)}
old=[r for r in rows if r['peer'].startswith('192.168.0.1:') and 'pid=48,' in r['owner'] and max(r['stats'].get('lastsnd',0),r['stats'].get('lastrcv',0))>3_600_000]
new=[r for r in rows if r not in old]
report=dict(inspection_ns=time.time_ns(),old_idle_connections=old,protected_other_connections=new,apply=a.apply,robot_commands_sent=False)
if a.apply:
 if not old or len(old)>20 or not new:raise ValueError('unexpected_connection_groups_do_not_disconnect')
 report['disconnect_results']=[]
 for row in old:
  host,port=row['peer'].rsplit(':',1)
  cmd=['/usr/bin/ss','-K','sport','=',':9090','dst',host,'dport','=',':'+port]
  result=subprocess.run(cmd,capture_output=True,text=True)
  report['disconnect_results'].append(dict(peer=row['peer'],returncode=result.returncode,stdout=result.stdout,stderr=result.stderr))
  if result.returncode:break
 output=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/rosbridge-stale-connections-cleanup-20261025.json')
 with output.open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report),flush=True)
