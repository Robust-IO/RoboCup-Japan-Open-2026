"""Read-only demonstration intake. No ROS/control start in this helper."""
import hashlib,json,socket,subprocess,urllib.request
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');report=dict(robot_commands_sent=False,offline_package_activated=False)
d=json.loads((b/'outcome-policy-deployment.json').read_text());changed=[]
for row in d['installed']:
 p=b/row['name']
 if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:changed.append(row['name'])
report['active_deployment_files_changed']=changed
ports={}
for port in (8766,9090,50001):
 with socket.socket() as sock:sock.settimeout(.3);ports[str(port)]=sock.connect_ex(('127.0.0.1',port))==0
report['ports']=ports;procs=[]
scripts={'run_cigarette_transfer_20261101.py','flat_box_grasp.py','flat_box_contact.py','flat_box_carry.py','flat_box_place.py','navigate_carry_trial.py','table_localization_service.py','camera_browser_server_20261101.py','hold_cigarette_bridges_20261101.py'}
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  args=[s.decode(errors='replace') for s in (p/'cmdline').read_bytes().split(b'\0') if s]
  if len(args)>1 and Path(args[1]).name in scripts:procs.append(dict(pid=int(p.name),script=Path(args[1]).name))
 except (FileNotFoundError,PermissionError):pass
report['relevant_processes']=procs
if ports['8766']:
 try:
  response=urllib.request.urlopen('http://127.0.0.1:8766',timeout=2);report['camera_page_http_status']=response.status
 except Exception as exc:report['camera_page_error']=str(exc)
active=[p for p in procs if p['script'] in scripts-{'table_localization_service.py','camera_browser_server_20261101.py','hold_cigarette_bridges_20261101.py'}]
report['active_robot_tasks']=active
if not changed and not active and ports['9090'] and ports['50001']:
 attempt=23
 while any((b/(f'cigarette-readiness-20261101-{attempt:02d}'+s)).exists() for s in ('','-console.log','-launch.json')):attempt+=1
 r=subprocess.run(['/usr/bin/python3',str(b/'launch_flat_box_stage_20261101.py'),'readiness','--attempt',str(attempt)],capture_output=True,text=True)
 report['readiness_launcher_returncode']=r.returncode;report['readiness_output']=str(b/f'cigarette-readiness-20261101-{attempt:02d}')
 report['readiness_launcher_tail']=(r.stdout+r.stderr)[-1200:]
(b/'demo-preparation-latest.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
