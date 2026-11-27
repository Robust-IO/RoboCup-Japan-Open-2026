import json,subprocess,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');name='cigarette-full-auto-'+time.strftime('%Y%m%d-%H%M%S');out=b/name
with (b/(name+'-console.log')).open('xb') as stream:child=subprocess.Popen(['/usr/bin/python3',str(b/'run_cigarette_transfer_20261101.py'),'--output',str(out),'--readiness-run',str(b/'cigarette-readiness-20261101-22'),'--resume-at-source'],stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
r=dict(pid=child.pid,output=str(out),continued_task='cigarette-full-auto-20261101-202059',source_navigation='cigarette-source-20261101-05');(b/'cigarette-full-auto-latest.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))
