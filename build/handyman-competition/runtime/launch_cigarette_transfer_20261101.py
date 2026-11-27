import json,subprocess,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');name='cigarette-full-auto-'+time.strftime('%Y%m%d-%H%M%S');out=b/name
with (b/(name+'-console.log')).open('xb') as log:
    child=subprocess.Popen(['/usr/bin/python3',str(b/'run_cigarette_transfer_20261101.py'),'--output',str(out),'--readiness-run',str(b/'cigarette-readiness-20261101-20')],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
r=dict(pid=child.pid,output=str(out));(b/'cigarette-full-auto-latest.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))
