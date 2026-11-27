import json,subprocess,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');name='cigarette-full-auto-'+time.strftime('%Y%m%d-%H%M%S');out=b/name
with (b/(name+'-console.log')).open('xb') as log:
 child=subprocess.Popen(['/usr/bin/python3',str(b/'run_cigarette_transfer_20261101.py'),'--output',str(out),'--readiness-run',str(b/'cigarette-readiness-20261101-21'),'--resume-at-source','--tracked-source',str(b/'cigarette-current-target-reference-20261101.json')],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
r=dict(pid=child.pid,output=str(out),continued_task='cigarette-full-auto-20261101-193127');(b/'cigarette-full-auto-latest.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))
