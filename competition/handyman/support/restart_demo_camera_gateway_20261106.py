import json,os,signal,subprocess,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');script=b/'demo_camera_gateway_20261106.py'
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:args=[v.decode() for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    except (FileNotFoundError,PermissionError,UnicodeError):continue
    if len(args)>1 and args[0]=='/usr/bin/python3' and str(script) in args[1:3]:os.kill(int(proc.name),signal.SIGTERM)
time.sleep(.3)
log=b/('demo-camera-gateway-'+time.strftime('%Y%m%d-%H%M%S')+'.log')
with log.open('x') as stream:
    child=subprocess.Popen(['/usr/bin/python3','-u',str(script)],stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(.5)
if child.poll() is not None:raise RuntimeError(log.read_text())
report=dict(pid=child.pid,log=str(log),url='http://100.71.59.3:18766',robot_control=False)
(b/'demo-camera-gateway-latest.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
