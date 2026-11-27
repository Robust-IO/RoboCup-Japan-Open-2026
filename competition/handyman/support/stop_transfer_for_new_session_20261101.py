"""Stop only the current automatic transfer tree before fresh-session intake."""
import json,os,signal,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');receipt=json.loads((b/'cigarette-full-auto-latest.json').read_text());root=Path(receipt['output']);r=json.loads((root/'result.json').read_text()) if (root/'result.json').exists() else {};pids=[receipt['pid']]
active=r.get(r.get('active_stage',''))
if active:
 launch=b/(Path(active).name+'-launch.json')
 if launch.exists():pids.append(json.loads(launch.read_text())['pid'])
def running(pid):
 try:return Path('/proc',str(pid),'stat').read_text().split()[2]!='Z'
 except FileNotFoundError:return False
for pid in pids:
 if running(pid):
  try:os.kill(pid,signal.SIGINT)
  except ProcessLookupError:pass
deadline=time.monotonic()+8
while time.monotonic()<deadline and any(running(pid) for pid in pids):time.sleep(.1)
out=dict(status='previous_transfer_stopped' if not any(running(pid) for pid in pids) else 'previous_transfer_still_exiting',pids=pids,root=str(root),previous_stage=r.get('active_stage'))
(b/'transfer-new-session-stop-latest.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
