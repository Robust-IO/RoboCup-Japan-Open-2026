"""Read-only model replay of the failed source frames at two resolutions."""
import json,os,sys,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(f))
from apple_worker_client import Worker
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text())
run=Path(json.loads((b/'penguin-kitchen-live-latest.json').read_text())['output'])
capture=run/'grasp/captures/identity-00'
rows=[]
for size in (960,1280):
    worker=Worker(cfg['model_python'],cfg['registry'],run/('identity-audit-'+str(size)+'.log'),imgsz=size)
    try:result=worker.infer(capture);rows.append(dict(imgsz=size,ready=worker.ready,result=result))
    finally:worker.close()
out=run/'identity-resolution-audit.json';out.write_text(json.dumps(dict(rows=rows,robot_commands_sent=False,archived_capture=True),indent=2))
print(json.dumps(dict(output=str(out),rows=rows,robot_commands_sent=False)))
