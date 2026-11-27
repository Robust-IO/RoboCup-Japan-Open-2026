"""Replay one saved frame with the EXACT active model/resolution. No ROS."""
import json
from pathlib import Path
import sys

code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_worker_client import Worker
base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'can-auto-03-identity-audit';root.mkdir(exist_ok=False)
cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
capture=base/'can-auto-live-20261023-03/grasp/captures/identity-00'
worker=Worker(cfg['model_python'],cfg['registry'],root/'model-worker.log',imgsz=960)
try:
    result=worker.infer(capture);(root/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(model=worker.ready,capture=str(capture),detections=result.get('detections',[]),
        worker_status=result.get('status'),robot_commands_sent=False)),flush=True)
finally:worker.close()
