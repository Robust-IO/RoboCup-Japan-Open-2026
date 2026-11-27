"""Read-only two-task GPU model-lifetime benchmark, archived frames only."""
import json,os,sys,time,subprocess
from pathlib import Path
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from resident_inference import connect
base=Path('/__HANDYMAN_PACKAGE__/runtime')
output=base/'resident-model-benchmark-20261026-01.json'
if output.exists():raise RuntimeError('benchmark_already_recorded')
cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
frames=[base/'penguin-unified-transfer-20261025-03/grasp/captures'/('identity-%02d'%i) for i in (1,2)]
rows=[]
for frame in frames:
    start=time.monotonic()
    client=connect(cfg['model_python'],cfg['registry'],imgsz=640)
    attached=time.monotonic();result=client.infer(frame)
    rows.append(dict(ready=client.ready,client_attach_wall_s=attached-start,
        request_wall_s=time.monotonic()-attached,capture=str(frame),
        detections=result.get('detections'),depth_stamp_ns=result.get('depth_stamp_ns'),
        worker_processing_s=result.get('worker_processing_s'),archived_readonly_replay=True))
    client.close()
report=dict(status='two_task_resident_reuse_verified',rows=rows,
    same_model_pid=rows[0]['ready']['model_pid']==rows[1]['ready']['model_pid'],
    robot_commands_sent=False,live_freshness_not_asserted=True,saved_ns=time.time_ns())
try:
    report['gpu']=subprocess.run(['nvidia-smi','--query-gpu=name,memory.used,memory.total','--format=csv,noheader'],capture_output=True,text=True,timeout=10).stdout.strip()
except (OSError,subprocess.SubprocessError) as exc:report['gpu_query_error']=str(exc)
output.write_text(json.dumps(report,indent=2))
print(json.dumps(report))
