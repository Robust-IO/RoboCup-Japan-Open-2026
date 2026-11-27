"""Read-only source test with the actual model loaded and fresh final checks."""
import json,sys,time
from pathlib import Path
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
import rclpy
from apple_worker_client import Worker
from pepper_runtime import PepperRuntime
from pepper_transfer_automation import confirmed_peppers
from pepper_geometry import estimate_source
from canned_carry_geometry import current_images_and_feedback

root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/pepper-native-model-timing-20261024-02');root.mkdir(exist_ok=False)
cfg=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text())
rclpy.init();runtime=PepperRuntime(root,'pepper-native-model-timing',lambda _:None);worker=None
rows=[]
try:
    worker=Worker(cfg['model_python'],cfg['registry'],root/'model-worker.log',imgsz=640)
    for i in range(6):
        row={};start=time.monotonic()
        try:
            sample=runtime.sample(timeout=3,processing_reserve_ns=0);s=sample['snapshot']
            captured=runtime.node.get_clock().now().nanoseconds
            row['capture_ages_s']={k:(captured-s['streams'][k]['stamp_ns'])/1e9 for k in ('rgb','depth','hand')}
            row['joint_age_before_s']=(captured-s['joint_stamp_ns'])/1e9
            capture=runtime.save(sample,'identity-'+str(i));row['capture']=str(capture)
            raw=worker.infer(capture,pump=lambda:runtime.pump(0.));row['detections']=confirmed_peppers(raw['detections'])
            (capture/'raw-detector-result.json').write_text(json.dumps(raw,indent=2))
            if len(row['detections'])==1:row['measurement']=estimate_source(sample['rgb'],sample['depth'],s,row['detections'][0])
            row['timing']=current_images_and_feedback(s,runtime.state(),runtime.node.get_clock().now().nanoseconds,runtime.epoch)
            row['status']='fresh_source_passed'
        except Exception as exc:row.update(status='rejected',reason=str(exc))
        row['elapsed_s']=time.monotonic()-start;rows.append(row);print(json.dumps(row,allow_nan=False),flush=True)
finally:
    if worker:worker.close()
    runtime.shutdown();rclpy.shutdown()
    (root/'result.json').write_text(json.dumps(dict(rows=rows,robot_commands_sent=False),indent=2,allow_nan=False))
