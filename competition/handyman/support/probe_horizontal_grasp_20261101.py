import json,sys,time,uuid
from pathlib import Path
sys.path[:0]=['/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1','/__HANDYMAN_PACKAGE__/runtime']
import rclpy
from rclpy.time import Time
from pepper_runtime import PepperRuntime
from apple_worker_client import Worker
from horizontal_can_geometry import estimate
root=Path('/__HANDYMAN_PACKAGE__/runtime')/('horizontal-can-fresh-'+str(time.time_ns()))
root.mkdir();rclpy.init();runtime=worker=None;report=dict(robot_commands_sent=False)
try:
    runtime=PepperRuntime(root,'horizontal-fresh-'+uuid.uuid4().hex,lambda _:None)
    cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text())
    worker=Worker(cfg['model_python'],cfg['registry'],root/'model.json',imgsz=640)
    sample=runtime.sample(processing_reserve_ns=0);s=sample['snapshot'];capture=runtime.save(sample,'source')
    result=worker.infer(capture,pump=runtime.pump)
    (capture/'detections.json').write_text(json.dumps(result,indent=2))
    cans=[d for d in result['detections'] if d['name']=='canned_juice']
    if len(cans)!=1:raise ValueError('one_visible_can_required')
    geometry=estimate(sample['rgb'],sample['depth'],s,cans[0])
    when=Time(nanoseconds=s['streams']['depth']['stamp_ns'])
    transforms={}
    for link in ['arm_lift_link','arm_flex_link','arm_roll_link','wrist_flex_link','wrist_roll_link','hand_palm_link',
                 'hand_l_proximal_link','hand_r_proximal_link','hand_l_distal_link','hand_r_distal_link']:
        transforms[link]=runtime.tf_dict(runtime.buf.lookup_transform('base_link',link,when).transform)
    report.update(status='current_can_measured',capture=str(capture),geometry=geometry,
                  transforms=transforms,state=runtime.state())
except Exception as exc:report.update(status='observation_failed',reason=str(exc))
finally:
    if worker:worker.close()
    if runtime:runtime.shutdown()
    rclpy.shutdown();(root/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(dict(root=str(root),**report)))
