import os,sys
single='--single-thread' in sys.argv
if single:
    for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import json,time
from pathlib import Path
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
import rclpy
from pepper_runtime import PepperRuntime
from can_destination_surface import measure
from pepper_place import make_plan

base=Path('/__HANDYMAN_PACKAGE__/runtime');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
root=base/('pepper-destination-profile-20261024-'+('single' if single else 'default'));root.mkdir(exist_ok=False)
cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
held=json.loads((base/'pepper-native-pregrasp-continuation-20261024-07/held-context.json').read_text())
tables=json.loads(Path(cfg['semantic_map']).read_text())['destinations'];destination=next(t for t in tables if t['id']==cfg['destination_id'])
report=dict(robot_commands_sent=False,single_thread=single)
try:
    from threadpoolctl import threadpool_info
    report['threadpools']=threadpool_info()
except ImportError:report['threadpoolctl_available']=False
rclpy.init();runtime=PepperRuntime(root,'pepper-place-profile',lambda _:None)
try:
    sample=runtime.sample();s=sample['snapshot'];state=runtime.state()
    if abs(state['joints'][5]-held['holding_opening_rad'])>.02:raise ValueError('held_grip_changed')
    start=time.monotonic();plane=measure(sample['depth'],s,destination);report['table_measure_s']=time.monotonic()-start
    report['post_plane_oldest_image_age_s']=(runtime.node.get_clock().now().nanoseconds-min(s['streams'][k]['stamp_ns'] for k in ('rgb','depth')))/1e9
    report['plane']=plane
    start=time.monotonic();plan=make_plan(sample['depth'],s,destination,plane,held,cfg,'pepper-place-profile')
    report['placement_plan_s']=time.monotonic()-start;report['plan']=plan
    runtime.save(sample,'destination-profile')
    report['status']='profiled_without_motion';report['state']=runtime.state()
except Exception as exc:report.update(status='read_only_profile_failed',reason=str(exc))
finally:
    runtime.shutdown();rclpy.shutdown();(root/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
print(json.dumps({k:v for k,v in report.items() if k not in ('plane','plan')},allow_nan=False),flush=True)
