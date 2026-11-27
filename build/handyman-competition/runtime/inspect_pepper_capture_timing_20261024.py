"""Read-only timing diagnosis at the current native pepper source position."""
import collections,json,sys,time
from pathlib import Path
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
import rclpy
import canned_grasp_runtime as transport
from synced_capture_selection import select_stamps as original
from pepper_runtime import PepperRuntime
from rclpy.time import Time

root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/pepper-native-capture-timing-20261024-01');root.mkdir(exist_ok=False)
counts=collections.Counter();latest={};rclpy.init()
runtime=PepperRuntime(root,'pepper-timing-readonly',lambda event:None)
def inspect(history,now_ns,last=None,**kwargs):
    result=original(history,now_ns,last,**kwargs)
    counts['selection_calls']+=1;counts['selected_default']+=result is not None
    latest.update(ages_s={k:None if not rows else (now_ns-max(rows))/1e9 for k,rows in history.items()},
        joint_age_s=(now_ns-kwargs['joint_stamp_ns'])/1e9)
    for reserve in (200_000_000,100_000_000,0):
        options=dict(kwargs,processing_reserve_ns=reserve)
        row=original(history,now_ns,last,**options)
        counts['selected_with_reserve_'+str(reserve)]+=row is not None
    for depth in sorted(history['depth'],reverse=True):
        if not 0<=now_ns-depth<=400_000_000:continue
        rgb=min(history['rgb'],key=lambda s:abs(s-depth),default=None)
        hand=min((s for s in history['hand'] if depth<=s<=depth+200_000_000),default=None)
        if rgb is None or hand is None or abs(rgb-depth)>30_000_000:continue
        a=history['rgb'][rgb].header.frame_id;b=history['depth'][depth].header.frame_id;c=history['hand'][hand].header.frame_id
        pairs=[('base_link',a,depth),(a,b,depth),('base_link',c,hand),('hand_palm_link',c,hand),
            ('odom','base_link',depth),('base_link','odom',hand),('map','base_link',depth),('base_link','hand_palm_link',depth)]
        missing=[t+'<-'+s for t,s,ns in pairs if not runtime.buf.can_transform(t,s,Time(nanoseconds=ns))]
        for name in missing:counts['missing_tf:'+name]+=1
        latest['best_pair_ages_s']=[(now_ns-v)/1e9 for v in (rgb,depth,hand)]
        latest['missing_tf']=missing
        break
    return result
transport.select_stamps=inspect
report=dict(robot_commands_sent=False)
try:
    try:
        sample=runtime.sample(timeout=4)
        report['default_sample']='passed';runtime.save(sample,'default')
    except Exception as exc:report['default_sample']=str(exc)
    try:
        sample=runtime.sample(timeout=4,processing_reserve_ns=0)
        report['zero_reserve_sample']='passed';report['capture']=str(runtime.save(sample,'zero-reserve'))
    except Exception as exc:report['zero_reserve_sample']=str(exc)
    report.update(counts=dict(counts),latest=latest,state=runtime.state())
finally:
    runtime.shutdown();rclpy.shutdown()
    (root/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
print(json.dumps(report,allow_nan=False),flush=True)
