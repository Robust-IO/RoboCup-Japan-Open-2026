import json,sys,time
from pathlib import Path
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
import numpy as np
import rclpy
from pepper_runtime import PepperRuntime
from synced_capture_selection import select_stamps
root=Path('/__HANDYMAN_PACKAGE__/runtime/pepper-raw-stream-timing-20261024-02');root.mkdir(exist_ok=False)
rclpy.init();runtime=PepperRuntime(root,'pepper-raw-readonly',lambda _:None)
rows=[];started=time.monotonic();counts={}
try:
    while time.monotonic()-started<8:
        runtime.pump(.01)
        if len(runtime.received)!=7:continue
        now=runtime.node.get_clock().now().nanoseconds
        ages={k:(now-runtime.stamp(runtime.received[k]))/1e9 for k in ('rgb','depth','hand','joints')}
        rows.append(dict(now_ns=now,ages_s=ages,joint_stamp_ns=runtime.stamp(runtime.received['joints'])))
        for reserve in (100_000_000,150_000_000):
            for joint in (True,False):
                key=str(reserve)+'_with_joints_'+str(joint)
                selected=select_stamps(runtime.history,now,ready=lambda _:True,
                    joint_stamp_ns=runtime.stamp(runtime.received['joints']) if joint else None,
                    processing_reserve_ns=reserve,joint_processing_reserve_ns=0)
                counts[key]=counts.get(key,0)+(selected is not None)
    result=dict(robot_commands_sent=False,samples=len(rows),unique_joint_stamps=len(set(r['joint_stamp_ns'] for r in rows)),
        age_ranges_s={k:dict(min=float(np.min([r['ages_s'][k] for r in rows])),
            median=float(np.median([r['ages_s'][k] for r in rows])),max=float(np.max([r['ages_s'][k] for r in rows]))) for k in ('rgb','depth','hand','joints')},
        selection_counts=counts)
    (root/'rows.json').write_text(json.dumps(rows,indent=2));(root/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
finally:runtime.shutdown();rclpy.shutdown()
