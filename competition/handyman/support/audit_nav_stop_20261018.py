"""Read-only post-stop holding audit and actual navigation feedback sampling."""
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, '/__HANDYMAN_PACKAGE__/runtime')
from carry_place_reviewed_grasp_20261018 import (
    code, held_audit, palm_rotation, palm_in_base, read, LiveBackend, map_pose)
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener

base = Path('/__HANDYMAN_PACKAGE__/runtime')
out = base/'apple-nav-stop-audit-20261018-01'
proof = read(base/'apple-gate-pruning-live-carry-place-20261018-01/source-grasp-recomputed.json')
last = proof['rows'][-1]
s = read(Path(last['capture'])/'snapshot.json')
center = np.asarray(last['center_measurement']['center_m'])
offset = palm_rotation(s).T @ (center-palm_in_base(s))
backend = LiveBackend(read(code/'config/apple_transfer_auto_v1.json'))
trial = read(base/'apple-gate-pruning-live-close-lift-20261018-01/result.json')
backend.holding = trial['close']['closure_evidence']['observed_opening_rad']
frames = sorted(out.glob('*/snapshot.json'))
result = dict(robot_commands_sent=False, recording=str(out))
try:
    result['holding_audit'] = held_audit(backend,frames[-1].parent,offset,
        last['center_measurement']['radius_m'],[0.,0.,0.,-math.pi/2,0.])
except Exception as exc:
    result['holding_audit'] = dict(held_object_verified=False,reason=str(exc))
result['last_capture_map_pose'] = map_pose(read(frames[-1]))
result['last_capture_joints'] = read(frames[-1])['joints']
rclpy.init()
n = rclpy.create_node('read_only_post_nav_stop_feedback_audit')
b = Buffer(); listener = TransformListener(b,n); data = {}; rows = []
subs = [n.create_subscription(JointState,'/hsrb/joint_states',lambda m:data.update(joints=m),qos_profile_sensor_data),
        n.create_subscription(LaserScan,'/hsrb/base_scan',lambda m:data.update(scan=m),qos_profile_sensor_data)]
start = time.monotonic(); next_sample = start+3
try:
    while time.monotonic()-start < 15:
        rclpy.spin_once(n,timeout_sec=.02)
        for _ in range(20): rclpy.spin_once(n,timeout_sec=0)
        if time.monotonic()<next_sample: continue
        next_sample=time.monotonic()+.1
        now=n.get_clock().now()
        row=dict(now_ns=now.nanoseconds,ages_s={key:(now-Time.from_msg(m.header.stamp)).nanoseconds/1e9 for key,m in data.items()})
        try:
            tf=b.lookup_transform('map','base_footprint',Time())
            row['ages_s']['tf']=(now-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
            t=tf.transform.translation; q=tf.transform.rotation
            row['map_pose']=[t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        except Exception as exc: row['tf_error']=str(exc)
        rows.append(row)
finally:
    n.destroy_node();rclpy.shutdown()
result['feedback_rows']=rows
result['feedback_age_max_s']={key:max(r['ages_s'].get(key,float('inf')) for r in rows) for key in ('joints','scan','tf')}
result['samples_over_0_7s']={key:sum(r['ages_s'].get(key,float('inf'))>.7 for r in rows) for key in ('joints','scan','tf')}
result['last_sample']=rows[-1]
with (out/'post-stop-audit.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in result.items() if k not in ('holding_audit','feedback_rows')},allow_nan=False))
print(json.dumps(dict(holding_audit={k:v for k,v in result['holding_audit'].items() if k!='rows'})))
