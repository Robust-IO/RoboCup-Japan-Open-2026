"""Read-only receipt/capture timing diagnostic, no motion publisher."""
import json
import time
import rclpy
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data

rclpy.init();node=rclpy.create_node('joint_stamp_age_probe');rows=[]
def receive(m):
    stamp=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
    rows.append(dict(stamp_ns=stamp,age_ms=(node.get_clock().now().nanoseconds-stamp)/1e6))
sub=node.create_subscription(JointState,'/hsrb/joint_states',receive,qos_profile_sensor_data)
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
ages=sorted(r['age_ms'] for r in rows)
print(json.dumps(dict(count=len(rows),min_ms=min(ages) if ages else None,max_ms=max(ages) if ages else None,
    median_ms=ages[len(ages)//2] if ages else None,future_over_20ms=sum(a < -20 for a in ages),
    past_over_300ms=sum(a > 300 for a in ages),nonincreasing=sum(b['stamp_ns']<=a['stamp_ns'] for a,b in zip(rows,rows[1:])),rows=rows)))
node.destroy_node();rclpy.shutdown()
