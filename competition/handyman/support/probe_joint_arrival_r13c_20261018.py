"""Independent JointState receipt latency, no publishers or robot commands."""
import json
from pathlib import Path
import statistics
import time
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
rclpy.init();node=rclpy.create_node('joint_arrival_r13c_readonly');rows=[]
def receive(message):
    now=node.get_clock().now().nanoseconds;stamp=Time.from_msg(message.header.stamp).nanoseconds
    rows.append(dict(now_ns=now,original_stamp_ns=stamp,age_s=(now-stamp)/1e9))
sub=node.create_subscription(JointState,'/hsrb/joint_states',receive,qos_profile_sensor_data)
end=time.monotonic()+8
try:
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
finally:node.destroy_node();rclpy.shutdown()
report=dict(messages=len(rows),minimum_age_s=min(r['age_s'] for r in rows),median_age_s=statistics.median(r['age_s'] for r in rows),
    maximum_age_s=max(r['age_s'] for r in rows),robot_commands_sent=False,rows=rows)
with Path('/__HANDYMAN_PACKAGE__/runtime/apple-r13c-joint-arrival-20261018.json').open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
