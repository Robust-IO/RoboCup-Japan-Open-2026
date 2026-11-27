"""Passive joint/laser timing: no control publishers or parameter writes."""
import json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState,LaserScan
b=Path('/__HANDYMAN_PACKAGE__/runtime');out=b/('joint-latency-'+time.strftime('%Y%m%d-%H%M%S'));out.mkdir()
rclpy.init();node=rclpy.create_node('handyman_passive_latency_probe');rows={'joints':[],'scan':[]};gaps=[]
def receive(key,m):
    rows[key].append(dict(received_mono=time.monotonic(),stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
        age_s=(node.get_clock().now().nanoseconds-m.header.stamp.sec*10**9-m.header.stamp.nanosec)/1e9))
subscriptions=[node.create_subscription(kind,topic,lambda m,key=key:receive(key,m),qos_profile_sensor_data) for key,topic,kind in (
    ('joints','/hsrb/joint_states',JointState),('scan','/hsrb/base_scan',LaserScan))]
start=time.monotonic();previous=start
while time.monotonic()-start<12:
    rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic();gaps.append(now-previous);previous=now
def stats(values):
    return dict(median=float(np.median(values)),p95=float(np.quantile(values,.95)),maximum=float(max(values))) if values else None
report=dict(robot_commands_sent=False,robot_moving_commanded=False,duration_s=time.monotonic()-start,streams={})
for key,topic in (('joints','/hsrb/joint_states'),('scan','/hsrb/base_scan')):
    data=rows[key]
    report['streams'][key]=dict(messages=len(data),timestamp_age_s=stats([v['age_s'] for v in data]),
        arrival_gap_s=stats([c['received_mono']-a['received_mono'] for a,c in zip(data,data[1:])]),
        source_stamp_gap_s=stats([(c['stamp_ns']-a['stamp_ns'])/1e9 for a,c in zip(data,data[1:])]),
        publisher_nodes=[e.node_name for e in node.get_publishers_info_by_topic(topic)])
report['probe_executor_gap_s']=stats(gaps)
report['limitation']='Current stationary observation, not a reproduction of the earlier moving episode; timestamp age includes any clock offset.'
(out/'samples.json').write_text(json.dumps(rows));(out/'result.json').write_text(json.dumps(report,indent=2));(b/'joint-latency-probe-latest.json').write_text(json.dumps(dict(output=str(out),report=report),indent=2))
node.destroy_node();rclpy.shutdown();print(json.dumps(report))
