"""Fresh initial laser/odom evidence before any map anchor exists; read-only."""
from collections import deque
import json
from pathlib import Path
import time
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener

output=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/apple-r16b-live-initial-20261022-01/unanchored-scans.json')
if output.exists():raise ValueError('new_scan_file_required')
rclpy.init();node=rclpy.create_node('fresh_initial_unanchored_scan');buf=Buffer();listener=TransformListener(buf,node)
scans=deque(maxlen=40);sub=node.create_subscription(LaserScan,'/hsrb/base_scan',scans.append,qos_profile_sensor_data)
rows=[];seen=set();deadline=time.monotonic()+12;next_sample=time.monotonic()+3
try:
    while time.monotonic()<deadline and len(rows)<3:
        rclpy.spin_once(node,timeout_sec=.02)
        for _ in range(20):rclpy.spin_once(node,timeout_sec=0)
        if time.monotonic()<next_sample:continue
        now=node.get_clock().now().nanoseconds
        for scan in reversed(scans):
            when=Time.from_msg(scan.header.stamp);stamp=when.nanoseconds
            frames=[('base_from_scan','base_footprint',scan.header.frame_id),('odom_from_base','odom','base_footprint')]
            if stamp in seen or not 0<=(now-stamp)/1e9<=.4 or not all(buf.can_transform(t,s,when) for _,t,s in frames):continue
            row=dict(angle_min=scan.angle_min,angle_increment=scan.angle_increment,range_min=scan.range_min,
                range_max=scan.range_max,ranges=[v if v==v and abs(v)!=float('inf') else None for v in scan.ranges],
                stamp_ns=stamp,age_at_capture_s=(now-stamp)/1e9)
            for key,target,source in frames:
                tf=buf.lookup_transform(target,source,when).transform;q=tf.rotation
                row[key]=dict(translation=[tf.translation.x,tf.translation.y,tf.translation.z],
                    quaternion=[q.x,q.y,q.z,q.w],stamp_ns=stamp)
            rows.append(row);seen.add(stamp);next_sample=time.monotonic()+.3;break
    if len(rows)!=3:raise ValueError('fresh_laser_and_odometry_required')
    result=dict(scans=rows,map_anchor_required=False,map_tf_published=False,robot_commands_sent=False,original_stamps_preserved=True)
    with output.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(samples=len(rows),output=str(output),robot_commands_sent=False)),flush=True)
finally:node.destroy_node();rclpy.shutdown()
