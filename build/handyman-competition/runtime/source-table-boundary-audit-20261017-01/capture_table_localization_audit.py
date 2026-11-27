"""Bounded read-only laser/TF samples; no publishers or transform writes."""
import argparse
from collections import deque
import json
from pathlib import Path
import time


def main():
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise ValueError('output_exists')
    rclpy.init();node=rclpy.create_node('source_table_localization_readonly')
    buf=Buffer();listener=TransformListener(buf,node);scans=deque(maxlen=40)
    sub=node.create_subscription(LaserScan,'/hsrb/base_scan',scans.append,qos_profile_sensor_data)
    rows=[];seen=set();end=time.monotonic()+12;next_sample=time.monotonic()+4
    try:
        while time.monotonic()<end and len(rows)<3:
            rclpy.spin_once(node,timeout_sec=.02)
            for _ in range(20):rclpy.spin_once(node,timeout_sec=0)
            if time.monotonic()<next_sample:continue
            now=node.get_clock().now().nanoseconds
            for scan in reversed(scans):
                when=Time.from_msg(scan.header.stamp);stamp=when.nanoseconds
                if stamp in seen or not 0<=(now-stamp)/1e9<=.4:continue
                frames=[('base_from_scan','base_footprint',scan.header.frame_id),
                        ('odom_from_base','odom','base_footprint'),
                        ('map_from_base','map','base_footprint')]
                if not all(buf.can_transform(target,source,when) for _,target,source in frames):continue
                row=dict(angle_min=scan.angle_min,angle_increment=scan.angle_increment,
                         range_min=scan.range_min,range_max=scan.range_max,
                         ranges=[v if v==v and abs(v)!=float('inf') else None for v in scan.ranges],
                         stamp_ns=stamp,age_at_capture_s=(now-stamp)/1e9)
                for key,target,source in frames:
                    t=buf.lookup_transform(target,source,when).transform;q=t.rotation
                    row[key]=dict(translation=[t.translation.x,t.translation.y,t.translation.z],
                                  quaternion=[q.x,q.y,q.z,q.w],stamp_ns=stamp)
                rows.append(row);seen.add(stamp);next_sample=time.monotonic()+.3
                print(json.dumps(dict(sample=len(rows),stamp_ns=stamp,age_s=row['age_at_capture_s'])),flush=True)
                break
        if len(rows)!=3:raise ValueError('three_fresh_scan_tf_pairs_required')
        with args.output.open('x') as f:
            json.dump(dict(scans=rows,robot_commands_sent=False,map_tf_published=False,
                           original_stamps_preserved=True),f,indent=2,allow_nan=False)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
