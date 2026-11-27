"""Publish ONE new independently measured current-scene map anchor, no motion."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer,TransformListener,StaticTransformBroadcaster

p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--match',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or a.output.exists():
    raise ValueError('explicit_new_current_simulation_alignment_required')
match=json.loads(a.match.read_text());rows=match['scans'];best=match['best'];x,y,yaw=best['pose']
if len(rows)!=3 or any(r['new_score']>.02 or r['new_inlier_fraction']<.9 or
    math.dist(r['new_pose'][:2],best['pose'][:2])>.02 or abs(r['new_pose'][2]-yaw)>.01 for r in rows):
    raise ValueError('three_independent_laser_matches_required')
rclpy.init();node=rclpy.create_node('r12b_reviewed_current_map_anchor');buffer=Buffer();listener=TransformListener(buffer,node)
publisher=StaticTransformBroadcaster(node)
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
tf=buffer.lookup_transform('odom','base_footprint',Time());t=tf.transform.translation;q=tf.transform.rotation
old=match['odom_from_base'];oq=old['quaternion'];old_yaw=math.atan2(2*(oq[3]*oq[2]+oq[0]*oq[1]),1-2*(oq[1]**2+oq[2]**2))
actual_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
if (not -.02<=(node.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9<=.4
        or math.dist([t.x,t.y],old['translation'][:2])>.01
        or abs(math.atan2(math.sin(actual_yaw-old_yaw),math.cos(actual_yaw-old_yaw)))>.01):
    raise ValueError('robot_moved_or_feedback_stale_since_scan')
angle=yaw-old_yaw;cx,sy=math.cos(angle),math.sin(angle);ox,oy=old['translation'][:2]
msg=TransformStamped();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='map';msg.child_frame_id='odom'
msg.transform.translation.x=x-cx*ox+sy*oy;msg.transform.translation.y=y-sy*ox-cx*oy;msg.transform.translation.z=-old['translation'][2]
msg.transform.rotation.z=math.sin(angle/2);msg.transform.rotation.w=math.cos(angle/2)
publisher.sendTransform(msg)
record=dict(status='reviewed_map_anchor_published',reviewed_base_pose=best['pose'],
    map_from_odom=dict(translation=[msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z],
                      quaternion=[0,0,msg.transform.rotation.z,msg.transform.rotation.w]),
    match=str(a.match),map_resolution_m=.05,estimated_not_ground_truth=True,robot_commands_sent=False)
with a.output.open('x') as stream:json.dump(record,stream,indent=2)
print(json.dumps(record),flush=True)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:node.destroy_node();rclpy.try_shutdown()
