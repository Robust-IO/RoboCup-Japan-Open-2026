"""Receive-only image health check; no publishers, services or robot actions."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

output=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/dual-camera-check-20261022-01.json')
if output.exists():raise ValueError('new_output_required')
topics={'head':'/hsrb/head_rgbd_sensor/rgb/image_raw','hand':'/hsrb/hand_camera/image_raw'}
rclpy.init();node=rclpy.create_node('read_only_dual_camera_check_20261022');images={}
def receive(key,msg):
    item=images.setdefault(key,dict(count=0));item.update(count=item['count']+1,width=msg.width,height=msg.height,
        encoding=msg.encoding,stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
        received_ns=node.get_clock().now().nanoseconds,frame=msg.header.frame_id)
subs=[node.create_subscription(Image,topic,lambda msg,key=key:receive(key,msg),qos_profile_sensor_data) for key,topic in topics.items()]
try:
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    now=node.get_clock().now().nanoseconds
    result=dict(images=images,publishers={key:node.count_publishers(topic) for key,topic in topics.items()},
        latest_image_age_s={key:(now-item['stamp_ns'])/1e9 for key,item in images.items()},
        motion_command_publishers={topic:node.count_publishers(topic) for topic in (
            '/hsrb/command_velocity','/hsrb/omni_base_controller/command','/hsrb/arm_trajectory_controller/command',
            '/hsrb/gripper_controller/command','/hsrb/head_trajectory_controller/command')},
        both_cameras_received=all(key in images for key in topics),robot_commands_sent=False)
    with output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result),flush=True)
finally:node.destroy_node();rclpy.shutdown()
