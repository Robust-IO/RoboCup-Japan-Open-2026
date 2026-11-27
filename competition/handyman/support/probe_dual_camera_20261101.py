"""Read camera arrivals only; does not create control publishers."""
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node('dual_camera_readonly_20261101')
seen = {}
def receive(key, message):
    row = seen.setdefault(key, dict(count=0))
    row.update(count=row['count'] + 1, width=message.width, height=message.height,
               stamp_ns=message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
               encoding=message.encoding)
subs = [node.create_subscription(Image, topic, lambda msg, key=key: receive(key, msg), qos_profile_sensor_data)
        for key, topic in [('head', '/hsrb/head_rgbd_sensor/rgb/image_raw'),
                           ('hand', '/hsrb/hand_camera/image_raw'),
                           ('depth', '/hsrb/head_rgbd_sensor/depth_registered/image_raw')]]
try:
    start = time.monotonic()
    while time.monotonic() - start < 5:
        rclpy.spin_once(node, timeout_sec=.05)
    now = node.get_clock().now().nanoseconds
    for row in seen.values():
        row['age_s'] = (now - row['stamp_ns']) / 10**9
    print(json.dumps(dict(robot_commands_sent=False, streams=seen)))
finally:
    node.destroy_node()
    rclpy.shutdown()
