import json,time,math
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from rcl_interfaces.srv import GetParameters
rclpy.init();node=rclpy.create_node('speed_trial_readonly');buffer=Buffer();listener=TransformListener(buffer,node)
try:
 start=time.monotonic()
 while time.monotonic()-start<2:rclpy.spin_once(node,timeout_sec=.05)
 tf=buffer.lookup_transform('map','base_footprint',Time()).transform;t=tf.translation;q=tf.rotation
 service=node.create_client(GetParameters,'/controller_server/get_parameters');service.wait_for_service(timeout_sec=3)
 req=GetParameters.Request();req.names=['FollowPath.max_vel_x','FollowPath.max_speed_xy','FollowPath.max_vel_theta']
 future=service.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=3)
 print(json.dumps(dict(robot_commands_sent=False,now_ns=time.time_ns(),map_pose=[t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],speed_caps=dict(zip(req.names,[v.double_value for v in future.result().values])))))
finally:node.destroy_node();rclpy.shutdown()
