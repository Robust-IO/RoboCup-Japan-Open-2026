import rclpy,time,json
rclpy.init();n=rclpy.create_node('session4_readonly_graph_probe');end=time.monotonic()+6
from handyman_msgs.msg import HandymanMsg
n.create_subscription(HandymanMsg,'/handyman/message/to_robot',lambda m:print('MODERATOR',m.message,m.detail,flush=True),10)
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
for t in ['/hsrb/joint_states','/hsrb/head_rgbd_sensor/rgb/image_raw','/hsrb/gripper_controller/command','/hsrb/arm_trajectory_controller/command','/hsrb/omni_base_controller/command','/handyman/message/to_robot','/handyman/message/to_moderator']:
 print(json.dumps(dict(topic=t,publishers=n.count_publishers(t),subscribers=n.count_subscribers(t))))
for t in ['/hsrb/gripper_controller/command','/hsrb/arm_trajectory_controller/command']:
 for x in n.get_subscriptions_info_by_topic(t):print(t,str(x))
n.destroy_node();rclpy.shutdown()
