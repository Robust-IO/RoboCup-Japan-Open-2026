import json,time
import rclpy
rclpy.init();n=rclpy.create_node('transfer_readonly_control_graph_audit')
try:
 end=time.monotonic()+4
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 def info(e):return dict(node=e.node_name,namespace=e.node_namespace,topic_type=e.topic_type,endpoint_gid=list(e.endpoint_gid))
 topics=['/hsrb/'+tail for tail in ('command_velocity','omni_base_controller/command','arm_trajectory_controller/command','gripper_controller/command','head_trajectory_controller/command')]
 print(json.dumps({t:dict(publishers=[info(e) for e in n.get_publishers_info_by_topic(t)],subscribers=[info(e) for e in n.get_subscriptions_info_by_topic(t)]) for t in topics}),flush=True)
finally:n.destroy_node();rclpy.shutdown()
