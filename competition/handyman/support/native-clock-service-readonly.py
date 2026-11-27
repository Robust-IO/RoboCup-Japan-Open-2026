import rclpy,json
from std_srvs.srv import Trigger
rclpy.init();n=rclpy.create_node('clock_service_readonly_check');c=n.create_client(Trigger,'/sigverse/recalibrate_startup_clock');print(json.dumps(dict(service_available=c.wait_for_service(timeout_sec=8),request_sent=False,robot_commands_sent=False)));n.destroy_node();rclpy.shutdown()
