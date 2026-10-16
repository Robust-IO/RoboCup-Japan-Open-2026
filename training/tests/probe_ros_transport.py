"""Bounded synthetic String transport probe; domain 73 only."""
import os
import sys
import time
import subprocess

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='73'
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(); child=None
    receiving=len(sys.argv)>1
    node=Node('transport_receiver' if receiving else 'transport_sender')
    count=[0]
    def receive(msg):
        count[0]+=1
    sub=node.create_subscription(String,'/handyman_test/transport',receive,qos_profile_sensor_data)
    pub=None
    try:
        if not receiving:
            pub=node.create_publisher(String,'/handyman_test/transport',qos_profile_sensor_data)
            child=subprocess.Popen([sys.executable,__file__,'receive'])
        start=time.monotonic(); sent=0
        while time.monotonic()-start<6:
            if pub is not None:
                msg=String();msg.data=str(sent);pub.publish(msg);sent+=1
            rclpy.spin_once(node,timeout_sec=.05)
            time.sleep(.02)
        print(dict(role='receiver' if receiving else 'sender',received=count[0],sent=sent,
                   matched=pub.get_subscription_count() if pub else None),flush=True)
        if child is not None: child.wait(timeout=10)
    finally:
        if child is not None and child.poll() is None:
            child.terminate();child.wait(timeout=5)
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__': main()
