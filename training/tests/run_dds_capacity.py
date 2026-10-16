"""Domain 73 localhost only; 16 publisher processes plus one listener, no control."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    p=argparse.ArgumentParser();p.add_argument('--child',type=int);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('Requires isolated domain 73 / localhost')
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('dds_capacity_'+('listener' if a.child is None else str(a.child)))
    children=[]
    try:
        if a.child is not None:
            pub=node.create_publisher(String,'/handyman_test/dds_capacity',10)
            end=time.monotonic()+12
            while time.monotonic()<end:
                m=String();m.data=str(a.child);pub.publish(m);rclpy.spin_once(node,timeout_sec=.1)
        else:
            seen=set()
            sub=node.create_subscription(String,'/handyman_test/dds_capacity',lambda m:seen.add(int(m.data)),10)
            for i in range(16):children.append(subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--child',str(i)]))
            end=time.monotonic()+18
            while time.monotonic()<end and len(seen)<16:rclpy.spin_once(node,timeout_sec=.1)
            for child in children:child.wait(timeout=18)
            assert seen==set(range(16)),sorted(seen)
            assert all(child.returncode==0 for child in children)
            print(json.dumps(dict(passed=True,publisher_processes=16,listener_processes=1,received_ids=sorted(seen))))
    finally:
        for child in children:
            if child.poll() is None:
                child.terminate()
                try:child.wait(timeout=2)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
