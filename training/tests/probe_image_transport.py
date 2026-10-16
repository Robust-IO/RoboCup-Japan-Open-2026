"""Synthetic Image transport comparison; isolated domain 73, no Unity."""
import argparse
from array import array
from collections import Counter
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--receiver',type=Path)
    a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='73':raise RuntimeError('domain 73 required')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init();node=Node('image_transport_receiver' if a.receiver else 'image_transport_sender')
    counts={'best_effort':Counter(),'reliable':Counter()};sizes={};subs=[];child=None
    def receive(msg,kind):
        counts[kind][msg.header.frame_id]+=1
        sizes[msg.header.frame_id]=len(msg.data)
    try:
        if a.receiver:
            for kind,qos in [('best_effort',qos_profile_sensor_data),('reliable',10)]:
                subs.append(node.create_subscription(Image,'/handyman_test/image',lambda m,k=kind:receive(m,k),qos))
            deadline=time.monotonic()+30
            while time.monotonic()<deadline and not a.receiver.with_suffix('.stop').exists():
                rclpy.spin_once(node,timeout_sec=.02)
            a.receiver.write_text(json.dumps(dict(counts={k:dict(v) for k,v in counts.items()},sizes=sizes),indent=2))
        else:
            out=Path(tempfile.mkdtemp(prefix='image-transport-'));print('OUTPUT',out,flush=True)
            pub=node.create_publisher(Image,'/handyman_test/image',10)
            with (out/'receiver.log').open('w') as log:
                child=subprocess.Popen([sys.executable,__file__,'--receiver',str(out/'received.json')],stdout=log,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+10
                while pub.get_subscription_count()<2 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
                if pub.get_subscription_count()<2:raise RuntimeError('subscriptions not matched')
                print('Both subscribers matched before publishing',flush=True)
                sent=Counter()
                for name,w,h,bpp,encoding in [('small',64,64,3,'rgb8'),('rgb',640,480,3,'rgb8'),('depth',640,480,2,'16UC1')]:
                    msg=Image();msg.header.frame_id=name;msg.width=w;msg.height=h;msg.step=w*bpp;msg.encoding=encoding
                    msg.data=array('B',[17])*(w*h*bpp)
                    for _ in range(25):
                        msg.header.stamp=node.get_clock().now().to_msg();pub.publish(msg);sent[name]+=1
                        rclpy.spin_once(node,timeout_sec=.01);time.sleep(.09)
                time.sleep(2)
                (out/'received.stop').touch()
                child.wait(timeout=10)
            if child.returncode:raise RuntimeError('receiver failed: '+str(out))
            report=dict(sent=dict(sent),received=json.loads((out/'received.json').read_text()),
                        rmw=os.environ.get('RMW_IMPLEMENTATION'),synthetic=True)
            (out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
