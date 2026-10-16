"""Bounded RGBD/TF diagnostics. No control publishers, no unit guessing."""
import argparse
from collections import deque
import json
from pathlib import Path
import time
from capture_rgb_readonly import encode_png

def stamp(msg):
    return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

def image_meta(msg):
    return dict(width=msg.width, height=msg.height, encoding=msg.encoding,
                step=msg.step, is_bigendian=msg.is_bigendian,
                frame_id=msg.header.frame_id, stamp_ns=stamp(msg))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--seconds',type=int,default=30)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import Image, CameraInfo
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node=Node('handyman_rgbd_readonly_probe')
    buffer=Buffer()
    listener=TransformListener(buffer,node)
    rgb=deque(maxlen=5)
    depth=deque(maxlen=5)
    info={}
    counts={'rgb':0,'depth':0}
    def receive(kind,msg):
        counts[kind]+=1
        (rgb if kind=='rgb' else depth).append(msg)
    def camera(kind,msg):
        info[kind]=dict(width=msg.width,height=msg.height,frame_id=msg.header.frame_id,
                        stamp_ns=stamp(msg),k=list(msg.k),p=list(msg.p),d=list(msg.d),distortion_model=msg.distortion_model)
    subscriptions=[]
    for kind,prefix in [('rgb','rgb'),('depth','depth_registered')]:
        topic='/hsrb/head_rgbd_sensor/'+prefix
        subscriptions.append(node.create_subscription(Image,topic+'/image_raw',lambda m,k=kind:receive(k,m),qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(CameraInfo,topic+'/camera_info',lambda m,k=kind:camera(k,m),qos_profile_sensor_data))
    print('READY RGBD probe, no robot commands',flush=True)
    deadline=time.monotonic()+args.seconds
    pair=None
    try:
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if rgb and depth and 'rgb' in info and 'depth' in info:
                a,b=min(((a,b) for a in rgb for b in depth),key=lambda pair:abs(stamp(pair[0])-stamp(pair[1])))
                if stamp(a)>0 and stamp(b)>0 and abs(stamp(a)-stamp(b))<=100000000:
                    pair=(a,b)
                    break
        report={'counts':counts,'camera_info':info,'paired':pair is not None,'max_time_delta_ms':100,
                'alignment_verified':False,'note':'Matching dimensions/intrinsics alone do not verify image registration or optical-axis convention.'}
        if pair:
            a,b=pair
            report.update(rgb=image_meta(a),depth=image_meta(b),time_delta_ms=abs(stamp(a)-stamp(b))/1e6)
            (args.output/'rgb.png').write_bytes(encode_png(a))
            (args.output/'depth.bin').write_bytes(bytes(b.data))
            report['tf_at_depth_stamp']={}
            for target in ['base_link','odom']:
                try:
                    tf=buffer.lookup_transform(target,b.header.frame_id,Time.from_msg(b.header.stamp))
                    t,q=tf.transform.translation,tf.transform.rotation
                    report['tf_at_depth_stamp'][target]={'translation':[t.x,t.y,t.z],'rotation_xyzw':[q.x,q.y,q.z,q.w]}
                except Exception as exc:
                    report['tf_at_depth_stamp'][target]={'error':str(exc)}
        (args.output/'probe.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
