"""Bounded metadata-only ROS probe. No model, images saved, or control publishers."""
import argparse
from collections import Counter, deque
import json
from pathlib import Path
import time


def stamp(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec


class StreamStats:
    def __init__(self):
        self.count=0;self.first=None;self.last=None;self.duplicates=0;self.backward=0
        self.min_age=None;self.max_age=None;self.latest=None

    def add(self, ns, now_ns, metadata):
        if self.last is not None:
            self.duplicates+=ns==self.last;self.backward+=ns<self.last
        if self.first is None:self.first=ns
        self.last=ns;self.count+=1;self.latest=metadata
        age=(now_ns-ns)/1e9
        self.min_age=age if self.min_age is None else min(self.min_age,age)
        self.max_age=age if self.max_age is None else max(self.max_age,age)


def closest(rgb_stamps, depth_ns):
    if not rgb_stamps:return None
    return min(rgb_stamps,key=lambda ns:abs(ns-depth_ns))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=8)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not 0<a.seconds<=30:p.error('seconds must be in (0,30]')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import Image,CameraInfo
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer,TransformListener,TransformException
    from rgbd_localization import camera_matches
    stream=a.output.open('x')
    rclpy.init();node=Node('handyman_stream_metadata_probe')
    buffer=Buffer();listener=TransformListener(buffer,node)
    topics={
        'rgb':'/hsrb/head_rgbd_sensor/rgb/image_raw',
        'depth':'/hsrb/head_rgbd_sensor/depth_registered/image_raw',
        'rgb_info':'/hsrb/head_rgbd_sensor/rgb/camera_info',
        'depth_info':'/hsrb/head_rgbd_sensor/depth_registered/camera_info',
        'odom':'/odom','hsrb_odom':'/hsrb/odom'}
    stats={k:StreamStats() for k in topics};latest={};subs=[]
    rgb=deque(maxlen=120);pending=deque(maxlen=120);pairs=Counter();tf_counts=Counter();tf_edges=Counter()
    deltas=[];tf_examples=[]

    def receive(msg,key):
        ns=stamp(msg);meta=dict(frame=msg.header.frame_id)
        if key in ('rgb','depth'):
            meta.update(width=msg.width,height=msg.height,encoding=msg.encoding,
                        step=msg.step,bigendian=msg.is_bigendian,data_bytes=len(msg.data))
        elif key.endswith('_info'):
            meta.update(width=msg.width,height=msg.height,k=list(msg.k),d=list(msg.d))
        else:meta.update(child_frame=msg.child_frame_id)
        stats[key].add(ns,node.get_clock().now().nanoseconds,meta)
        latest[key]=msg
        # Retain timestamps only; do not queue large image payloads.
        if key=='rgb':rgb.append(ns)
        if key=='depth':pending.append((ns,msg.header.frame_id,time.monotonic()))

    def receive_tf(msg,key):
        tf_counts[key]+=1
        for tf in msg.transforms:tf_edges[tf.header.frame_id+' -> '+tf.child_frame_id]+=1

    try:
        for key,topic in topics.items():
            typ=Image if key in ('rgb','depth') else CameraInfo if key.endswith('_info') else Odometry
            subs.append(node.create_subscription(typ,topic,lambda msg,k=key:receive(msg,k),qos_profile_sensor_data))
        subs.append(node.create_subscription(TFMessage,'/tf',lambda msg:receive_tf(msg,'dynamic'),qos_profile_sensor_data))
        subs.append(node.create_subscription(TFMessage,'/tf_static',lambda msg:receive_tf(msg,'static'),
            QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        end=time.monotonic()+a.seconds
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            # Allow 0.25 seconds for RGB/TF to arrive before auditing depth.
            while pending and time.monotonic()-pending[0][2]>=.25:
                ns,frame,_=pending.popleft();match=closest(rgb,ns)
                if match is None:pairs['no_rgb']+=1
                else:
                    delta=abs(ns-match)/1e6;deltas.append(delta)
                    pairs['within_50ms' if delta<=50 else 'outside_50ms']+=1
                try:
                    buffer.lookup_transform('odom',frame,Time(nanoseconds=ns))
                    pairs['exact_depth_tf_ready']+=1
                except TransformException as exc:
                    pairs['exact_depth_tf_missing']+=1
                    if len(tf_examples)<3:tf_examples.append(str(exc))
        calibration=None
        if all(k in latest for k in ('rgb','depth','rgb_info','depth_info')):
            calibration=camera_matches(*(latest[k] for k in ('rgb','depth','rgb_info','depth_info')))
        report=dict(metadata_only=True,actionable=False,seconds=a.seconds,
            streams={k:vars(v) for k,v in stats.items()},tf_messages=dict(tf_counts),tf_edges=dict(tf_edges),
            pair_counts=dict(pairs),unprocessed_depth_tail=len(pending),
            nearest_rgb_delta_ms_range=[min(deltas),max(deltas)] if deltas else None,
            latest_camera_profile_matches=calibration,tf_error_examples=tf_examples,
            publishers={k:[dict(node=info.node_name,type=info.topic_type,reliability=str(info.qos_profile.reliability),
                durability=str(info.qos_profile.durability)) for info in node.get_publishers_info_by_topic(topic)]
                for k,topic in topics.items()})
        json.dump(report,stream,indent=2,allow_nan=False)
        print(json.dumps(dict(output=str(a.output),counts={k:v.count for k,v in stats.items()},
            tf_messages=dict(tf_counts),pair_counts=dict(pairs),camera_profile=calibration)),flush=True)
    finally:
        node.destroy_node();rclpy.try_shutdown();stream.close()


if __name__=='__main__':main()
