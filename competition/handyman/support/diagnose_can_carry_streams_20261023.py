"""Receive-only stream timing and TF diagnosis before any carry action."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener

rclpy.init();n=rclpy.create_node('diagnose_can_carry_readonly');b=Buffer();listener=TransformListener(b,n)
rows={};frames={};joints={}
def stamp(m):return m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
def receive(key,m):
    rows.setdefault(key,dict(count=0))['count']+=1
    rows[key].update(stamp_ns=stamp(m),frame=m.header.frame_id,receive_age_s=(n.get_clock().now().nanoseconds-stamp(m))/1e9)
    if key=='joints':joints.update(dict(zip(m.name,m.position)))
def tf_receive(msg):
    for t in msg.transforms:
        frames[t.header.frame_id+'->'+t.child_frame_id]=dict(stamp_ns=stamp(t),age_s=(n.get_clock().now().nanoseconds-stamp(t))/1e9)
subs=[n.create_subscription(kind,topic,lambda m,key=key:receive(key,m),qos_profile_sensor_data) for key,kind,topic in
    [('rgb',Image,'/hsrb/head_rgbd_sensor/rgb/image_raw'),('depth',Image,'/hsrb/head_rgbd_sensor/depth_registered/image_raw'),
     ('hand',Image,'/hsrb/hand_camera/image_raw'),('joints',JointState,'/hsrb/joint_states')]]
subs.append(n.create_subscription(TFMessage,'/tf',tf_receive,qos_profile_sensor_data))
end=time.monotonic()+4
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.01)
    for _ in range(30):rclpy.spin_once(n,timeout_sec=0.)
checks={}
for target,source in [('map','base_link'),('odom','base_link'),('base_link','head_rgbd_sensor_rgb_frame'),('base_link','hand_palm_link')]:
    try:
        t=b.lookup_transform(target,source,Time());checks[target+'<-'+source]=dict(stamp_ns=stamp(t),age_s=(n.get_clock().now().nanoseconds-stamp(t))/1e9)
    except Exception as exc:checks[target+'<-'+source]=str(exc)
report=dict(streams=rows,tf_frames=frames,tf_checks=checks,joints=joints,now_ns=n.get_clock().now().nanoseconds,
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),robot_commands_sent=False)
out=Path('/__HANDYMAN_PACKAGE__/runtime/can-carry-stream-diagnosis-20261023-01.json')
with out.open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report),flush=True);n.destroy_node();rclpy.shutdown()
