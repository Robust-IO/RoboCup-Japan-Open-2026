import rclpy,time,math,json
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
rclpy.init();n=rclpy.create_node('session4_pose_readonly');b=Buffer();l=TransformListener(b,n);end=time.monotonic()+2
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
for frame in ['map','odom']:
 t=b.lookup_transform(frame,'base_footprint',Time());q=t.transform.rotation;p=t.transform.translation
 print(json.dumps(dict(frame=frame,pose=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])))
n.destroy_node();rclpy.shutdown()
