import rclpy,time,json,math
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
rclpy.init();n=rclpy.create_node('readonly_stopped_state_listener');b=Buffer();l=TransformListener(b,n);rows=[];poses=[]
def cb(m):rows.append((n.get_clock().now().nanoseconds,m.header.stamp.sec*10**9+m.header.stamp.nanosec,dict(zip(m.name,m.position))))
n.create_subscription(JointState,'/hsrb/joint_states',cb,qos_profile_sensor_data)
end=time.monotonic()+5
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.05)
 try:
  tf=b.lookup_transform('map','base_footprint',Time());p=tf.transform.translation;q=tf.transform.rotation
  poses.append([p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
 except Exception:pass
out=dict(joint_frames=len(rows),poses=len(poses),robot_commands_sent=False)
if rows:
 keys=['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint']
 out.update(joint_age_s=(rows[-1][0]-rows[-1][1])/1e9,last_joints=rows[-1][2],joint_range={k:max(x[2][k] for x in rows)-min(x[2][k] for x in rows) for k in keys})
if poses:out.update(last_pose=poses[-1],pose_range=[max(p[i] for p in poses)-min(p[i] for p in poses) for i in range(3)])
print(json.dumps(out));n.destroy_node();rclpy.shutdown()
