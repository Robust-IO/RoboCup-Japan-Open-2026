import rclpy,time,json,statistics
from sensor_msgs.msg import JointState,Image
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('startup_clock_readonly');rows={};messages={}
def receive(m,key):
 stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
 rows.setdefault(key,[]).append((time.monotonic(),stamp,(n.get_clock().now().nanoseconds-stamp)/1e9))
 messages[key]=m
subs=[]
for key,kind,topic in [('joints',JointState,'/hsrb/joint_states'),('rgb',Image,'/hsrb/head_rgbd_sensor/rgb/image_raw'),('depth',Image,'/hsrb/head_rgbd_sensor/depth_registered/image_raw')]:
 subs.append(n.create_subscription(kind,topic,lambda m,k=key:receive(m,k),qos_profile_sensor_data))
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
for k,a in rows.items():
 ages=[x[2] for x in a];print(json.dumps(dict(stream=k,count=len(a),age_min=min(ages),age_median=statistics.median(ages),age_max=max(ages),last_age=(n.get_clock().now().nanoseconds-a[-1][1])/1e9,stamp_backwards=sum(b[1]<c[1] for c,b in zip(a,a[1:])),duration=a[-1][0]-a[0][0])))
for t in ['/hsrb/joint_states','/hsrb/head_rgbd_sensor/rgb/image_raw','/hsrb/head_rgbd_sensor/depth_registered/image_raw','/hsrb/command_velocity']:
 print(json.dumps(dict(topic=t,publishers=[dict(node=x.node_name,namespace=x.node_namespace) for x in n.get_publishers_info_by_topic(t)])))
if 'joints' in messages:print('JOINTS',dict(zip(messages['joints'].name,messages['joints'].position)))
n.destroy_node();rclpy.shutdown()
