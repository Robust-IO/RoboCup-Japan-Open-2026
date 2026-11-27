import rclpy,time,json
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('tf_jitter_readonly');b=Buffer();l=TransformListener(b,n);latest=[];counts={};errors={}
s=n.create_subscription(LaserScan,'/hsrb/base_scan',lambda m:latest.__setitem__(slice(None),[m]),qos_profile_sensor_data)
end=time.monotonic()+8
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.03)
 if not latest:continue
 for label,frame,stamp in [('pose','base_footprint',Time()),('scan',latest[0].header.frame_id,Time.from_msg(latest[0].header.stamp))]:
  try:b.lookup_transform('map',frame,stamp);key=label+'_ok'
  except Exception as e:key=label+'_'+type(e).__name__;errors[key]=str(e)
  counts[key]=counts.get(key,0)+1
print(json.dumps(dict(counts=counts,errors=errors)));n.destroy_node();rclpy.shutdown()
