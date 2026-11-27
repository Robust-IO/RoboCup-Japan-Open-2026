"""Real BSON/ROS test against candidate bridge, isolated from Unity/domain71."""
import os,time,socket,subprocess,json,sys
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='84':raise RuntimeError('test_domain84_required')
import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import bson
binary=Path(sys.argv[1]);log=Path(sys.argv[2]);stream=log.open('w')
child=subprocess.Popen([str(binary),'55091','65535'],stdout=stream,stderr=subprocess.STDOUT)
node=None;sock=None
try:
 rclpy.init();node=rclpy.create_node('isolated_native_clock_test')
 pub=node.create_publisher(JointState,'/hsrb/joint_states',10)
 client=node.create_client(Trigger,'/sigverse/recalibrate_startup_clock')
 start=time.monotonic()
 while True:
  try:sock=socket.create_connection(('127.0.0.1',55091),timeout=.3);break
  except OSError:
   if time.monotonic()-start>10:raise
   time.sleep(.1)
 source=time.time()-1.;packet=dict(op='publish',topic='/sigverse/TimeSync',type='sigverse/TimeSync',msg=dict(sec=int(source),nanosec=int(source%1*1e9)))
 payload=bson.BSON.encode(packet) if hasattr(bson,'BSON') else bson.dumps(packet)
 sock.sendall(payload);sock.settimeout(5);initial=sock.recv(256).decode().split(',')
 assert initial[0]=='time_gap' and len(initial)==3,initial
 initial_ms=int(initial[1])*1000+int(initial[2]);correction=[0]
 def publish():
  stamp=int(time.time()*1e9)+160000000-correction[0]*1000000
  m=JointState();m.header.stamp.sec=stamp//1000000000;m.header.stamp.nanosec=stamp%1000000000;m.name=['test_joint'];m.position=[0.];pub.publish(m)
 timer=node.create_timer(.1,publish)
 def spin(seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
 def call():
  assert client.wait_for_service(timeout_sec=5)
  f=client.call_async(Trigger.Request());end=time.monotonic()+5
  while not f.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
  assert f.done(),'service_timeout'
  return f.result()
 spin(3.5);first=call();assert first.success and first.message.startswith('source_clock_adjustment_sent'),first.message
 update=sock.recv(256).decode().split(',');assert update[0]=='time_gap' and len(update)==3,update
 new_ms=int(update[1])*1000+int(update[2]);correction[0]=new_ms-initial_ms
 assert 150<=correction[0]<=200,correction[0]
 spin(3.7);second=call();assert second.success and second.message=='clock_ready',second.message
 print(json.dumps(dict(status='passed',domain=84,correction_ms=correction[0],verified_native_protocol=True,unity_or_robot_commands_sent=False)))
finally:
 if sock:sock.close()
 if node:node.destroy_node();rclpy.shutdown()
 child.send_signal(__import__('signal').SIGINT)
 try:child.wait(timeout=5)
 except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
 stream.close()
