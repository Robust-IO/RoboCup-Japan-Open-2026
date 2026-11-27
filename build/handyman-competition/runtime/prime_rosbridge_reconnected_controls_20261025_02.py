import json,sys,time,math
from pathlib import Path
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from pepper_runtime import PepperRuntime
from canned_grasp_runtime import NAMES,TOPICS
from task_outcome_policy import pose_matches
root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/rosbridge-pending-subscriber-prime-20261025-02');root.mkdir(exist_ok=False)
result=dict(robot_commands_sent=False,neutral_commands_sent=[],source_timestamps_preserved=True)
rclpy.init();runtime=PepperRuntime(root,'rosbridge-current-pose-prime',lambda e:None)
try:
 sample=runtime.sample();state=runtime.state();runtime.ownership();result['before']=state
 if not pose_matches(state['joints'],[0.,0.,0.,-math.pi/2,0.,0.]):raise ValueError('fresh_initial_empty_tucked_closed_hand_required')
 initial=state['poses']['odom'];j=sample['snapshot']['joints']
 def check_infos(topic,expected):
  infos=runtime.node.get_subscriptions_info_by_topic(topic)
  if len(infos)!=expected or any(i.node_name!='rosbridge_websocket' or i.node_namespace!='/' for i in infos):
   raise ValueError('unexpected_control_receivers:'+topic+':'+str(len(infos)))
  if len({bytes(i.endpoint_gid)[:12] for i in infos})!=1:raise ValueError('different_bridge_instances')
 def prime(topic,message,kind):
  check_infos(topic,2)
  pub=runtime.node.create_publisher(kind,topic,10)
  try:
   deadline=time.monotonic()+3
   while pub.get_subscription_count()!=2 and time.monotonic()<deadline:runtime.pump(.02)
   if pub.get_subscription_count()!=2:raise ValueError('pending_receiver_discovery_not_ready')
   for candidate in TOPICS:
    if runtime.node.count_publishers(candidate)!=(1 if candidate==topic else 0):raise ValueError('another_controller_present')
   check_infos(topic,2)
   pub.publish(message);result['robot_commands_sent']=True
   result['neutral_commands_sent'].append(topic)
   deadline=time.monotonic()+4
   while runtime.node.count_subscribers(topic)!=1 and time.monotonic()<deadline:runtime.pump(.02)
   check_infos(topic,1)
  finally:runtime.node.destroy_publisher(pub)
 # Head first: verify the documented first-message cleanup before other controls.
 prime(TOPICS[4],runtime.trajectory(['head_pan_joint','head_tilt_joint'],[j['head_pan_joint'],j['head_tilt_joint']],1.),JointTrajectory)
 result['head_first_message_cleanup_verified']=True
 prime(TOPICS[0],Twist(),Twist)
 prime(TOPICS[2],runtime.trajectory(NAMES[:5],state['joints'][:5],1.),JointTrajectory)
 prime(TOPICS[3],runtime.trajectory([NAMES[5]],[state['joints'][5]],1.),JointTrajectory)
 prime(TOPICS[1],runtime.trajectory(['odom_x','odom_y','odom_t'],initial,1.),JointTrajectory)
 until=time.monotonic()+1.2
 while time.monotonic()<until:runtime.pump(.02)
 after=runtime.state();result['after']=after
 if not pose_matches(after['joints'],state['joints']) or math.dist(initial[:2],after['poses']['odom'][:2])>.01:
  raise ValueError('unexpected_pose_change_during_neutral_handshake')
 runtime.ownership()
 for topic in TOPICS:check_infos(topic,1)
 result.update(status='all_five_reconnected_control_topics_have_one_receiver',base_displacement_m=math.dist(initial[:2],after['poses']['odom'][:2]),grasp_commands_sent=False)
except Exception as exc:result.update(status='stopped',reason=str(exc))
finally:
 runtime.shutdown();rclpy.shutdown();(root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False));print(json.dumps(result),flush=True)
if result.get('status')!='all_five_reconnected_control_topics_have_one_receiver':raise SystemExit(1)
