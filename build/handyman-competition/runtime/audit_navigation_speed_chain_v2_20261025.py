"""Read navigation/smoother parameters only. No motion publishers."""
import json,time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
rclpy.init();node=rclpy.create_node('navigation_speed_chain_readonly')
result=dict(robot_commands_sent=False,nodes=[],parameters={})
try:
 end=time.monotonic()+2
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
 nodes=sorted(set((ns.rstrip('/')+'/'+name) for name,ns in node.get_node_names_and_namespaces()))
 result['nodes']=[n for n in nodes if any(k in n for k in ('controller','velocity','smoother','cmd_vel'))]
 for target in result['nodes']:
  if 'controller_server' in target:
   names=['FollowPath.'+p for p in ('max_vel_x','max_vel_y','max_speed_xy','max_vel_theta','acc_lim_x','acc_lim_y','acc_lim_theta','decel_lim_theta')]
  elif 'velocity_smoother' in target:names=['max_velocity','min_velocity','max_accel','max_decel','velocity_timeout']
  else:continue
  client=node.create_client(GetParameters,target+'/get_parameters')
  if not client.wait_for_service(timeout_sec=2):continue
  request=GetParameters.Request();request.names=names;future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=3)
  if not future.done():continue
  row={}
  for name,p in zip(names,future.result().values):
   value=Parameter.from_parameter_msg(type('Message',(),dict(name=name,value=p))()).value
   if value is not None and not isinstance(value,(str,int,float,bool)):value=list(value)
   row[name]=value
  result['parameters'][target]=row
finally:
 node.destroy_node();rclpy.shutdown()
 out=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/navigation-speed-chain-readonly-20261025.json')
 with out.open('x') as f:json.dump(result,f,indent=2)
 print(json.dumps(result))
