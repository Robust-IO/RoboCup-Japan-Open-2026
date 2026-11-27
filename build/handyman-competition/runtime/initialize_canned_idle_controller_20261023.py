"""Configure then clean up an idle controller so plugin defaults are declared.

No activation, action goal or parameter setting. Fresh-start Nav2 plugins do not
declare their parameters until configure; unset must not be mistaken for changed.
"""
import json
from pathlib import Path
import time
import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState,GetState
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Twist

output=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/canned-initial-20261023-01/controller-initialization.json')
if output.exists():raise ValueError('new_initialization_evidence_required')
rclpy.init();node=rclpy.create_node('initialize_idle_controller_without_motion');messages=[]
sub=node.create_subscription(Twist,'/hsrb/command_velocity',lambda m:messages.append(dict(x=m.linear.x,y=m.linear.y,w=m.angular.z)),10)
def call(service,kind,request,timeout=20):
    client=node.create_client(kind,service)
    if not client.wait_for_service(timeout_sec=5):raise ValueError('service_unavailable:'+service)
    future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
    if not future.done():raise ValueError('service_timeout:'+service)
    return future.result()
def state():return call('/controller_server/get_state',GetState,GetState.Request()).current_state.id
def parameters():
    req=GetParameters.Request();req.names=['general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance','FollowPath.xy_goal_tolerance']
    return [dict(type=v.type,double_value=v.double_value) for v in call('/controller_server/get_parameters',GetParameters,req).values]
result=dict(robot_commands_sent=False,parameters_set=False,activation_requested=False)
try:
    result['before_state']=state();result['before_parameters']=parameters()
    if result['before_state']!=1:raise ValueError('only_fresh_idle_unconfigured_controller_supported')
    req=ChangeState.Request();req.transition.id=Transition.TRANSITION_CONFIGURE
    if not call('/controller_server/change_state',ChangeState,req).success:raise ValueError('controller_configure_failed')
    result['configured_state']=state();result['configured_parameters']=parameters()
    if result['configured_state']!=2:raise ValueError('controller_not_inactive')
    req=ChangeState.Request();req.transition.id=Transition.TRANSITION_CLEANUP
    if not call('/controller_server/change_state',ChangeState,req).success:raise ValueError('controller_cleanup_failed')
    result['after_state']=state();result['after_parameters']=parameters()
    end=time.monotonic()+1
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    result['received_velocity_messages']=messages
    if result['after_state']!=1 or messages:raise ValueError('idle_controller_or_no_velocity_proof_failed')
    if any(v['type']!=3 or abs(v['double_value']-expected)>1e-8 for v,expected in zip(result['after_parameters'],[.06,.08,.06])):
        raise ValueError('verified_precision_not_available_after_plugin_initialization')
    result['status']='idle_plugin_parameters_initialized_without_activation_or_parameter_changes'
except Exception as exc:result.update(status='initialization_incomplete',reason=str(exc))
finally:
    with output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result),flush=True);node.destroy_node();rclpy.shutdown()
if result['status']!='idle_plugin_parameters_initialized_without_activation_or_parameter_changes':raise SystemExit(1)
