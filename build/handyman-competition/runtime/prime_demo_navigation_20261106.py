"""Initialize controller plugin parameters then return to unconfigured; no goals."""
import json
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import GetState, ChangeState
from rcl_interfaces.srv import GetParameters

rclpy.init()
n=rclpy.create_node('demo_idle_navigation_initialization')
def call(srv, suffix, req):
    c=n.create_client(srv,'/controller_server/'+suffix)
    if not c.wait_for_service(timeout_sec=5): raise RuntimeError('service_missing:'+suffix)
    f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=15)
    if not f.done(): raise RuntimeError('service_timeout:'+suffix)
    return f.result()
names=['general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance','FollowPath.xy_goal_tolerance']
req=GetParameters.Request();req.names=names
report={'robot_commands_sent':False,'before_parameters':[]}
try:
    state=call(GetState,'get_state',GetState.Request()).current_state.id
    report['initial_state']=state
    report['before_parameters']=[{'type':v.type,'value':v.double_value} for v in call(GetParameters,'get_parameters',req).values]
    if state!=1: raise RuntimeError('expected_unconfigured_idle_controller')
    if n.count_publishers('/hsrb/command_velocity'): raise RuntimeError('velocity_publisher_present')
    transition=ChangeState.Request();transition.transition.id=1
    if not call(ChangeState,'change_state',transition).success: raise RuntimeError('configure_failed')
    report['configured_parameters']=[{'type':v.type,'value':v.double_value} for v in call(GetParameters,'get_parameters',req).values]
    transition.transition.id=2
    if not call(ChangeState,'change_state',transition).success: raise RuntimeError('cleanup_failed')
    report['final_parameters']=[{'type':v.type,'value':v.double_value} for v in call(GetParameters,'get_parameters',req).values]
    report['final_state']=call(GetState,'get_state',GetState.Request()).current_state.id
    if any(v['type']!=3 or abs(v['value']-e)>1e-8 for v,e in zip(report['final_parameters'],[.06,.08,.06])): raise RuntimeError('precision_not_loaded')
    report['status']='idle_precision_initialized'
except Exception as exc:
    report.update(status='failed',reason=str(exc))
finally:
    (Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')/'demo-navigation-initialization.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report));n.destroy_node();rclpy.shutdown()
