"""Load declared Nav2 plugin parameters without activating navigation or sending a goal."""
import json
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.srv import GetParameters

output = Path('/__HANDYMAN_PACKAGE__/runtime/apple-initial-readiness-20261018-01/controller-idle-parameter-initialization.json')
if output.exists(): raise ValueError('output_exists')
rclpy.init(); n = rclpy.create_node('initialize_idle_controller_parameters')
def call(kind, name, request):
    client = n.create_client(kind, '/controller_server/'+name)
    if not client.wait_for_service(timeout_sec=5): raise ValueError('service_missing:'+name)
    f = client.call_async(request); rclpy.spin_until_future_complete(n, f, timeout_sec=20)
    if not f.done(): raise ValueError('service_timeout:'+name)
    return f.result()
def state(): return call(GetState,'get_state',GetState.Request()).current_state.id
def params():
    req = GetParameters.Request(); req.names = ['general_goal_checker.xy_goal_tolerance',
        'general_goal_checker.yaw_goal_tolerance','FollowPath.xy_goal_tolerance']
    return [dict(type=v.type, value=v.double_value) for v in call(GetParameters,'get_parameters',req).values]
result = dict(robot_motion_commands_sent=False, navigation_activation_requested=False,
              navigation_goal_sent=False, parameter_values_changed=False)
try:
    result['before_state'] = state(); result['before_parameters'] = params()
    if result['before_state'] != 1: raise ValueError('only_idle_unconfigured_controller_allowed')
    req = ChangeState.Request(); req.transition.id = 1
    if not call(ChangeState,'change_state',req).success: raise ValueError('configure_failed')
    try:
        result['configured_parameters'] = values = params()
        if any(v['type'] != 3 or abs(v['value']-expected)>1e-8 for v,expected in zip(values,[.06,.08,.06])):
            raise ValueError('configured_precision_not_verified_do_not_adjust')
    finally:
        req = ChangeState.Request(); req.transition.id = 2
        if not call(ChangeState,'change_state',req).success: raise ValueError('cleanup_failed')
    result['after_state'] = state(); result['after_parameters'] = params()
    if result['after_state'] != 1: raise ValueError('controller_not_idle_after_cleanup')
    result['status'] = 'plugin_parameters_initialized_navigation_not_activated'
except Exception as exc:
    result.update(status='failed', reason=str(exc)); raise
finally:
    output.write_text(json.dumps(result, indent=2)); print(json.dumps(result), flush=True)
    n.destroy_node(); rclpy.shutdown()
