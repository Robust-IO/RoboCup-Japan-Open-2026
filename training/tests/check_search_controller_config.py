"""Configure but NEVER activate a controller, in domain 73 only. No goals."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('Requires domain 73 localhost')
    import rclpy
    from lifecycle_msgs.srv import ChangeState,GetState
    from rcl_interfaces.srv import GetParameters
    from ament_index_python.packages import get_package_share_directory
    root=Path(__file__).resolve().parents[2]
    spec=importlib.util.spec_from_file_location('control',root/'src/handyman_rebuild_ros2/launch/phase4_search_control.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    values=module.SEARCH_GOAL_OVERRIDES
    assert values['general_goal_checker.xy_goal_tolerance']<.15
    assert values['general_goal_checker.yaw_goal_tolerance']<.17453292519943295
    assert values['FollowPath.xy_goal_tolerance']==values['general_goal_checker.xy_goal_tolerance']
    command=['/opt/ros/humble/lib/nav2_controller/controller_server','--ros-args','--params-file',
             str(Path(get_package_share_directory('handyman_ros2'))/'param/nav2_params.yaml'),
             '-r','cmd_vel:=/handyman_test/unused_velocity']
    for key,value in values.items():command+=['-p',key+':='+str(value)]
    child=subprocess.Popen(command)
    rclpy.init();node=rclpy.create_node('search_controller_config_check')
    def call(kind,name,request):
        client=node.create_client(kind,name)
        if not client.wait_for_service(timeout_sec=8.):raise RuntimeError('service unavailable: '+name)
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=10.)
        if not future.done():raise TimeoutError(name)
        return future.result()
    try:
        request=ChangeState.Request();request.transition.id=1 # CONFIGURE only.
        assert call(ChangeState,'/controller_server/change_state',request).success
        assert call(GetState,'/controller_server/get_state',GetState.Request()).current_state.id==2 # INACTIVE
        request=GetParameters.Request();request.names=list(values)
        response=call(GetParameters,'/controller_server/get_parameters',request)
        actual={k:v.double_value for k,v in zip(request.names,response.values)}
        assert actual==values,actual
        print(json.dumps(dict(passed=True,controller_state='inactive',parameters=actual,motion_started=False)))
    finally:
        node.destroy_node();rclpy.try_shutdown()
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)


if __name__=='__main__':main()
