"""Map active, navigation unconfigured, NO old map anchor or task sender."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    source=Path(get_package_share_directory('handyman_ros2'))
    rebuilt=Path(get_package_share_directory('handyman_rebuild_ros2'))
    params=str(source/'param/nav2_params.yaml')
    nodes=[Node(package='nav2_map_server',executable='map_server',name='map_server',
                parameters=[{'yaml_filename':str(rebuilt/'maps/LayoutA/map.yaml')}],output='screen'),
           Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_map',
                parameters=[{'autostart':True,'node_names':['map_server']}],output='screen')]
    for package,name in [('nav2_controller','controller_server'),('nav2_planner','planner_server'),
                         ('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator')]:
        overrides={}
        if name=='controller_server':
            overrides={'general_goal_checker.xy_goal_tolerance':.06,
                       'general_goal_checker.yaw_goal_tolerance':.08,'FollowPath.xy_goal_tolerance':.06}
        elif name=='bt_navigator':
            overrides={'default_nav_to_pose_bt_xml':str(source/'behavior_trees/navigate_w_recovery.xml')}
        nodes.append(Node(package=package,executable=name,name=name,parameters=[params,overrides],
                          remappings=[('cmd_vel','/hsrb/command_velocity')],output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',
                      parameters=[{'autostart':False,'node_names':['controller_server','planner_server','behavior_server','bt_navigator']}],output='screen'))
    return LaunchDescription(nodes)
