#!/usr/bin/env python3
"""
Full-stack launch for Interactive Cleanup:
  - interactive_cleanup_sample  (C++ controller)
  - head_perception_node        (Head RGB-D perception)
  - hand_perception_node        (Hand-camera perception)
  - sigverse_ros_bridge         (SIGVerse ↔ ROS 2)
  - rosbridge_websocket         (WebSocket bridge)
  - Nav2 localization + navigation (optional, default on)
  - RViz2 (optional, default off)
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# Robot initial pose in map frame (from Unity scene: x=-2.0, z=-3.5, yaw≈90°)
ROBOT_INIT_X = -2.0
ROBOT_INIT_Y = -3.5
ROBOT_INIT_YAW = math.pi / 2.0  # 90 degrees, facing +Y


def generate_launch_description():
    # Package paths
    pkg_interactive_cleanup = FindPackageShare('interactive_cleanup')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    # Launch arguments
    args = [
        DeclareLaunchArgument(
            'sigverse_ros_bridge_port', default_value='50001'),
        DeclareLaunchArgument(
            'ros_bridge_port', default_value='9090'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'use_nav2', default_value='true',
            description='Launch Nav2 navigation stack'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz2 with preconfigured display'),
    ]

    # Map and config file paths
    map_yaml = PathJoinSubstitution([
        pkg_interactive_cleanup, 'map', 'cleanup_map.yaml'])
    nav2_params = PathJoinSubstitution([
        pkg_interactive_cleanup, 'config', 'nav2_params.yaml'])
    rviz_config = PathJoinSubstitution([
        pkg_interactive_cleanup, 'config', 'cleanup.rviz'])
    nav2_common_parameters = [
        nav2_params,
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ]
    nav2_common_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]
    cmd_vel_remappings = nav2_common_remappings + [
        ('cmd_vel', '/hsrb/command_velocity'),
    ]

    # ---- Core nodes ----
    cleanup_node = Node(
        package='interactive_cleanup',
        executable='interactive_cleanup_sample',
        name='interactive_cleanup_sample',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'destination_regions_file': PathJoinSubstitution([
                pkg_interactive_cleanup, 'config', 'destination_regions.yaml']),
        }],
    )

    head_perception_node = Node(
        package='cleanup_vision_ros2',
        executable='head_perception_node',
        name='head_perception_node',
        output='screen',
    )

    hand_perception_node = Node(
        package='cleanup_vision_ros2',
        executable='hand_perception_node',
        name='hand_perception_node',
        output='screen',
    )

    sigverse_bridge = Node(
        package='sigverse_ros_bridge',
        executable='sigverse_ros_bridge',
        arguments=[LaunchConfiguration('sigverse_ros_bridge_port')],
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch', 'rosbridge_websocket_launch.xml',
            ])
        ),
        launch_arguments={
            'port': LaunchConfiguration('ros_bridge_port'),
            'default_call_service_timeout': '5.0',
            'call_services_in_new_thread': 'true',
            'send_action_goals_in_new_thread': 'true',
        }.items(),
    )

    # ---- TF placeholders ----
    # map→odom: static (AMCL will take over once it receives laser scans)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # odom→base_footprint: DYNAMIC (published on /tf, not /tf_static)
    # so SIGVerse bridge can seamlessly override it once connected.
    placeholder_tf = Node(
        package='interactive_cleanup',
        executable='placeholder_tf_publisher.py',
        name='placeholder_tf_odom_base',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'parent_frame': 'odom',
            'child_frame': 'base_footprint',
            'x': ROBOT_INIT_X,
            'y': ROBOT_INIT_Y,
            'z': 0.0,
            'yaw': ROBOT_INIT_YAW,
            'rate': 2.0,
        }],
    )

    # ---- Nav2 (conditional) ----
    # Use the stock localization launch, but launch navigation nodes explicitly
    # so velocity commands reach the HSR base topic directly.
    nav2_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_nav2')),
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([
                        pkg_nav2_bringup, 'launch', 'localization_launch.py',
                    ])
                ),
                launch_arguments={
                    'map': map_yaml,
                    'params_file': nav2_params,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': 'true',
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'log_level': 'info',
                }.items(),
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=nav2_common_parameters,
                remappings=cmd_vel_remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=nav2_common_parameters,
                remappings=nav2_common_remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=nav2_common_parameters,
                remappings=cmd_vel_remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=nav2_common_parameters,
                remappings=nav2_common_remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': True},
                    {'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                    ]},
                ],
            ),
        ],
    )

    # ---- RViz2 (conditional) ----
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription(
        args + [
            static_tf_map_odom,
            placeholder_tf,
            cleanup_node,
            head_perception_node,
            hand_perception_node,
            sigverse_bridge,
            rosbridge,
            nav2_launch,
            rviz_node,
        ]
    )
