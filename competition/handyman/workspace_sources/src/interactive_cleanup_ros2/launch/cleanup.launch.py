#!/usr/bin/env python3
"""
Full-stack launch for Interactive Cleanup:
  - interactive_cleanup_sample  (C++ controller)
  - cleanup_detection_node      (Python vision: YOLO + PoseLandmarker)
  - sigverse_ros_bridge         (SIGVerse ↔ ROS 2)
  - rosbridge_websocket         (WebSocket bridge)
  - Nav2 stack (optional, default on)
  - RViz2 (optional, default off)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


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
            'use_sim_time', default_value='true'),
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

    # ---- Core nodes ----
    cleanup_node = Node(
        package='interactive_cleanup',
        executable='interactive_cleanup_sample',
        name='interactive_cleanup_sample',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_nav2': LaunchConfiguration('use_nav2'),
        }],
    )

    vision_node = Node(
        package='cleanup_vision_ros2',
        executable='cleanup_detection_node',
        name='cleanup_detection_node',
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

    # ---- Nav2 (conditional) ----
    nav2_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_nav2')),
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([
                        pkg_nav2_bringup, 'launch', 'bringup_launch.py',
                    ])
                ),
                launch_arguments={
                    'map': map_yaml,
                    'params_file': nav2_params,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': 'true',
                }.items(),
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
            cleanup_node,
            vision_node,
            sigverse_bridge,
            rosbridge,
            nav2_launch,
            rviz_node,
        ]
    )
