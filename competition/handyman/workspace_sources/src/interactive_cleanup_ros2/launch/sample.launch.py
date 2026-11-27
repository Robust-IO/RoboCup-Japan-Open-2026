#!/usr/bin/env python3
"""Launch only the interactive cleanup controller (no vision, for basic testing)."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='interactive_cleanup',
            executable='interactive_cleanup_sample',
            name='interactive_cleanup_sample',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
