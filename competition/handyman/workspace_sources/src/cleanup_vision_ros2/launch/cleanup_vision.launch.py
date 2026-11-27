#!/usr/bin/env python3
"""Launch the new head/hand perception nodes standalone."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cleanup_vision_ros2',
            executable='head_perception_node',
            name='head_perception_node',
            output='screen',
        ),
        Node(
            package='cleanup_vision_ros2',
            executable='hand_perception_node',
            name='hand_perception_node',
            output='screen',
        ),
    ])
