from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='handyman_rebuild_ros2',
            executable='handyman_phase1_simulation',
            output='screen',
            additional_env={'ROS_LOCALHOST_ONLY': '1'},
        ),
    ])
