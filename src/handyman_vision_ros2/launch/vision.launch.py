from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vision_node = Node(
        package='handyman_vision_ros2',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(vision_node)

    return ld
