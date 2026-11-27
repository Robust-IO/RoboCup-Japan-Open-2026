from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_bridges = LaunchConfiguration('start_bridges')
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    sigverse_port = LaunchConfiguration('sigverse_port')

    return LaunchDescription([
        DeclareLaunchArgument('start_bridges', default_value='true'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('sigverse_port', default_value='50001'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rosbridge_server'), 'launch',
                'rosbridge_websocket_launch.xml',
            ])),
            launch_arguments={'port': rosbridge_port}.items(),
            condition=IfCondition(start_bridges),
        ),
        Node(
            package='sigverse_ros_bridge',
            executable='sigverse_ros_bridge',
            arguments=[sigverse_port],
            output='screen',
            condition=IfCondition(start_bridges),
        ),
        Node(
            package='handyman_rebuild_ros2',
            executable='handyman_coordinator',
            output='screen',
        ),
    ])
