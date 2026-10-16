import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    scan_topic = LaunchConfiguration('scan_topic', default='/hsrb/base_scan')
    sigverse_ros_bridge_port = LaunchConfiguration('sigverse_ros_bridge_port', default='50001')
    ros_bridge_port = LaunchConfiguration('ros_bridge_port', default='9090')

    pkg_share = get_package_share_directory('handyman_ros2')
    rviz_config = os.path.join(pkg_share, 'launch', 'hsr.rviz')

    teleop_key_node = Node(
        package='handyman_ros2',
        executable='teleop_key_handyman',
        name='teleop_key_handyman',
        output='screen',
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_update_interval': 0.1,
            'max_laser_range': 4.0,
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.2,
            'resolution': 0.05,
        }],
        remappings=[
            ('scan', scan_topic),
        ],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
    )

    sigverse_ros_bridge_node = Node(
        package='sigverse_ros_bridge',
        executable='sigverse_ros_bridge',
        name='sigverse_ros_bridge',
        arguments=[sigverse_ros_bridge_port],
        output='screen',
    )

    rosbridge_websocket = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml',
            ])
        ),
        launch_arguments={
            'port': ros_bridge_port,
            'default_call_service_timeout': '5.0',
            'call_services_in_new_thread': 'true',
            'send_action_goals_in_new_thread': 'true',
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('scan_topic', default_value='/hsrb/base_scan'))
    ld.add_action(DeclareLaunchArgument('sigverse_ros_bridge_port', default_value='50001'))
    ld.add_action(DeclareLaunchArgument('ros_bridge_port', default_value='9090'))

    ld.add_action(rosbridge_websocket)
    ld.add_action(sigverse_ros_bridge_node)
    ld.add_action(rviz2_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(teleop_key_node)

    return ld
