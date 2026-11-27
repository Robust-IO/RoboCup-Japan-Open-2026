from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    sigverse_ros_bridge_port = LaunchConfiguration('sigverse_ros_bridge_port', default='50001')
    ros_bridge_port = LaunchConfiguration('ros_bridge_port', default='9090')

    teleop_key_node = Node(
        package='handyman_ros2',
        executable='teleop_key_handyman',
        name='teleop_key_handyman',
        output='screen',
        prefix='gnome-terminal --geometry=80x30 -- ',
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

    ld.add_action(DeclareLaunchArgument(
        'sigverse_ros_bridge_port', default_value='50001',
        description='Port for sigverse_ros_bridge.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'ros_bridge_port', default_value='9090',
        description='Port for rosbridge websocket.',
    ))

    ld.add_action(rosbridge_websocket)
    ld.add_action(sigverse_ros_bridge_node)
    ld.add_action(teleop_key_node)

    return ld
