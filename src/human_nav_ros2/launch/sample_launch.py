# 原 ROS1: launch/sample.launch -> ROS2: launch/sample_launch.py
# 等价迁移：启动 human_navigation_sample 节点
# 按照官方 competition_test_tools 标准，包含 sigverse_ros_bridge 和 rosbridge_websocket
# 启动 human_navigation_sample（deterministic guidance only）

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    sigverse_ros_bridge_port = LaunchConfiguration("sigverse_ros_bridge_port").perform(context)
    ros_bridge_port = LaunchConfiguration("ros_bridge_port").perform(context)

    direction_hint_interval = float(LaunchConfiguration("direction_hint_interval_sec").perform(context))

    rosbridge_websocket = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml",
            ])
        ),
        launch_arguments={
            "port": ros_bridge_port,
            "default_call_service_timeout": "5.0",
            "call_services_in_new_thread": "true",
            "send_action_goals_in_new_thread": "true",
        }.items(),
    )

    sigverse_ros_bridge_node = Node(
        package="sigverse_ros_bridge",
        executable="sigverse_ros_bridge",
        name="sigverse_ros_bridge",
        arguments=[sigverse_ros_bridge_port],
        output="screen",
    )

    actions = [
        rosbridge_websocket,
        sigverse_ros_bridge_node,
    ]

    actions.append(
        Node(
            package="human_nav_ros2",
            executable="human_navigation_sample",
            name="human_navigation_sample",
            output="screen",
            parameters=[
                {
                    "direction_hint_interval_sec": direction_hint_interval,
                }
            ],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sigverse_ros_bridge_port",
                default_value="50001",
                description="Port for sigverse_ros_bridge.",
            ),
            DeclareLaunchArgument(
                "ros_bridge_port",
                default_value="9090",
                description="Port for rosbridge websocket.",
            ),
            DeclareLaunchArgument(
                "direction_hint_interval_sec",
                default_value="10.0",
                description="Interval (seconds) between directional hints (Forward/Left/Right/Backward).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
