# 原 ROS1: launch/sample_with_rviz.launch -> ROS2: launch/sample_with_rviz_launch.py
# 等价迁移：启动 human_navigation_sample、hsr_key_teleop 与 RViz2
# 按照官方 competition_test_tools 标准，包含 sigverse_ros_bridge 和 rosbridge_websocket
# 启动 human_navigation_sample + hsr_key_teleop + RViz（deterministic guidance only）

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    sigverse_ros_bridge_port = LaunchConfiguration("sigverse_ros_bridge_port").perform(context)
    ros_bridge_port = LaunchConfiguration("ros_bridge_port").perform(context)

    sub_joint_state_topic_name = LaunchConfiguration("sub_joint_state_topic_name").perform(context)
    pub_base_twist_topic_name = LaunchConfiguration("pub_base_twist_topic_name").perform(context)
    pub_arm_trajectory_topic_name = LaunchConfiguration(
        "pub_arm_trajectory_topic_name"
    ).perform(context)
    pub_gripper_trajectory_topic_name = LaunchConfiguration(
        "pub_gripper_trajectory_topic_name"
    ).perform(context)

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

    human_navigation_sample_node = Node(
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

    hsr_key_teleop_node = Node(
        package="human_nav_ros2",
        executable="hsr_key_teleop",
        name="human_navigation_hsr_key_teleop",
        output="screen",
        parameters=[
            {
                "sub_joint_state_topic_name": sub_joint_state_topic_name,
                "pub_base_twist_topic_name": pub_base_twist_topic_name,
                "pub_arm_trajectory_topic_name": pub_arm_trajectory_topic_name,
                "pub_gripper_trajectory_topic_name": pub_gripper_trajectory_topic_name,
            }
        ],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("human_nav_ros2"), "rviz", "hsr.rviz"
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    actions = [
        rosbridge_websocket,
        sigverse_ros_bridge_node,
    ]

    actions.extend(
        [
            human_navigation_sample_node,
            hsr_key_teleop_node,
            rviz2_node,
        ]
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
                "sub_joint_state_topic_name",
                default_value="/hsrb/joint_states",
                description="Topic name for joint states (hsr_key_teleop).",
            ),
            DeclareLaunchArgument(
                "pub_base_twist_topic_name",
                default_value="/hsrb/command_velocity",
                description="Topic name for base twist command.",
            ),
            DeclareLaunchArgument(
                "pub_arm_trajectory_topic_name",
                default_value="/hsrb/arm_trajectory_controller/command",
                description="Topic name for arm trajectory command.",
            ),
            DeclareLaunchArgument(
                "pub_gripper_trajectory_topic_name",
                default_value="/hsrb/gripper_controller/command",
                description="Topic name for gripper trajectory command.",
            ),
            DeclareLaunchArgument(
                "direction_hint_interval_sec",
                default_value="10.0",
                description="Interval (seconds) between directional hints.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
