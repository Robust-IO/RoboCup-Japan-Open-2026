import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LAYOUT_FILES = {
    'LayoutA': 'layout_a.yaml',
    'LayoutB': 'layout_b.yaml',
    'LayoutC': 'layout_c.yaml',
    'LayoutD': 'layout_d.yaml',
}


def launch_for_layout(context):
    layout = LaunchConfiguration('layout').perform(context)
    if layout not in LAYOUT_FILES:
        raise RuntimeError(
            f'Unknown layout {layout!r}; choose one of {list(LAYOUT_FILES)}')

    rebuild_share = get_package_share_directory('handyman_rebuild_ros2')
    legacy_share = get_package_share_directory('handyman_ros2')
    environment_yaml = os.path.join(
        rebuild_share, 'config', 'environments', LAYOUT_FILES[layout])
    with open(environment_yaml, encoding='utf-8') as stream:
        environment = yaml.safe_load(stream)
    map_yaml = os.path.join(rebuild_share, 'maps', layout, 'map.yaml')
    initial_pose = environment['initial_pose']
    localization_mode = LaunchConfiguration('localization_mode').perform(context)
    if localization_mode not in ('unity_odom', 'amcl'):
        raise RuntimeError(
            "localization_mode must be either 'unity_odom' or 'amcl'")

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml}],
    )
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server']}],
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'min_particles': 500,
            'max_particles': 3000,
            'kld_err': 0.02,
            'update_min_d': 0.20,
            'update_min_a': 0.20,
            'resample_interval': 2,
            'transform_tolerance': 0.5,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,
            'laser_max_range': 3.5,
            'laser_max_beams': 180,
            'laser_model_type': 'likelihood_field',
            'odom_model_type': 'diff',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
            'global_frame_id': 'map',
            'scan_topic': '/hsrb/base_scan',
            'set_initial_pose': True,
            'always_reset_initial_pose': True,
            'initial_pose.x': float(initial_pose['x']),
            'initial_pose.y': float(initial_pose['y']),
            'initial_pose.z': 0.0,
            'initial_pose.yaw': float(initial_pose['yaw']),
        }],
        remappings=[('scan', '/hsrb/base_scan')],
    )
    amcl_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['amcl']}],
    )
    unity_odom_localizer = Node(
        package='handyman_rebuild_ros2',
        executable='handyman_initial_pose_localizer',
        name='initial_pose_localizer',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'initial_pose.x': float(initial_pose['x']),
            'initial_pose.y': float(initial_pose['y']),
            'initial_pose.yaw': float(initial_pose['yaw']),
        }],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(legacy_share, 'launch', 'move_base.launch.py')),
    )
    coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rebuild_share, 'launch', 'handyman_rebuild.launch.py')),
        launch_arguments={
            'start_bridges': LaunchConfiguration('start_bridges'),
            'rosbridge_port': LaunchConfiguration('rosbridge_port'),
            'sigverse_port': LaunchConfiguration('sigverse_port'),
        }.items(),
    )
    localization = (
        [unity_odom_localizer]
        if localization_mode == 'unity_odom'
        else [amcl, amcl_lifecycle]
    )
    return [map_server, map_lifecycle, *localization, navigation, coordinator]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='LayoutA'),
        DeclareLaunchArgument('localization_mode', default_value='unity_odom'),
        DeclareLaunchArgument('start_bridges', default_value='true'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('sigverse_port', default_value='50001'),
        OpaqueFunction(function=launch_for_layout),
    ])
