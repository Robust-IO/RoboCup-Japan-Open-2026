"""Search integration bringup; no coordinator, bridges, inference or goals.

Requires an explicit layout and confirmation that the robot is at that layout's
initial pose. Navigation defaults OFF. When enabled, its normal velocity output
can control the robot if another client sends a goal; this is not read-only.
"""
import math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    layout=LaunchConfiguration('layout').perform(context)
    files={'LayoutA':'layout_a.yaml','LayoutB':'layout_b.yaml','LayoutC':'layout_c.yaml','LayoutD':'layout_d.yaml'}
    if layout not in files:raise RuntimeError('Explicit layout must be LayoutA/B/C/D')
    if LaunchConfiguration('confirm_initial_pose').perform(context).lower()!='true':
        raise RuntimeError('Confirm robot is still at the exported initial pose; otherwise this localizer is inappropriate')
    enable=LaunchConfiguration('enable_navigation').perform(context).lower()
    if enable not in ('true','false'):raise RuntimeError('enable_navigation must be true or false')
    share=Path(get_package_share_directory('handyman_rebuild_ros2'))
    env=yaml.safe_load((share/'config/environments'/files[layout]).read_text())
    map_path=share/'maps'/layout/'map.yaml'
    if env['internal_name']!=layout or not map_path.is_file():raise RuntimeError('Layout/map configuration mismatch')
    pose={key:float(env['initial_pose'][key]) for key in ('x','y','yaw')}
    if not all(math.isfinite(v) for v in pose.values()):raise RuntimeError('Invalid initial pose')
    actions=[Node(package='nav2_map_server',executable='map_server',name='map_server',
                  parameters=[{'yaml_filename':str(map_path)}],output='screen'),
             Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_map',
                  parameters=[{'autostart':True,'node_names':['map_server']}],output='screen'),
             Node(package='handyman_rebuild_ros2',executable='handyman_initial_pose_localizer',
                  name='initial_pose_localizer',parameters=[{'map_frame':'map','odom_frame':'odom','base_frame':'base_footprint',
                  **{'initial_pose.'+key:value for key,value in pose.items()}}],output='screen')]
    if enable=='true':
        legacy=Path(get_package_share_directory('handyman_ros2'))
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(legacy/'launch/move_base.launch.py'))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('layout',description='Required actual Unity layout'),
        DeclareLaunchArgument('confirm_initial_pose',default_value='false'),
        DeclareLaunchArgument('enable_navigation',default_value='false'),
        OpaqueFunction(function=setup)])
