"""Map and Nav2 for a selected layout; no task controller or duplicate bridge."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    rebuild=get_package_share_directory('handyman_rebuild_ros2')
    legacy=get_package_share_directory('handyman_ros2')
    return LaunchDescription([
        DeclareLaunchArgument('layout',default_value='LayoutA',choices=['LayoutA','LayoutB','LayoutC','LayoutD']),
        Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',parameters=[{'yaml_filename':PathJoinSubstitution([rebuild,'maps',LaunchConfiguration('layout'),'map.yaml'])}]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_map',parameters=[{'autostart':True,'node_names':['map_server']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(legacy,'launch','move_base.launch.py')),launch_arguments={'cmd_vel_topic':'/handyman/navigation_velocity_unchecked'}.items()),
    ])
