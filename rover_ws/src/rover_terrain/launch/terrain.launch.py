"""Terrain assessment node bring-up.

Usage (normally included by sim.launch.py with terrain:=true):
    ros2 launch rover_terrain terrain.launch.py
    ros2 launch rover_terrain terrain.launch.py map_frame:=map   # when SLAM is running
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_terrain')
    cfg = os.path.join(pkg_share, 'config', 'terrain.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_frame = LaunchConfiguration('map_frame')

    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    map_frame_arg = DeclareLaunchArgument(
        'map_frame', default_value='odom',
        description='Reference frame for the terrain grid (odom, or map with SLAM).')

    terrain = Node(
        package='rover_terrain',
        executable='terrain_analysis_node',
        name='terrain_analysis_node',
        output='screen',
        parameters=[cfg, {'use_sim_time': use_sim_time, 'map_frame': map_frame}],
    )

    return LaunchDescription([
        sim_time_arg,
        map_frame_arg,
        terrain,
    ])
