"""FAST-LIO2 LiDAR-inertial SLAM for the rover (sim).

FAST-LIO2 (hku-mars/FAST_LIO) is an EXTERNAL package -- clone and build it in the
workspace first (see docs/setup.md, "FAST-LIO2"). This launch starts its mapping
node with the sim-tuned config and remaps its odometry to /fastlio/odometry so
the map-frame EKF (ekf_map) can consume it.

Usage (normally included by localization.launch.py with slam:=true):
    ros2 launch rover_localization fastlio.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    loc_share = get_package_share_directory('rover_localization')
    fastlio_cfg = os.path.join(loc_share, 'config', 'fastlio_sim.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    # FAST-LIO2 mapping node.
    #   - Remap its default /Odometry output to /fastlio/odometry.
    #   - We intentionally do NOT let its internal camera_init->body TF drive the
    #     robot's transform tree; ekf_map owns map->odom from the odometry topic.
    fastlio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[fastlio_cfg, {'use_sim_time': use_sim_time}],
        remappings=[('/Odometry', '/fastlio/odometry')],
    )

    # Align FAST-LIO2's world frame ("camera_init") with our "map" frame so the
    # /fastlio/odometry measurement resolves correctly for ekf_map.
    map_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        sim_time_arg,
        fastlio,
        map_to_camera_init,
    ])
