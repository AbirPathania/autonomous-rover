"""Localization stack: dual-EKF + degraded dead-reckoning manager (+ optional SLAM).

Always started:
    * ekf_local  -- fuses wheel /odom + /imu, publishes odom -> base_footprint
                    (the dead-reckoning estimate that survives LiDAR blackout).
    * localization_mode_manager -- watches LiDAR health, announces the active mode.

Started only when slam:=true (requires FAST-LIO2 built in the workspace):
    * FAST-LIO2 (via fastlio.launch.py) -- LiDAR-inertial SLAM.
    * ekf_map    -- fuses /fastlio/odometry, publishes map -> odom (drift correction).

Usage:
    ros2 launch rover_localization localization.launch.py            # dead-reckoning only
    ros2 launch rover_localization localization.launch.py slam:=true # full SLAM stack
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    loc_share = get_package_share_directory('rover_localization')
    ekf_cfg = os.path.join(loc_share, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')

    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='false',
        description='Start FAST-LIO2 SLAM + the map-frame EKF (needs FAST-LIO2 built).')

    # --- Local EKF: wheel + IMU -> odom->base_footprint (always on) ---
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_cfg, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered/local')],
    )

    # --- LiDAR-health / degraded dead-reckoning manager (always on) ---
    mode_manager = Node(
        package='rover_localization',
        executable='localization_mode_manager',
        name='localization_mode_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- Map EKF: SLAM pose -> map->odom (only with slam:=true) ---
    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map',
        output='screen',
        parameters=[ekf_cfg, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered/global')],
        condition=IfCondition(slam),
    )

    # --- FAST-LIO2 SLAM (only with slam:=true) ---
    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, 'launch', 'fastlio.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(slam),
    )

    return LaunchDescription([
        sim_time_arg,
        slam_arg,
        ekf_local,
        mode_manager,
        ekf_map,
        fastlio,
    ])
