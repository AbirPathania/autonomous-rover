"""Mission logic (BehaviorTree.CPP) bring-up.

Ticks the rover mission tree, which drives Nav2 via the /navigate_to_pose action.
Run this on top of the autonomy stack (sim + localization + terrain + Nav2), e.g.
after rover_bringup autonomy.launch.py.

Usage:
    ros2 launch rover_mission mission.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_mission')
    default_params = os.path.join(pkg_share, 'config', 'mission.yaml')
    default_bt = os.path.join(pkg_share, 'bt', 'mission.xml')

    # Named mission_params_file (not params_file) to avoid colliding with
    # rover_navigation's own params_file argument when both are included
    # together (e.g. via rover_bringup autonomy.launch.py) -- LaunchConfiguration
    # names are global across a launch tree, so two same-named
    # DeclareLaunchArgument calls collide and one silently loses its file.
    mission_params_file = LaunchConfiguration('mission_params_file')
    bt_xml = LaunchConfiguration('bt_xml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    params_arg = DeclareLaunchArgument('mission_params_file', default_value=default_params)
    bt_arg = DeclareLaunchArgument('bt_xml', default_value=default_bt)
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    mission = Node(
        package='rover_mission',
        executable='mission_server',
        name='mission_server',
        output='screen',
        parameters=[mission_params_file, {'use_sim_time': use_sim_time, 'bt_xml': bt_xml}],
    )

    return LaunchDescription([
        params_arg,
        bt_arg,
        sim_time_arg,
        mission,
    ])
