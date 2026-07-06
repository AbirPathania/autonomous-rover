"""Safety/fault manager bring-up."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_safety')
    cfg = os.path.join(pkg_share, 'config', 'safety.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    safety = Node(
        package='rover_safety',
        executable='safety_manager_node',
        name='safety_manager_node',
        output='screen',
        parameters=[cfg, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([sim_time_arg, safety])
