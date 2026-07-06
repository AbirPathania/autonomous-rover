"""Hardware bridge + SITL MCU emulator bring-up.

Starts the Jetson-side bridge and the emulated microcontroller. The MCU drives
the sim motors on /motor/cmd_vel, so the rover must be spawned with the diff-drive
command topic set to /motor/cmd_vel (sim.launch.py hw:=true does this).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_hw_bridge')
    cfg = os.path.join(pkg_share, 'config', 'hw.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    common = [cfg, {'use_sim_time': use_sim_time}]

    bridge = Node(
        package='rover_hw_bridge', executable='hw_bridge_node',
        name='hw_bridge_node', output='screen', parameters=common)
    mcu = Node(
        package='rover_hw_bridge', executable='mcu_sim_node',
        name='mcu_sim_node', output='screen', parameters=common)

    return LaunchDescription([sim_time_arg, bridge, mcu])
