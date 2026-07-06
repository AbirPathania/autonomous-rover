"""Keyboard teleop for driving the rover in sim.

Run this in its OWN terminal (it needs keyboard focus), after sim.launch.py:
    ros2 launch rover_bringup teleop.launch.py

Keys: i/k/j/l to move, space to stop, q/z to change speed. Publishes /cmd_vel.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # needs a real TTY; remove if running directly in a terminal
        remappings=[('/cmd_vel', '/cmd_vel')],
    )
    return LaunchDescription([teleop])
