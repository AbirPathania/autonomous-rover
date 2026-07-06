"""Display the rover URDF in RViz with joint sliders (no Gazebo).

Usage:
    ros2 launch rover_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    default_model = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'rviz', 'rover.rviz')

    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model,
        description='Absolute path to the robot xacro file')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch joint_state_publisher_gui with sliders')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('rover_description'), 'rviz', 'rover.rviz'])],
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz,
    ])
