"""Detection & fusion pipeline bring-up.

Starts the simulated buried-threat sensors, the GPR processor, the fusion node,
and (optionally) the evaluation node. Publishes /detection/threat to the mission
layer and /detection/confidence for RViz.

Usage (normally via autonomy.launch.py detection:=true):
    ros2 launch rover_detection detection.launch.py
    ros2 launch rover_detection detection.launch.py evaluate:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_detection')
    cfg = os.path.join(pkg_share, 'config', 'detection.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    evaluate = LaunchConfiguration('evaluate')

    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    evaluate_arg = DeclareLaunchArgument('evaluate', default_value='true')

    common = [cfg, {'use_sim_time': use_sim_time}]

    sensor_sim = Node(
        package='rover_detection', executable='sensor_sim_node',
        name='sensor_sim_node', output='screen', parameters=common)
    gpr_processor = Node(
        package='rover_detection', executable='gpr_processor_node',
        name='gpr_processor_node', output='screen', parameters=common)
    fusion = Node(
        package='rover_detection', executable='fusion_node',
        name='fusion_node', output='screen', parameters=common)
    evaluator = Node(
        package='rover_detection', executable='detection_eval_node',
        name='detection_eval_node', output='screen', parameters=common,
        condition=IfCondition(evaluate))

    return LaunchDescription([
        sim_time_arg, evaluate_arg,
        sensor_sim,
        gpr_processor,
        fusion,
        evaluator,
    ])
