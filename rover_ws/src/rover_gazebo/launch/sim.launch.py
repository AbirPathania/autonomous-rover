"""Full simulation bring-up: Gazebo + rover spawn + robot_state_publisher + RViz.

Usage:
    ros2 launch rover_gazebo sim.launch.py
    ros2 launch rover_gazebo sim.launch.py rviz:=false headless:=true
    ros2 launch rover_gazebo sim.launch.py localization:=true           # dead-reckoning EKF
    ros2 launch rover_gazebo sim.launch.py localization:=true slam:=true # full SLAM stack
    ros2 launch rover_gazebo sim.launch.py localization:=true terrain:=true # + terrain costmap
    ros2 launch rover_gazebo sim.launch.py hw:=true                     # + safety + HW bridge + MCU
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('rover_description')
    gazebo_share = get_package_share_directory('rover_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    loc_share = get_package_share_directory('rover_localization')
    terrain_share = get_package_share_directory('rover_terrain')
    safety_share = get_package_share_directory('rover_safety')
    hw_share = get_package_share_directory('rover_hw_bridge')

    xacro_file = os.path.join(desc_share, 'urdf', 'rover.urdf.xacro')
    default_world = os.path.join(gazebo_share, 'worlds', 'test_terrain.world')
    rviz_config = os.path.join(desc_share, 'rviz', 'rover.rviz')

    world_arg = DeclareLaunchArgument('world', default_value=default_world)
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    localization_arg = DeclareLaunchArgument(
        'localization', default_value='false',
        description='Run the robot_localization EKF stack (owns odom->base TF).')
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='false',
        description='With localization:=true, also start FAST-LIO2 SLAM + map EKF.')
    terrain_arg = DeclareLaunchArgument(
        'terrain', default_value='false',
        description='Run the terrain assessment node (slope/roughness costmap).')
    hw_arg = DeclareLaunchArgument(
        'hw', default_value='false',
        description='Run the safety manager + hardware bridge + SITL MCU emulator.')
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.3')

    localization = LaunchConfiguration('localization')
    slam = LaunchConfiguration('slam')
    hw = LaunchConfiguration('hw')

    # When the EKF stack runs, it owns odom->base_footprint, so the diff-drive
    # plugin must NOT publish that TF. Otherwise the plugin publishes it.
    publish_wheel_odom_tf = PythonExpression(
        ["'false' if '", localization, "' == 'true' else 'true'"])

    # Terrain grid frame: 'map' when SLAM is up (drift-corrected), else 'odom'.
    terrain_map_frame = PythonExpression(
        ["'map' if '", slam, "' == 'true' else 'odom'"])

    # With the hardware stack, the emulated MCU drives the motors, so the
    # diff-drive plugin obeys /motor/cmd_vel (Jetson -> safety -> bridge -> MCU).
    cmd_vel_topic = PythonExpression(
        ["'/motor/cmd_vel' if '", hw, "' == 'true' else 'cmd_vel'"])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                ' publish_wheel_odom_tf:=', publish_wheel_odom_tf,
                ' cmd_vel_topic:=', cmd_vel_topic]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                    'use_sim_time': True}],
    )

    # Gazebo server (physics) — always started
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'),
                        'verbose': 'true'}.items(),
    )

    # Gazebo client (GUI) — skipped when headless:=true
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    spawn_rover = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rover',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    # Localization stack (dual-EKF + mode manager, optional SLAM)
    localization_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, 'launch', 'localization.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                        'slam': LaunchConfiguration('slam')}.items(),
        condition=IfCondition(localization),
    )

    # Terrain assessment (slope/roughness drivability costmap)
    terrain_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(terrain_share, 'launch', 'terrain.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                        'map_frame': terrain_map_frame}.items(),
        condition=IfCondition(LaunchConfiguration('terrain')),
    )

    # Safety manager + hardware bridge + SITL MCU emulator
    safety_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_share, 'launch', 'safety.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(hw),
    )
    hw_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_share, 'launch', 'hw_bridge.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(hw),
    )

    return LaunchDescription([
        world_arg, rviz_arg, headless_arg, localization_arg, slam_arg,
        terrain_arg, hw_arg, x_arg, y_arg, z_arg,
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_rover,
        rviz,
        localization_stack,
        terrain_stack,
        safety_stack,
        hw_stack,
    ])
