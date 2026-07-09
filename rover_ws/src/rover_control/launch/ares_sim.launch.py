"""ARES-G full simulation bring-up (genuine four-wheel-steering model).

    ros2 launch rover_control ares_sim.launch.py
    ros2 launch rover_control ares_sim.launch.py headless:=true rviz:=false cameras:=false
    ros2 launch rover_control ares_sim.launch.py localization:=true terrain:=true

Brings up Gazebo + the dimensionally-accurate ARES-G rover. The wheels are
driven by the gazebo_ros diff-drive plugin (robust motion + /odom); the four
steered kingpins and the marker arm are actuated by the joint_pose_trajectory
plugin, commanded by the 4WS steering node from /cmd_vel.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    xacro_file = os.path.join(desc_share, 'urdf', 'ares_g.urdf.xacro')
    default_world = os.path.join(gazebo_share, 'worlds', 'test_terrain.world')
    rviz_config = os.path.join(desc_share, 'rviz', 'rover.rviz')

    world_arg = DeclareLaunchArgument('world', default_value=default_world)
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    localization_arg = DeclareLaunchArgument('localization', default_value='false')
    slam_arg = DeclareLaunchArgument('slam', default_value='false')
    terrain_arg = DeclareLaunchArgument('terrain', default_value='false')
    cameras_arg = DeclareLaunchArgument(
        'cameras', default_value='true',
        description='Enable the RGB/depth cameras. Set false for headless '
                    'gzserver on GPU-less hosts (Codespaces) to avoid a crash.')
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.25')

    localization = LaunchConfiguration('localization')
    slam = LaunchConfiguration('slam')

    # When the EKF stack runs it owns odom->base_footprint, so diff-drive must
    # not also publish that TF.
    publish_wheel_odom_tf = PythonExpression(
        ["'false' if '", localization, "' == 'true' else 'true'"])
    terrain_map_frame = PythonExpression(
        ["'map' if '", slam, "' == 'true' else 'odom'"])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                ' publish_wheel_odom_tf:=', publish_wheel_odom_tf,
                ' enable_cameras:=', LaunchConfiguration('cameras')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                    'use_sim_time': True}],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'),
                        'verbose': 'true'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    spawn_rover = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ares_g',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z')],
        output='screen',
    )

    # Four-station steering commander (/cmd_vel -> kingpin JointTrajectory).
    steering = Node(
        package='rover_control', executable='ares_kinematics_node',
        output='screen', parameters=[{'use_sim_time': True}],
    )
    # Marker arm (Bool /marker/deploy -> marker JointTrajectory).
    marker_arm = Node(
        package='rover_control', executable='marker_arm_node',
        output='screen', parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')), output='screen',
    )

    localization_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, 'launch', 'localization.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'slam': slam}.items(),
        condition=IfCondition(localization),
    )
    terrain_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(terrain_share, 'launch', 'terrain.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                        'map_frame': terrain_map_frame}.items(),
        condition=IfCondition(LaunchConfiguration('terrain')),
    )

    return LaunchDescription([
        world_arg, rviz_arg, headless_arg, localization_arg, slam_arg,
        terrain_arg, cameras_arg, x_arg, y_arg, z_arg,
        robot_state_publisher,
        gzserver, gzclient,
        spawn_rover,
        steering, marker_arm,
        rviz,
        localization_stack, terrain_stack,
    ])
