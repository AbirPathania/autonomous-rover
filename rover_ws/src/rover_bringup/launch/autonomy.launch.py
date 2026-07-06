"""Full autonomy stack in simulation: sim + localization + terrain + Nav2.

Brings up, in one command:
    * Gazebo + the rover + RViz            (rover_gazebo/sim.launch.py)
    * dual-EKF localization (dead-reckoning) with odom->base_footprint
    * terrain assessment -> /terrain/costmap
    * Nav2 (A* global + DWB local @20Hz) with the terrain costmap + threat keepout

Then send a goal from RViz ("Nav2 Goal") or:
    ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{...}'

Usage:
    ros2 launch rover_bringup autonomy.launch.py
    ros2 launch rover_bringup autonomy.launch.py slam:=true   # use FAST-LIO2 map frame
    ros2 launch rover_bringup autonomy.launch.py mission:=true # also run the mission BT
    ros2 launch rover_bringup autonomy.launch.py mission:=true detection:=true  # full pipeline
    ros2 launch rover_bringup autonomy.launch.py mission:=true detection:=true hw:=true  # + safety/HW
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_share = get_package_share_directory('rover_gazebo')
    nav_share = get_package_share_directory('rover_navigation')
    mission_share = get_package_share_directory('rover_mission')
    detection_share = get_package_share_directory('rover_detection')

    slam = LaunchConfiguration('slam')
    rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    mission = LaunchConfiguration('mission')
    detection = LaunchConfiguration('detection')
    hw = LaunchConfiguration('hw')

    slam_arg = DeclareLaunchArgument('slam', default_value='false')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    mission_arg = DeclareLaunchArgument('mission', default_value='false')
    detection_arg = DeclareLaunchArgument('detection', default_value='false')
    hw_arg = DeclareLaunchArgument('hw', default_value='false')

    # Sim + localization + terrain in one include.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'localization': 'true',
            'terrain': 'true',
            'slam': slam,
            'rviz': rviz,
            'headless': headless,
            'hw': hw,
        }.items(),
    )

    # Nav2 navigation stack.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # Mission logic (BehaviorTree.CPP) -- optional.
    mission_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mission_share, 'launch', 'mission.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(mission),
    )

    # Detection & fusion pipeline -- optional.
    detection_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detection_share, 'launch', 'detection.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(detection),
    )

    return LaunchDescription([
        slam_arg, rviz_arg, headless_arg, mission_arg, detection_arg, hw_arg,
        sim,
        navigation,
        mission_stack,
        detection_stack,
    ])
