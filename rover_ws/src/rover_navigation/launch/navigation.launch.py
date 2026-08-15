"""Nav2 navigation bring-up for the rover.

Starts the Nav2 servers (planner=A*, controller=DWB@20Hz), both costmaps layered
on /terrain/costmap, the keepout costmap-filter server, and the threat-zone node
that turns confirmed threats into exclusion polygons. Managed by a single
lifecycle manager (autostart).

Prerequisites (run these first, e.g. via rover_bringup autonomy.launch.py):
    * the simulation + a localization source publishing odom->base_footprint
    * rover_terrain publishing /terrain/costmap (in the 'odom' frame)

Usage:
    ros2 launch rover_navigation navigation.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_navigation')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Named nav_params_file (not params_file) because this launch tree also
    # includes rover_mission's own params_file argument -- LaunchConfiguration
    # names are global across an entire launch tree, so two same-named
    # DeclareLaunchArgument calls collide: whichever executes first silently
    # wins and the other resolves empty, which drops all of this file's
    # content from every node below (manifests as e.g. DWB's "No critics
    # defined for FollowPath" with no indication the params file was ever
    # read).
    nav_params_file = LaunchConfiguration('nav_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    params_arg = DeclareLaunchArgument('nav_params_file', default_value=default_params)
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')

    # Each Node() below gets its own [nav_params_file] list rather than a
    # shared list object -- use_sim_time is already set per-node inside
    # nav2_params.yaml, so there's no need to merge in an extra dict either.

    # Nav2 lifecycle-managed servers (order matters for the manager list).
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'costmap_filter_info_server',
    ]

    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav_params_file],
        remappings=[('cmd_vel', '/cmd_vel')],  # drive the Gazebo diff-drive
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', parameters=[nav_params_file],
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen', parameters=[nav_params_file],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen', parameters=[nav_params_file],
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=[nav_params_file],
    )
    costmap_filter_info_server = Node(
        package='nav2_map_server', executable='costmap_filter_info_server',
        name='costmap_filter_info_server', output='screen', parameters=[nav_params_file],
    )

    # Threat exclusion-zone manager (plain node, publishes the keepout mask).
    threat_zone = Node(
        package='rover_navigation', executable='threat_zone_node',
        name='threat_zone_node', output='screen', parameters=[nav_params_file],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes}],
    )

    return LaunchDescription([
        params_arg, sim_time_arg, autostart_arg,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        costmap_filter_info_server,
        threat_zone,
        lifecycle_manager,
    ])
