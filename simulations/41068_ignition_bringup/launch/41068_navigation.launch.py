"""
Navigation launch file for Husky robot with SLAM capabilities.

This launch file provides:
- SLAM using slam_toolbox for real-time mapping
- Nav2 navigation stack for autonomous navigation with proper /husky/ namespacing
- Navigation GUI for setting X,Y coordinate goals
- Optimized parameters for Husky robot in forest environment
- Support for both SLAM and localization modes

Usage:
- SLAM mode (default): ros2 launch 41068_ignition_bringup 41068_navigation.launch.py
- Localization mode: ros2 launch 41068_ignition_bringup 41068_navigation.launch.py map_file:=/path/to/map.yaml
- With GUI (default): ros2 launch 41068_ignition_bringup 41068_navigation.launch.py
- Without GUI: ros2 launch 41068_ignition_bringup 41068_navigation.launch.py use_gui:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    ld = LaunchDescription()

    config_path = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config'])

    # Additional command line arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Map file argument for potential map-based localization
    map_file = LaunchConfiguration('map_file')
    map_file_launch_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to map file for localization mode (leave empty for SLAM mode)'
    )

    # GUI argument
    use_gui = LaunchConfiguration('use_gui')
    use_gui_launch_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Flag to enable navigation GUI'
    )

    # Start Simultaneous Localisation and Mapping (SLAM) for Husky
    # Uses slam_toolbox with optimized parameters for forest environment
    slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                             'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_path, 'slam_params.yaml'])
        }.items()
    )

    # Start Navigation Stack for Husky with proper namespacing
    # Uses Nav2 with parameters optimized for SLAM and forest navigation
    # Remaps cmd_vel to /husky/cmd_vel for proper robot control
    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'nav2_params_husky.yaml'])
        }.items()
    )

    # Create a custom odometry relay that also publishes the transform
    # This creates both /odom topic and odom->base_link transform
    odom_relay = Node(
        package='41068_ignition_bringup',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Add cmd_vel remapping to properly namespace Nav2 output for Husky
    # This ensures Nav2 publishes to /husky/cmd_vel which the robot expects
    nav2_cmd_vel_remap = Node(
        package='41068_ignition_bringup',
        executable='cmd_vel_relay.py',
        name='nav2_cmd_vel_relay',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Navigation GUI for setting goals via X,Y coordinates
    navigation_gui = Node(
        package='41068_ignition_bringup',
        executable='navigation_gui.py',
        name='navigation_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_gui)
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(map_file_launch_arg)
    ld.add_action(use_gui_launch_arg)
    ld.add_action(odom_relay)
    ld.add_action(slam)
    ld.add_action(navigation)
    ld.add_action(nav2_cmd_vel_remap)
    ld.add_action(navigation_gui)

    return ld
