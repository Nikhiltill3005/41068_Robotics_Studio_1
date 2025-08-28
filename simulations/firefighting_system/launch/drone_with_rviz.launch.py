#!/usr/bin/env python3

"""
Drone Simulation with RViz Launch File

This launch file starts the integrated simulation (Husky + Drone) with RViz
for visualization of camera feeds and robot states.

Usage:
    ros2 launch firefighting_system drone_with_rviz.launch.py
    ros2 launch firefighting_system drone_with_rviz.launch.py world:=large_demo
    ros2 launch firefighting_system drone_with_rviz.launch.py use_rviz:=false
    ros2 launch firefighting_system drone_with_rviz.launch.py use_teleop:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    firefighting_pkg = get_package_share_directory('firefighting_system')
    ignition_pkg = get_package_share_directory('41068_ignition_bringup')
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        choices=['simple_trees', 'large_demo'],
        description='World to load (simple_trees or large_demo)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop',
        default_value='false',
        description='Whether to launch teleop control nodes'
    )
    
    # Path configurations
    rviz_config = PathJoinSubstitution([
        FindPackageShare('firefighting_system'),
        'config',
        'drone_rviz.rviz'
    ])
    
    # Set environment variables for model paths
    model_paths = os.pathsep.join([
        os.path.join(firefighting_pkg, 'models'),
        os.path.join(ignition_pkg, 'models'),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ])
    
    set_ign_gazebo_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', model_paths
    )
    
    set_gz_sim_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', model_paths
    )
    
    # Include the main 41068_ignition.launch.py with drone enabled
    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('41068_ignition_bringup'),
                'launch',
                '41068_ignition.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'drone': 'true'  # Enable drone in the main launch
        }.items()
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    # Husky Teleop (optional) - in new terminal
    husky_teleop = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=Husky Control', '--', 'bash', '-c',
            'echo "🤖 HUSKY TELEOP CONTROL"; '
            'echo "Use WASD keys to control Husky ground robot"; '
            'echo "Press Ctrl+C to stop"; echo ""; '
            'source /home/odyssey/ros2_ws/install/setup.bash && '
            'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
            '--ros-args --remap cmd_vel:=/husky_velocity_controller/cmd_vel_unstamped; '
            'exec bash'
        ],
        condition=IfCondition(LaunchConfiguration('use_teleop')),
        output='screen'
    )
    
    # Drone Teleop (optional) - in new terminal
    drone_teleop = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=Drone Control', '--', 'bash', '-c',
            'echo "🚁 DRONE TELEOP CONTROL"; '
            'echo "Use WASD keys to control firefighting drone"; '
            'echo "Press Ctrl+C to stop"; echo ""; '
            'source /home/odyssey/ros2_ws/install/setup.bash && '
            'ros2 run firefighting_system drone_teleop_ros.py; '
            'exec bash'
        ],
        condition=IfCondition(LaunchConfiguration('use_teleop')),
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        use_rviz_arg,
        use_teleop_arg,
        set_ign_gazebo_path,
        set_gz_sim_path,
        ignition_launch,
        rviz_node,
        husky_teleop,
        drone_teleop
    ])