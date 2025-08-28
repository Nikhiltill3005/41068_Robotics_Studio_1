#!/usr/bin/env python3

"""
Drone Simulation Launch File

This launch file starts the Ignition Gazebo simulation with the firefighting drone
and provides an easy way to run the complete drone simulation setup.

Usage:
    ros2 launch firefighting_system drone_simulation.launch.py

To control the drone, run in a separate terminal:
    python3 src/firefighting_system/src/drone_teleop_control.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_firefighting_system = get_package_share_directory('firefighting_system')
    
    # Define launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_firefighting_system, 'worlds', 'drone_test_world.sdf'),
        description='Path to the world file'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run headless'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Set to true for verbose output'
    )
    
    # Launch Ignition Gazebo
    ignition_gazebo = ExecuteProcess(
        cmd=[
            'ign', 'gazebo',
            LaunchConfiguration('world_file')
        ],
        output='screen',
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': f"{pkg_firefighting_system}/models:{os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}"}
    )
    
    # ROS-Ignition Bridge
    bridge_config = os.path.join(pkg_firefighting_system, 'config', 'drone_bridge.yaml')
    
    ign_ros_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
            '--ros-args', '-p', f'config_file:={bridge_config}'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world_file_arg,
        gui_arg,
        verbose_arg,
        ignition_gazebo,
        ign_ros_bridge
    ])
