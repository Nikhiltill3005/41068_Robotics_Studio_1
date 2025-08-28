#!/usr/bin/env python3

"""
RViz Only Launch File

This launch file starts only RViz with the drone configuration.
Use this when the simulation is already running.

Usage:
    ros2 launch firefighting_system rviz_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='drone_rviz.rviz',
        description='RViz configuration file to use'
    )
    
    # Path configurations
    rviz_config = PathJoinSubstitution([
        FindPackageShare('firefighting_system'),
        'config',
        LaunchConfiguration('rviz_config')
    ])
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        config_arg,
        rviz_node
    ])
