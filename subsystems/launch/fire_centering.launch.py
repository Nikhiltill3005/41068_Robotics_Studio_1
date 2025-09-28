#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled',
            default_value='false',
            description='Enable fire centering (true/false)'
        ),
        
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='2.0',
            description='Maximum linear velocity for drone movement'
        ),
        
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='2.0',
            description='Maximum angular velocity for drone rotation'
        ),
        
        DeclareLaunchArgument(
            'kp_linear',
            default_value='2.0',
            description='Proportional gain for linear movement'
        ),
        
        DeclareLaunchArgument(
            'kp_angular',
            default_value='3.0',
            description='Proportional gain for angular movement'
        ),
        
        DeclareLaunchArgument(
            'tolerance_pixels',
            default_value='20',
            description='Pixel tolerance for considering fire centered'
        ),
        
        DeclareLaunchArgument(
            'hover_z_velocity',
            default_value='0.1',
            description='Hover velocity when no fire detected'
        ),
        
        DeclareLaunchArgument(
            'fire_timeout',
            default_value='2.0',
            description='Timeout for fire detection in seconds'
        ),
        
        DeclareLaunchArgument(
            'target_height',
            default_value='7.5',
            description='Target height in meters'
        ),
        
        DeclareLaunchArgument(
            'height_tolerance',
            default_value='0.5',
            description='Height tolerance in meters'
        ),
        
        DeclareLaunchArgument(
            'kp_height',
            default_value='2.0',
            description='Proportional gain for height control'
        ),
        
        DeclareLaunchArgument(
            'kp_xy_fast',
            default_value='4.0',
            description='Fast proportional gain for x,y centering'
        ),

        # Fire centering node
        Node(
            package='drone_teleop',
            executable='fire_centering_node.py',
            name='fire_centering_node',
            output='screen',
            parameters=[{
                'enabled': LaunchConfiguration('enabled'),
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'kp_linear': LaunchConfiguration('kp_linear'),
                'kp_angular': LaunchConfiguration('kp_angular'),
                'tolerance_pixels': LaunchConfiguration('tolerance_pixels'),
                'hover_z_velocity': LaunchConfiguration('hover_z_velocity'),
                'fire_timeout': LaunchConfiguration('fire_timeout'),
                'target_height': LaunchConfiguration('target_height'),
                'height_tolerance': LaunchConfiguration('height_tolerance'),
                'kp_height': LaunchConfiguration('kp_height'),
                'kp_xy_fast': LaunchConfiguration('kp_xy_fast'),
            }]
        )
    ])
