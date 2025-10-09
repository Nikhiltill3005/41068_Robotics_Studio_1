#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device'
        ),

        # Joy driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'device': LaunchConfiguration('joy_dev'),
                'autorepeat_rate': 20.0,
                'deadzone': 0.12,
            }]
        ),

        # Combined teleop node
        Node(
            package='drone_teleop',
            executable='combined_joy_teleop',
            name='combined_joy_teleop',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Common
                'enable_button': 4,   # LB
                'turbo_button': 5,    # RB
                'toggle_button': 3,   # Y button
                # Husky mapping/scales
                'husky_axis_linear_x': 1,
                'husky_axis_angular_z': 3,
                'husky_scale_linear': 2.0,
                'husky_scale_angular': 4.0,
                'husky_scale_linear_turbo': 4.0,
                'husky_scale_angular_turbo': 8.0,
                # Drone mapping/scales
                'drone_axis_linear_x': 1,
                'drone_axis_linear_y': 0,
                'drone_axis_linear_z': -1,
                'drone_axis_angular_z': 3,
                'drone_button_altitude_up': 0,   # A
                'drone_button_altitude_down': 2, # X
                'drone_scale_linear_xy': 1.5,
                'drone_scale_linear_z': 1.0,
                'drone_scale_angular_z': 2.0,
                'drone_scale_linear_xy_turbo': 3.0,
                'drone_scale_linear_z_turbo': 2.0,
                'drone_scale_angular_z_turbo': 4.0,
                # Hover settings to prevent drone from falling
                'drone_hover_when_disabled': True,
                'drone_hover_linear_z': 0.1,
            }]
        )
    ])


