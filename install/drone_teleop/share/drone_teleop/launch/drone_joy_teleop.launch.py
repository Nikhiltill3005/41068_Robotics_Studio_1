#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Joystick driver (joy node)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autorepeat_rate': 20.0
            }]
        ),

        # Drone joystick teleop
        Node(
            package='drone_teleop',
            executable='drone_joy_teleop',
            name='drone_joy_teleop',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'axis_linear_x': 1,
                'axis_linear_y': 0,
                'axis_linear_z': 4,
                'axis_angular_z': 3,
                'enable_button': 4,   # LB
                'turbo_button': 5,    # RB
                'scale_linear_xy': 1.5,
                'scale_linear_z': 1.0,
                'scale_angular_z': 2.0,
                'scale_linear_xy_turbo': 3.0,
                'scale_linear_z_turbo': 2.0,
                'scale_angular_z_turbo': 4.0
            }]
        )
    ])
