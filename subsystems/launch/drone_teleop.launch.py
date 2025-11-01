#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Launch the drone teleop node
        Node(
            package='drone_teleop',
            executable='drone_teleop',
            name='drone_teleop',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            remappings=[
                # Publish to bridge-expected topic name
                ('/drone/cmd_vel', 'cmd_vel'),
                # Subscribe from bridge-expected IMU topic name
                ('/drone/imu', 'imu'),
                ('/drone/teleop_status', '/drone/teleop_status')
            ]
        )
    ])

