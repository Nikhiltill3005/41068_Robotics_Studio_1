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
        
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='1.0',
            description='Maximum linear speed for drone movement (m/s)'
        ),
        
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='1.0',
            description='Maximum angular speed for drone movement (rad/s)'
        ),
        
        DeclareLaunchArgument(
            'deadzone_pixels',
            default_value='20',
            description='Pixel radius around center considered as centered'
        ),
        
        # Fire Detection Node
        Node(
            package='41068_ignition_bringup',
            executable='fire_detection_node.py',
            name='fire_detection_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            remappings=[
                ('/drone/ir_camera/image_raw', 'ir_camera/image_raw'),
                ('/drone/fire_detection/location', '/drone/fire_detection/location'),
                ('/drone/fire_detection/debug_image', '/drone/fire_detection/debug_image')
            ]
        ),
        
        # Drone Fire Centering Node
        Node(
            package='41068_ignition_bringup',
            executable='drone_fire_centering_node.py',
            name='drone_fire_centering_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'deadzone_pixels': LaunchConfiguration('deadzone_pixels')
            }],
            remappings=[
                ('/drone/fire_detection/location', '/drone/fire_detection/location'),
                ('/drone/cmd_vel', 'cmd_vel'),
                ('/drone/ir_camera/camera_info', 'ir_camera/camera_info')
            ]
        )
    ])

