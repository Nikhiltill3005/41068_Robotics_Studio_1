#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_dev = LaunchConfiguration('joy_dev')

    # Camera topics (can be overridden)
    drone_rgb = LaunchConfiguration('drone_rgb_topic')
    drone_depth = LaunchConfiguration('drone_depth_topic')
    husky_rgb = LaunchConfiguration('husky_rgb_topic')
    husky_depth = LaunchConfiguration('husky_depth_topic')
    teleop_status = LaunchConfiguration('teleop_status_topic')


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),

        # Default topics (override per your system)
        DeclareLaunchArgument('drone_rgb_topic', default_value='/drone/camera/rgb/image_raw'),
        DeclareLaunchArgument('drone_depth_topic', default_value='/drone/camera/depth/image_raw'),
        DeclareLaunchArgument('husky_rgb_topic', default_value='/husky/camera/rgb/image_raw'),
        DeclareLaunchArgument('husky_depth_topic', default_value='/husky/camera/depth/image_raw'),
        DeclareLaunchArgument('teleop_status_topic', default_value='/teleop_status'),

        # Joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'device': joy_dev,
                'autorepeat_rate': 20.0,
                'deadzone': 0.12,
            }]
        ),

        # Combined teleop node (publishes to /husky/cmd_vel and /drone/cmd_vel)
        Node(
            package='drone_teleop',
            executable='combined_joy_teleop',
            name='combined_joy_teleop',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                # Common
                'enable_button': 4,
                'turbo_button': 5,
                'toggle_button': 3,
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
                'drone_button_altitude_up': 0,
                'drone_button_altitude_down': 2,
                'drone_scale_linear_xy': 1.5,
                'drone_scale_linear_z': 1.0,
                'drone_scale_angular_z': 2.0,
                'drone_scale_linear_xy_turbo': 3.0,
                'drone_scale_linear_z_turbo': 2.0,
                'drone_scale_angular_z_turbo': 4.0,
                'drone_hover_when_disabled': True,
                'drone_hover_linear_z': 0.1,
            }]
        ),

        # GUI node (installed via this package)
        Node(
            package='41068_ignition_bringup',
            executable='steamdeck_gui.py',
            name='steamdeck_gui',
            output='screen',
            parameters=[{
                'drone_rgb_topic': drone_rgb,
                'drone_depth_topic': drone_depth,
                'husky_rgb_topic': husky_rgb,
                'husky_depth_topic': husky_depth,
                'teleop_status_topic': teleop_status,
            }]
        ),
    ])


