#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_dev = LaunchConfiguration('joy_dev')

    # Topics (can be overridden)
    husky_rgb = LaunchConfiguration('husky_rgb_topic')
    drone_rgb = LaunchConfiguration('drone_rgb_topic')
    husky_map = LaunchConfiguration('husky_map_topic')
    husky_odom = LaunchConfiguration('husky_odom_topic')
    drone_odom = LaunchConfiguration('drone_odom_topic')
    fire_topic = LaunchConfiguration('fire_topic')
    teleop_status = LaunchConfiguration('teleop_status_topic')
    joy_topic = LaunchConfiguration('joy_topic')
    world_size = LaunchConfiguration('world_size')
    terrain_image = LaunchConfiguration('terrain_image')


    pkg_share = get_package_share_directory('41068_ignition_bringup')
    gui_script = os.path.join(pkg_share, 'scripts', 'steamdeck_gui.py')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),

        # Default topics (override per your system)
        DeclareLaunchArgument('husky_rgb_topic', default_value='/husky/camera/image'),
        DeclareLaunchArgument('drone_rgb_topic', default_value='/drone/camera/image'),
        DeclareLaunchArgument('husky_map_topic', default_value='/map'),
        DeclareLaunchArgument('husky_odom_topic', default_value='/husky/odometry'),
        DeclareLaunchArgument('drone_odom_topic', default_value='/drone/odometry'),
        DeclareLaunchArgument('fire_topic', default_value='/drone/fire_scan/fire_positions'),
        DeclareLaunchArgument('teleop_status_topic', default_value='/teleop_status'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('world_size', default_value='50.0'),
        DeclareLaunchArgument('terrain_image', default_value='bushland_terrain.png'),

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

        # GUI process (run via python3 to avoid executable path issues)
        ExecuteProcess(
            cmd=[
                'python3', gui_script,
                '--ros-args',
                '-p', ['husky_rgb_topic:=', husky_rgb],
                '-p', ['drone_rgb_topic:=', drone_rgb],
                '-p', ['husky_map_topic:=', husky_map],
                '-p', ['husky_odom_topic:=', husky_odom],
                '-p', ['drone_odom_topic:=', drone_odom],
                '-p', ['fire_topic:=', fire_topic],
                '-p', ['teleop_status_topic:=', teleop_status],
                '-p', ['joy_topic:=', joy_topic],
                '-p', ['world_size:=', world_size],
                '-p', ['terrain_image:=', terrain_image],
            ],
            output='screen'
        ),
    ])


