#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_dev = LaunchConfiguration('joy_dev')

    # Camera topics (can be overridden)
    drone_rgb = LaunchConfiguration('drone_rgb_topic')
    drone_depth = LaunchConfiguration('drone_depth_topic')
    husky_rgb = LaunchConfiguration('husky_rgb_topic')
    husky_depth = LaunchConfiguration('husky_depth_topic')
    teleop_status = LaunchConfiguration('teleop_status_topic')

    pkg_share = get_package_share_directory('41068_ignition_bringup')
    gui_script = os.path.join(pkg_share, 'scripts', 'steamdeck_gui.py')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),

        # Default topics (override per your system)
        DeclareLaunchArgument('drone_rgb_topic', default_value='/drone/camera/image'),
        DeclareLaunchArgument('drone_depth_topic', default_value='/drone/camera/depth/image'),
        DeclareLaunchArgument('husky_rgb_topic', default_value='/husky/camera/image'),
        DeclareLaunchArgument('husky_depth_topic', default_value='/husky/camera/depth/image'),
        DeclareLaunchArgument('teleop_status_topic', default_value='/teleop_status'),

        # Include combined joystick teleop from subsystems package
        # Controls: LB=Drone, RB=Husky, Y=Toggle, A/X=Altitude
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('drone_teleop'),
                'launch',
                'combined_joy_teleop.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'joy_dev': joy_dev,
            }.items()
        ),

        # GUI process (run via python3 to avoid executable path issues)
        ExecuteProcess(
            cmd=[
                'python3', gui_script,
                '--ros-args',
                '-p', ['drone_rgb_topic:=', drone_rgb],
                '-p', ['drone_depth_topic:=', drone_depth],
                '-p', ['husky_rgb_topic:=', husky_rgb],
                '-p', ['husky_depth_topic:=', husky_depth],
                '-p', ['teleop_status_topic:=', teleop_status],
            ],
            output='screen'
        ),
    ])


