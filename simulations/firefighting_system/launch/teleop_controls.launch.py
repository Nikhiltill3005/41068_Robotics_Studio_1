#!/usr/bin/env python3

"""
Teleop Controls Launch File

This launch file starts teleop control nodes for both Husky and Drone
in separate terminals.

Usage:
    ros2 launch firefighting_system teleop_controls.launch.py
    ros2 launch firefighting_system teleop_controls.launch.py drone_only:=true
    ros2 launch firefighting_system teleop_controls.launch.py husky_only:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    drone_only_arg = DeclareLaunchArgument(
        'drone_only',
        default_value='false',
        description='Launch only drone teleop'
    )
    
    husky_only_arg = DeclareLaunchArgument(
        'husky_only',
        default_value='false',
        description='Launch only husky teleop'
    )
    
    # Husky Teleop - in new terminal
    husky_teleop = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=Husky Control', '--', 'bash', '-c',
            'echo "🤖 HUSKY TELEOP CONTROL"; '
            'echo "Use WASD keys to control Husky ground robot"; '
            'echo "Press Ctrl+C to stop"; echo ""; '
            'source /home/odyssey/ros2_ws/install/setup.bash && '
            'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
            '--ros-args --remap cmd_vel:=/husky_velocity_controller/cmd_vel_unstamped; '
            'exec bash'
        ],
        condition=UnlessCondition(LaunchConfiguration('drone_only')),
        output='screen'
    )
    
    # Drone Teleop - in new terminal
    drone_teleop = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=Drone Control', '--', 'bash', '-c',
            'echo "🚁 DRONE TELEOP CONTROL"; '
            'echo "Use WASD keys to control firefighting drone"; '
            'echo "Press Ctrl+C to stop"; echo ""; '
            'source /home/odyssey/ros2_ws/install/setup.bash && '
            'ros2 run firefighting_system drone_teleop_ros.py; '
            'exec bash'
        ],
        condition=UnlessCondition(LaunchConfiguration('husky_only')),
        output='screen'
    )
    
    return LaunchDescription([
        drone_only_arg,
        husky_only_arg,
        husky_teleop,
        drone_teleop
    ])
