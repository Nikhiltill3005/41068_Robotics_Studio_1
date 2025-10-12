"""
Navigation Goal GUI Launch File

This launch file starts the Navigation Goal GUI for the Husky robot.
The GUI allows setting navigation goals using X,Y coordinates and orientation.

Usage:
    ros2 launch 41068_ignition_bringup nav_goal_gui.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Navigation Goal GUI node
    nav_gui_node = Node(
        package='41068_ignition_bringup',
        executable='nav_goal_gui.py',
        name='nav_goal_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        nav_gui_node
    ])
