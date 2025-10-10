#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch the autonomous firefighter node"""

    # Declare launch arguments
    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='1.8',
        description='Target distance from fire (meters) - increased for collision safety'
    )

    distance_tolerance_arg = DeclareLaunchArgument(
        'distance_tolerance',
        default_value='0.4',
        description='Position tolerance (meters)'
    )

    extinguish_duration_arg = DeclareLaunchArgument(
        'extinguish_duration',
        default_value='3.0',
        description='Duration to extinguish each fire (seconds)'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Automatically start firefighting when fires detected'
    )

    stuck_timeout_arg = DeclareLaunchArgument(
        'stuck_timeout',
        default_value='8.0',
        description='Seconds stuck before giving up (seconds)'
    )

    stuck_velocity_threshold_arg = DeclareLaunchArgument(
        'stuck_velocity_threshold',
        default_value='0.05',
        description='Velocity threshold for stuck detection (m/s)'
    )

    fallback_distance_arg = DeclareLaunchArgument(
        'fallback_distance',
        default_value='2.5',
        description='Distance to fire to consider close enough (meters)'
    )

    # Firefighter node
    firefighter_node = Node(
        package='41068_ignition_bringup',
        executable='autonomous_firefighter_node.py',
        name='autonomous_firefighter',
        output='screen',
        parameters=[{
            'target_distance': LaunchConfiguration('target_distance'),
            'distance_tolerance': LaunchConfiguration('distance_tolerance'),
            'extinguish_duration': LaunchConfiguration('extinguish_duration'),
            'auto_start': LaunchConfiguration('auto_start'),
            'stuck_timeout': LaunchConfiguration('stuck_timeout'),
            'stuck_velocity_threshold': LaunchConfiguration('stuck_velocity_threshold'),
            'fallback_distance': LaunchConfiguration('fallback_distance'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        target_distance_arg,
        distance_tolerance_arg,
        extinguish_duration_arg,
        auto_start_arg,
        stuck_timeout_arg,
        stuck_velocity_threshold_arg,
        fallback_distance_arg,
        firefighter_node,
    ])
