from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'world',
            default_value='simple_trees',
            description='Which world to load',
            choices=['simple_trees', 'large_demo', 'bushland']
        ),
        
        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Flag to launch RViz'
        ),
        
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),

        # Launch the main simulation (Husky only, not combined with drone)
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('41068_ignition_bringup'),
                'launch',
                '41068_ignition.launch.py'
            ]),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'rviz': LaunchConfiguration('rviz'),
                'use_sim_time': 'True'
            }.items()
        ),

        # Launch the joy teleop
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('drone_teleop'),  # The package containing our joy teleop
                'launch',
                'husky_joy_teleop.launch.py'
            ]),
            launch_arguments={
                'joy_dev': LaunchConfiguration('joy_dev')
            }.items()
        )
    ])
