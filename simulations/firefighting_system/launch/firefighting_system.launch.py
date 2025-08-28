#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package and directory setup
    pkg_firefighting = FindPackageShare('firefighting_system')
    pkg_ignition = FindPackageShare('41068_ignition_bringup')
    
    # Launch arguments
    use_sim = LaunchConfiguration('use_sim', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_slam = LaunchConfiguration('use_slam', default='true')
    use_nav2 = LaunchConfiguration('use_nav2', default='true')
    world = LaunchConfiguration('world', default='simple_trees')
    
    # Declare launch arguments
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (Gazebo)'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Use RViz for visualization'
    )
    
    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Use SLAM for mapping'
    )
    
    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Use Navigation2 for autonomous navigation'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='World file to use for simulation'
    )
    
    # Include the Ignition simulation launch
    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ignition, 'launch', '41068_ignition.launch.py'])
        ]),
        launch_arguments={
            'world': world,
            'slam': use_slam,
            'nav2': use_nav2,
            # Disable RViz in the ignition launch; we'll start our own RViz below
            'rviz': 'false',
        }.items(),
        condition=IfCondition(use_sim)
    )
    
    # Firefighting system nodes
    teleop_node = Node(
        package='firefighting_system',
        executable='teleop_control_node',
        name='teleop_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'max_steering_angle': 0.5,
            'max_speed': 2.0,
            'steering_sensitivity': 0.1,
            'speed_sensitivity': 0.5,
        }]
    )

    # RViz node with camera view
    rviz_config = PathJoinSubstitution([pkg_firefighting, 'config', 'firefighting_view.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )


    
    ugv_movement_node = Node(
        package='firefighting_system',
        executable='ugv_movement_node',
        name='ugv_movement_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wheelbase': 0.5,
            'max_steering_angle': 0.5,
            'max_speed': 2.0,
        }]
    )
    
    environment_mapping_node = Node(
        package='firefighting_system',
        executable='environment_mapping_node',
        name='environment_mapping_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_resolution': 0.05,
            'map_width': 1000,
            'map_height': 1000,
            'map_origin_x': -25.0,
            'map_origin_y': -25.0,
        }]
    )
    
    autonomous_navigation_node = Node(
        package='firefighting_system',
        executable='autonomous_navigation_node',
        name='autonomous_navigation_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'goal_tolerance': 0.1,
            'max_linear_velocity': 1.0,
            'max_angular_velocity': 1.0,
            'path_following_gain': 1.0,
        }]
    )
    

    
    # Keyboard teleop for testing
    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -e',
        arguments=['--ros-args', '--remap', 'cmd_vel:=/teleop_commands']
    )
    
    # Create launch description
    return LaunchDescription([
        declare_use_sim,
        declare_use_rviz,
        declare_use_slam,
        declare_use_nav2,
        declare_world,
        
        # Include simulation if requested
        ignition_launch,
        
        # Firefighting system nodes
        teleop_node,
        ugv_movement_node,
        environment_mapping_node,
        autonomous_navigation_node,
        
        # Optional keyboard teleop for testing
        keyboard_teleop,
        
        # RViz
        rviz_node,
    ])
