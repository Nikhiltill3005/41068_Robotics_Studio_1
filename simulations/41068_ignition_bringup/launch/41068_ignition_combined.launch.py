from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo', 'bushland']
    )
    ld.add_action(world_launch_arg)

    # Start Gazebo once
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    # Husky group (namespace: husky)
    husky_description = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf',
                                       'husky.urdf.xacro'])]),
        value_type=str)

    husky_group = GroupAction([
        PushRosNamespace('husky'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': husky_description,
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': PathJoinSubstitution([pkg_path, 'config', 'gazebo_bridge_husky.yaml']),
                        'use_sim_time': use_sim_time}]
        ),
        Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            # spawn slightly above ground to settle
            arguments=['-topic', 'robot_description', '-z', '0.4', '-name', 'husky']
        )
    ])
    ld.add_action(husky_group)

    # Drone group (namespace: drone)
    drone_description = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf_drone',
                                       'parrot.urdf.xacro'])]),
        value_type=str)

    drone_group = GroupAction([
        PushRosNamespace('drone'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': drone_description,
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': PathJoinSubstitution([pkg_path, 'config', 'gazebo_bridge_drone.yaml']),
                        'use_sim_time': use_sim_time}]
        ),
        Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            # spawn in the air with an XY offset from Husky
            arguments=['-topic', 'robot_description', '-z', '2.0', '-x', '2.0', '-y', '0.0', '-name', 'drone']
        ),
        # Optional: Drone joystick teleop and joy driver
        GroupAction([
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'autorepeat_rate': 20.0}]
            ),
            Node(
                package='drone_teleop',
                executable='drone_joy_teleop',
                name='drone_joy_teleop',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ], condition=IfCondition(LaunchConfiguration('drone_joy')))
    ])
    ld.add_action(drone_group)

    # Optional toggles
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='False',
        description='Flag to launch Nav2 for Husky only'
    )
    ld.add_action(nav2_launch_arg)

    drone_joy_launch_arg = DeclareLaunchArgument(
        'drone_joy',
        default_value='True',
        description='Flag to launch drone joystick teleop (Xbox)'
    )
    ld.add_action(drone_joy_launch_arg)

    # RViz (global)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([pkg_path, 'config', '41068_ignition_combined.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 (husky namespace)
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(GroupAction([PushRosNamespace('husky'), nav2]))

    return ld


