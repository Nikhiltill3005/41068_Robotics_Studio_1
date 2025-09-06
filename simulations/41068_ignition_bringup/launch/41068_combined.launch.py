from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)
    
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)
    
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    # ===== HUSKY ROBOT =====
    # Load Husky robot_description and start robot_state_publisher
    husky_robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf',
                                       'husky.urdf.xacro'])]),
        value_type=str)
    husky_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='husky_robot_state_publisher',
        namespace='husky',
        parameters=[{
            'robot_description': husky_robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(husky_robot_state_publisher_node)

    # Husky robot localization
    husky_robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='husky_robot_localization',
        namespace='husky',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization_combined.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(husky_robot_localization_node)
    
    # Add event handler for husky robot_localization node
    husky_robot_localization_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=husky_robot_localization_node,
            on_exit=[Shutdown()]
        )
    )
    ld.add_action(husky_robot_localization_exit_handler)

    # ===== DRONE ROBOT =====
    # Load Drone robot_description and start robot_state_publisher
    drone_robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf_drone',
                                       'parrot.urdf.xacro'])]),
        value_type=str)
    drone_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='drone_robot_state_publisher',
        namespace='drone',
        parameters=[{
            'robot_description': drone_robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(drone_robot_state_publisher_node)

    # Drone robot localization
    drone_robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='drone_robot_localization',
        namespace='drone',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization_combined.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(drone_robot_localization_node)
    
    # Add event handler for drone robot_localization node
    drone_robot_localization_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=drone_robot_localization_node,
            on_exit=[Shutdown()]
        )
    )
    ld.add_action(drone_robot_localization_exit_handler)

    # ===== GAZEBO SIMULATION =====
    # Start Gazebo to simulate both robots in the chosen world
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    ld.add_action(world_launch_arg)
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

    # Spawn Husky robot in Gazebo
    husky_robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='husky_robot_spawner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/husky/robot_description', '-z', '0.4', '-name', 'husky']
    )
    ld.add_action(husky_robot_spawner)

    # Spawn Drone robot in Gazebo
    drone_robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='drone_robot_spawner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/drone/robot_description', '-z', '2.0', '-name', 'drone']
    )
    ld.add_action(drone_robot_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge_combined.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)
    
    # Add event handler to ensure parameter_bridge terminates properly
    gazebo_bridge_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_bridge,
            on_exit=[Shutdown()]
        )
    )
    ld.add_action(gazebo_bridge_exit_handler)

    # ===== VISUALIZATION =====
    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # ===== NAVIGATION =====
    # SLAM for Husky
    husky_slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                             'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_path, 'slam_params_combined.yaml']),
            'namespace': 'husky'
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(husky_slam)

    # Navigation for Husky
    husky_navigation = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'nav2_params_combined.yaml']),
            'namespace': 'husky'
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(husky_navigation)

    # SLAM for Drone
    drone_slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                             'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_path, 'slam_params_combined.yaml']),
            'namespace': 'drone'
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(drone_slam)

    # Navigation for Drone
    drone_navigation = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'nav2_params_combined.yaml']),
            'namespace': 'drone'
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(drone_navigation)

    # Add cleanup mechanism to kill any remaining ros_parameters processes
    cleanup_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['pkill', '-f', 'ros_parameters'],
                    output='screen'
                )
            ]
        )
    )
    ld.add_action(cleanup_handler)

    return ld
