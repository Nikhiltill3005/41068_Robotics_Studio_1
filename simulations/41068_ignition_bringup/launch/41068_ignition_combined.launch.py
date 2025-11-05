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
        choices=['simple_trees', 'large_demo', 'bushland', 'bushland_spaced']
    )
    ld.add_action(world_launch_arg)

    # Declare drone_joy argument early so it can be used in conditions
    drone_joy_launch_arg = DeclareLaunchArgument(
        'drone_joy',
        default_value='True',
        description='Flag to launch combined joystick teleop (RB=Husky, LB=Drone)'
    )
    ld.add_action(drone_joy_launch_arg)
    drone_joy = LaunchConfiguration('drone_joy')

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
            arguments=['-topic', 'robot_description', '-z', '14.0', '-x', '2.0', '-y', '0.0', '-name', 'drone']
        ),
        # Optional: Combined joystick teleop for both vehicles
        GroupAction([
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time, 
                    'autorepeat_rate': 20.0,
                    'deadzone': 0.12
                }]
            ),
            Node(
                package='drone_teleop',
                executable='combined_joy_teleop',
                name='combined_joy_teleop',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    # Common
                    'enable_button': 4,   # LB for drone
                    'turbo_button': 5,    # RB for huskyx``
                    'toggle_button': 3,   # Y button
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
                    'drone_button_altitude_up': 0,   # A
                    'drone_button_altitude_down': 2, # X
                    'drone_scale_linear_xy': 1.5,
                    'drone_scale_linear_z': 1.0,
                    'drone_scale_angular_z': 2.0,
                    'drone_scale_linear_xy_turbo': 3.0,
                    'drone_scale_linear_z_turbo': 2.0,
                    'drone_scale_angular_z_turbo': 4.0,
                    # Hover settings to prevent drone from falling
                    'drone_hover_when_disabled': True,
                    'drone_hover_linear_z': 0.1,
                }]
            )
        ], condition=IfCondition(drone_joy))
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
        default_value='True',
        description='Flag to launch Nav2 for Husky only'
    )
    ld.add_action(nav2_launch_arg)

    fire_search_launch_arg = DeclareLaunchArgument(
        'fire_search',
        default_value='False',
        description='Flag to launch drone fire search pattern'
    )
    ld.add_action(fire_search_launch_arg)

    # RViz (global)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([pkg_path, 'config', '41068.rviz'])],
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

    # Fire Search Pattern (drone namespace)
    fire_search = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('drone_teleop'), 'launch', 'fire_centering.launch.py']),
        condition=IfCondition(LaunchConfiguration('fire_search'))
    )
    ld.add_action(fire_search)

    # Image compression republishers for efficient network transmission
    # Husky RGB camera
    husky_rgb_compress = Node(
        package='image_transport',
        executable='republish',
        name='husky_rgb_compress',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/husky/camera/image'),
            ('out/compressed', '/husky/camera/image/compressed')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(husky_rgb_compress)

    # Drone RGB camera
    drone_rgb_compress = Node(
        package='image_transport',
        executable='republish',
        name='drone_rgb_compress',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/drone/camera/image'),
            ('out/compressed', '/drone/camera/image/compressed')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(drone_rgb_compress)

    # Drone IR camera
    drone_ir_compress = Node(
        package='image_transport',
        executable='republish',
        name='drone_ir_compress',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/drone/ir_camera/image_raw'),
            ('out/compressed', '/drone/ir_camera/image_raw/compressed')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(drone_ir_compress)

    # Fire scan debug image compression (for Steam Deck video streaming)
    # This is critical for network bandwidth over travel routers
    fire_scan_debug_compress = Node(
        package='image_transport',
        executable='republish',
        name='fire_scan_debug_compress',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/drone/fire_scan/debug_image'),
            ('out/compressed', '/drone/fire_scan/debug_image/compressed')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(fire_scan_debug_compress)

    return ld


