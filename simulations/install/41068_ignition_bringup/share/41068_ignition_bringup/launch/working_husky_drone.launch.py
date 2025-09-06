from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Working launch file for both Husky and simple drone with teleop."""

    ld = LaunchDescription()

    # Get paths to directories
    ignition_pkg = FindPackageShare('41068_ignition_bringup')
    drone_pkg = FindPackageShare('custom_drone')

    # Launch arguments
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
        default_value='False',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo', 'bushland']
    )
    ld.add_action(world_launch_arg)

    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='false',
        description='Launch drone teleop for keyboard control'
    )
    ld.add_action(teleop_arg)

    drone_x_arg = DeclareLaunchArgument(
        'drone_x',
        default_value='4.0',
        description='Drone spawn X position'
    )
    ld.add_action(drone_x_arg)

    drone_y_arg = DeclareLaunchArgument(
        'drone_y',
        default_value='2.0',
        description='Drone spawn Y position'
    )
    ld.add_action(drone_y_arg)

    drone_z_arg = DeclareLaunchArgument(
        'drone_z',
        default_value='2.5',
        description='Drone spawn Z position'
    )
    ld.add_action(drone_z_arg)

    # === START EXISTING HUSKY SIMULATION ===
    
    # Include the existing Husky launch file
    husky_launch = IncludeLaunchDescription(
        PathJoinSubstitution([ignition_pkg, 'launch', '41068_ignition.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'nav2': LaunchConfiguration('nav2'),
        }.items()
    )
    ld.add_action(husky_launch)

    # === ADD WORKING SIMPLE DRONE ===
    
    # Simple drone robot description
    drone_robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([drone_pkg,
                                       'urdf',
                                       'simple_drone.urdf.xacro'])]),
        value_type=str)
    
    # Drone robot state publisher
    drone_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='drone_robot_state_publisher',
        namespace='drone',
        parameters=[{
            'robot_description': drone_robot_description_content,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    ld.add_action(drone_robot_state_publisher)

    # Spawn simple drone in Gazebo (single authoritative drone entity)
    spawn_drone = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_simple_drone',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-topic', '/drone/robot_description',
            '-name', 'simple_drone',
            '-x', LaunchConfiguration('drone_x'),
            '-y', LaunchConfiguration('drone_y'),
            '-z', LaunchConfiguration('drone_z')
        ]
    )
    ld.add_action(spawn_drone)

    # Drone controller node (software control)
    drone_controller_node = Node(
        package='custom_drone',
        executable='drone_controller',
        name='drone_controller',
        namespace='drone',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(drone_controller_node)

    # Optional drone teleop node for keyboard control
    drone_teleop_node = Node(
        package='custom_drone',
        executable='drone_teleop',
        name='drone_teleop',
        parameters=[{
            'use_sim_time': use_sim_time,
            'linear_speed': 1.0,
            'angular_speed': 1.0,
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('teleop'))
    )
    ld.add_action(drone_teleop_node)

    return ld
