from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    search_altitude_arg = DeclareLaunchArgument(
        'search_altitude',
        default_value='3.0',
        description='Altitude to maintain during search (meters)'
    )
    
    search_speed_arg = DeclareLaunchArgument(
        'search_speed',
        default_value='1.0',
        description='Speed during search pattern (m/s)'
    )
    
    pattern_width_arg = DeclareLaunchArgument(
        'pattern_width',
        default_value='10.0',
        description='Width of search area (meters)'
    )
    
    pattern_length_arg = DeclareLaunchArgument(
        'pattern_length',
        default_value='15.0',
        description='Length of search area (meters)'
    )
    
    lane_spacing_arg = DeclareLaunchArgument(
        'lane_spacing',
        default_value='2.0',
        description='Distance between search lanes (meters)'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Start search pattern automatically'
    )

    # Drone search pattern node
    drone_search_node = Node(
        package='drone_teleop',
        executable='drone_search_pattern',
        name='drone_search_pattern',
        namespace='drone',
        output='screen',
        parameters=[{
            'search_altitude': LaunchConfiguration('search_altitude'),
            'search_speed': LaunchConfiguration('search_speed'),
            'pattern_width': LaunchConfiguration('pattern_width'),
            'pattern_length': LaunchConfiguration('pattern_length'),
            'lane_spacing': LaunchConfiguration('lane_spacing'),
            'waypoint_tolerance': 0.5,
            'fire_investigation_time': 10.0,
            'fire_hover_altitude': 2.0,
            'auto_start': LaunchConfiguration('auto_start'),
        }]
    )

    return LaunchDescription([
        search_altitude_arg,
        search_speed_arg,
        pattern_width_arg,
        pattern_length_arg,
        lane_spacing_arg,
        auto_start_arg,
        drone_search_node
    ])
