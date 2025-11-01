from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch argument for joy device
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device'
        ),
        
        # Launch argument for deadman button
        DeclareLaunchArgument(
            'deadman_button',
            default_value='4',
            description='Deadman button index (LB = 4 for Xbox controller)'
        ),
        
        # Launch argument for turbo button
        DeclareLaunchArgument(
            'turbo_button',
            default_value='5',
            description='Turbo button index (RB = 5 for Xbox controller)'
        ),

        # Joy node to read Xbox controller input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device': LaunchConfiguration('joy_dev'),
                'deadzone': 0.12,
                'autorepeat_rate': 20.0,
            }]
        ),

        # Husky joy teleop node
        Node(
            package='drone_teleop',  # Using the existing package name
            executable='husky_joy_teleop',
            name='husky_joy_teleop',
            parameters=[{
                'axis_linear_x': 1,      # Left stick Y-axis (forward/backward thrust)
                'axis_angular_z': 3,     # Right stick X-axis (left/right steering)
                'enable_button': LaunchConfiguration('deadman_button'),  # LB button
                'turbo_button': LaunchConfiguration('turbo_button'),     # RB button
                'scale_linear': 2.0,         # Normal linear velocity scaling
                'scale_angular': 4.0,        # Normal angular velocity scaling  
                'scale_linear_turbo': 4.0,   # Turbo linear velocity scaling
                'scale_angular_turbo': 8.0,  # Turbo angular velocity scaling
            }],
            output='screen'
        )
    ])
