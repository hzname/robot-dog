"""
Launch file for keyboard teleoperation.

Usage:
    ros2 launch dog_teleop teleop_keyboard.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch keyboard teleoperation for robot dog."""
    
    # Declare arguments
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Default linear speed scale (0.1 - 1.0)'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.8',
        description='Default angular speed scale (0.1 - 1.0)'
    )
    
    smoothing_arg = DeclareLaunchArgument(
        'smoothing_factor',
        default_value='0.2',
        description='Velocity smoothing factor (0.0 - 1.0)'
    )
    
    # Teleop keyboard node
    teleop_node = Node(
        package='dog_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Open in separate terminal
        parameters=[{
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'smoothing_factor': LaunchConfiguration('smoothing_factor'),
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ],
    )
    
    return LaunchDescription([
        linear_speed_arg,
        angular_speed_arg,
        smoothing_arg,
        teleop_node,
    ])
