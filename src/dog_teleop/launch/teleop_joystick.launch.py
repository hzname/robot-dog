"""
Launch file for joystick teleoperation.

Usage:
    ros2 launch dog_teleop teleop_joystick.launch.py
    
Prerequisites:
    - Connect gamepad/joystick
    - Run joy_node: ros2 run joy joy_node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch joystick teleoperation for robot dog."""
    
    # Declare arguments
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='auto',
        description='Controller type: auto, xbox, ps4, or generic'
    )
    
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
    
    deadzone_arg = DeclareLaunchArgument(
        'deadzone',
        default_value='0.15',
        description='Joystick deadzone (0.0 - 0.5)'
    )
    
    # Joy node (reads raw joystick input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )
    
    # Teleop joystick node
    teleop_node = Node(
        package='dog_teleop',
        executable='teleop_joystick',
        name='teleop_joystick',
        output='screen',
        parameters=[{
            'controller_type': LaunchConfiguration('controller_type'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'deadzone': LaunchConfiguration('deadzone'),
            'smoothing_factor': 0.2,
            'axis_linear_x': 1,   # Left stick Y
            'axis_linear_y': 0,   # Left stick X
            'axis_angular_z': 3,  # Right stick X
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ],
    )
    
    return LaunchDescription([
        controller_type_arg,
        linear_speed_arg,
        angular_speed_arg,
        deadzone_arg,
        joy_node,
        teleop_node,
    ])
