"""
Complete teleoperation launch file for robot dog.

Launches all teleoperation methods (keyboard, joystick, UDP) simultaneously.

Usage:
    # Launch all teleop methods
    ros2 launch dog_teleop teleop_all.launch.py
    
    # Launch with specific mode only
    ros2 launch dog_teleop teleop_all.launch.py mode:=keyboard
    ros2 launch dog_teleop teleop_all.launch.py mode:=joystick
    ros2 launch dog_teleop teleop_all.launch.py mode:=udp
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Launch teleoperation for robot dog."""
    
    # Declare arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='all',
        description='Teleop mode: all, keyboard, joystick, or udp'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Default linear speed scale'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.8',
        description='Default angular speed scale'
    )
    
    udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='8888',
        description='UDP port for remote teleop'
    )
    
    # Conditions for different modes
    mode = LaunchConfiguration('mode')
    use_keyboard = PythonExpression(["'", mode, "' in ['all', 'keyboard']"])
    use_joystick = PythonExpression(["'", mode, "' in ['all', 'joystick']"])
    use_udp = PythonExpression(["'", mode, "' in ['all', 'udp']"])
    
    # Keyboard teleop
    keyboard_node = Node(
        condition=IfCondition(use_keyboard),
        package='dog_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
        }],
    )
    
    # Joy node (for joystick)
    joy_node = Node(
        condition=IfCondition(use_joystick),
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )
    
    # Joystick teleop
    joystick_node = Node(
        condition=IfCondition(use_joystick),
        package='dog_teleop',
        executable='teleop_joystick',
        name='teleop_joystick',
        output='screen',
        parameters=[{
            'controller_type': 'auto',
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'deadzone': 0.15,
        }],
    )
    
    # UDP server
    udp_node = Node(
        condition=IfCondition(use_udp),
        package='dog_teleop',
        executable='udp_server',
        name='teleop_udp_server',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('udp_port'),
            'max_clients': 4,
            'timeout_sec': 2.0,
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
        }],
    )
    
    return LaunchDescription([
        mode_arg,
        linear_speed_arg,
        angular_speed_arg,
        udp_port_arg,
        keyboard_node,
        joy_node,
        joystick_node,
        udp_node,
    ])
