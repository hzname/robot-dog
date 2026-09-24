"""Launch file for all teleoperation modes."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Arguments
    enable_keyboard_arg = DeclareLaunchArgument(
        'enable_keyboard',
        default_value='true',
        description='Enable keyboard teleop'
    )
    
    enable_joystick_arg = DeclareLaunchArgument(
        'enable_joystick',
        default_value='false',
        description='Enable joystick teleop'
    )
    
    enable_udp_arg = DeclareLaunchArgument(
        'enable_udp',
        default_value='false',
        description='Enable UDP teleop'
    )
    
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.5',
        description='Maximum linear speed (m/s)'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='2.0',
        description='Maximum angular speed (rad/s)'
    )

    # Launch files
    launch_dir = os.path.join(
        FindPackageShare('dog_teleop_cpp').find('dog_teleop_cpp'),
        'launch'
    )

    # Include keyboard teleop
    keyboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'teleop_keyboard.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_keyboard')),
        launch_arguments={
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
        }.items()
    )

    # Include joystick teleop
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'teleop_joystick.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_joystick')),
        launch_arguments={
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
        }.items()
    )

    # Include UDP teleop
    udp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'teleop_udp.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_udp')),
        launch_arguments={
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
        }.items()
    )

    return LaunchDescription([
        enable_keyboard_arg,
        enable_joystick_arg,
        enable_udp_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        keyboard_launch,
        joystick_launch,
        udp_launch,
    ])
