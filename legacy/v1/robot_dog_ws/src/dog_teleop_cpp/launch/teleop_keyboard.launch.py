"""Launch file for keyboard teleoperation."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Arguments
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
    
    max_lateral_speed_arg = DeclareLaunchArgument(
        'max_lateral_speed',
        default_value='0.3',
        description='Maximum lateral speed (m/s)'
    )
    
    watchdog_timeout_arg = DeclareLaunchArgument(
        'watchdog_timeout',
        default_value='0.5',
        description='Watchdog timeout in seconds'
    )

    # Config file
    config_file = os.path.join(
        FindPackageShare('dog_teleop_cpp').find('dog_teleop_cpp'),
        'config',
        'teleop_config.yaml'
    )

    # Keyboard teleop node
    keyboard_node = Node(
        package='dog_teleop_cpp',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file,
            {
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'max_lateral_speed': LaunchConfiguration('max_lateral_speed'),
                'watchdog_timeout': LaunchConfiguration('watchdog_timeout'),
            }
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ],
    )

    return LaunchDescription([
        max_linear_speed_arg,
        max_angular_speed_arg,
        max_lateral_speed_arg,
        watchdog_timeout_arg,
        keyboard_node,
    ])
