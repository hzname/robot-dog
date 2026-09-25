"""Launch file for UDP teleoperation."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
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
    
    udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='8888',
        description='UDP port for receiving commands'
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

    # UDP teleop node
    udp_node = Node(
        package='dog_teleop_cpp',
        executable='udp_teleop',
        name='udp_teleop',
        output='screen',
        parameters=[
            config_file,
            {
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'max_lateral_speed': LaunchConfiguration('max_lateral_speed'),
                'udp_port': LaunchConfiguration('udp_port'),
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
        udp_port_arg,
        watchdog_timeout_arg,
        udp_node,
    ])
