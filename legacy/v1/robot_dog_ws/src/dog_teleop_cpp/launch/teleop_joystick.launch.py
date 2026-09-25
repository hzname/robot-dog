"""Launch file for joystick/gamepad teleoperation."""
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
    
    axis_deadzone_arg = DeclareLaunchArgument(
        'axis_deadzone',
        default_value='0.1',
        description='Axis deadzone (0.0 - 1.0)'
    )
    
    watchdog_timeout_arg = DeclareLaunchArgument(
        'watchdog_timeout',
        default_value='0.5',
        description='Watchdog timeout in seconds'
    )
    
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    # Config file
    config_file = os.path.join(
        FindPackageShare('dog_teleop_cpp').find('dog_teleop_cpp'),
        'config',
        'teleop_config.yaml'
    )

    # Joy node (from joy package)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen',
    )

    # Gamepad teleop node
    gamepad_node = Node(
        package='dog_teleop_cpp',
        executable='gamepad_teleop',
        name='gamepad_teleop',
        output='screen',
        parameters=[
            config_file,
            {
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'max_lateral_speed': LaunchConfiguration('max_lateral_speed'),
                'axis_deadzone': LaunchConfiguration('axis_deadzone'),
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
        axis_deadzone_arg,
        watchdog_timeout_arg,
        joy_dev_arg,
        joy_node,
        gamepad_node,
    ])
