from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Arguments
    teleop_type_arg = DeclareLaunchArgument(
        'teleop_type',
        default_value='keyboard',
        description='Teleoperation type: keyboard or gamepad'
    )
    
    # Keyboard teleop
    keyboard_node = Node(
        package='dog_teleop_cpp',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        parameters=[{
            'linear_speed': 0.3,
            'angular_speed': 1.0,
            'lateral_speed': 0.2,
            'publish_rate': 50.0,
        }],
        condition=UnlessCondition(LaunchConfiguration('teleop_type') == 'gamepad')
    )
    
    # Gamepad teleop
    gamepad_node = Node(
        package='dog_teleop_cpp',
        executable='gamepad_teleop',
        name='gamepad_teleop',
        output='screen',
        parameters=[{
            'max_linear_speed': 0.5,
            'max_angular_speed': 2.0,
            'max_lateral_speed': 0.3,
            'axis_deadzone': 0.1,
            'publish_rate': 50.0,
        }],
        condition=IfCondition(LaunchConfiguration('teleop_type') == 'gamepad')
    )
    
    return LaunchDescription([
        teleop_type_arg,
        keyboard_node,
        gamepad_node,
    ])
