from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for balance controller."""
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to config file (optional)'
    )
    
    # Build parameters dict
    config_file = LaunchConfiguration('config_file')
    
    # Balance controller node
    balance_node = Node(
        package='dog_control',
        executable='balance_controller',
        name='balance_controller',
        output='screen',
        parameters=[
            # Default parameters (used if no config file provided)
            {
                'pid.roll.kp': 0.5,
                'pid.roll.ki': 0.01,
                'pid.roll.kd': 0.1,
                'pid.pitch.kp': 0.5,
                'pid.pitch.ki': 0.01,
                'pid.pitch.kd': 0.1,
                'pid.output_limit': 0.05,
                'pid.integral_limit': 0.1,
                'balance.enabled': True,
                'balance.leg_span_x': 0.3,
                'balance.leg_span_y': 0.2,
                'balance.max_correction': 0.03,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
            # Config file (if provided)
            config_file,
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        balance_node,
    ])
