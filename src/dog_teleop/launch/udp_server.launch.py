"""
Launch file for UDP remote teleoperation server.

Usage:
    ros2 launch dog_teleop udp_server.launch.py
    
On remote PC:
    python3 udp_client.py --host <robo_dog_ip> --port 8888
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch UDP server for remote teleoperation."""
    
    # Declare arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8888',
        description='UDP port to listen on'
    )
    
    max_clients_arg = DeclareLaunchArgument(
        'max_clients',
        default_value='4',
        description='Maximum number of simultaneous clients'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout_sec',
        default_value='2.0',
        description='Client timeout in seconds'
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
    
    # UDP server node
    udp_server_node = Node(
        package='dog_teleop',
        executable='udp_server',
        name='teleop_udp_server',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'max_clients': LaunchConfiguration('max_clients'),
            'timeout_sec': LaunchConfiguration('timeout_sec'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'smoothing_factor': 0.2,
            'publish_rate': 20.0,
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ],
    )
    
    return LaunchDescription([
        port_arg,
        max_clients_arg,
        timeout_arg,
        linear_speed_arg,
        angular_speed_arg,
        udp_server_node,
    ])
