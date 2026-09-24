from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Health Monitor node."""
    return LaunchDescription([
        Node(
            package='dog_monitor_cpp',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[{
                'check_interval_s': 1.0,
                'node_timeout_s': 3.0,
                'max_restarts': 3,
                'monitored_nodes': [
                    '/gait_controller',
                    '/servo_driver_node',
                    '/imu_node',
                    '/balance_controller'
                ]
            }]
        )
    ])
