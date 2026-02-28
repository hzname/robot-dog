#!/usr/bin/env python3
"""
Launch file for gait controller with parameters.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for gait controller."""
    
    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('dog_bringup'),
        'config',
        'robot_config.yaml'
    ])
    
    # Gait controller node
    gait_controller_node = Node(
        package='dog_control',
        executable='gait_controller',
        name='gait_controller',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        gait_controller_node,
    ])
