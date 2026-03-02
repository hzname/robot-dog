#!/usr/bin/env python3
"""Launch robot with Rust nodes + ROS2 bridge for RViz"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    use_rust = LaunchConfiguration('use_rust', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rust',
            default_value='true',
            description='Use Rust nodes for control'
        ),
        
        # Robot State Publisher (URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(
                    '/home/sg/robot_dog_ws/install/dog_description/share/dog_description/urdf/dog.urdf.xacro'
                ).read()
            }]
        ),
        
        # Rust Bridge (connects Rust nodes to ROS2)
        Node(
            package='dog_bringup',
            executable='rust_bridge.py',
            name='rust_bridge',
            output='screen',
            condition=IfCondition(use_rust)
        ),
        
        # Joint State Publisher GUI (optional, for testing)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=UnlessCondition(use_rust)
        ),
    ])
