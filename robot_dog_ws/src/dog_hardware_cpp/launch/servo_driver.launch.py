#!/usr/bin/env python3
"""
Launch file for servo driver node

Usage:
    ros2 launch dog_hardware_cpp servo_driver.launch.py
    ros2 launch dog_hardware_cpp servo_driver.launch.py bus_type:=i2c
    ros2 launch dog_hardware_cpp servo_driver.launch.py bus_type:=simulation
"""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, DeclareLaunchArgument
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import Transition
import os

def generate_launch_description():
    # Launch arguments
    bus_type_arg = DeclareLaunchArgument(
        'bus_type',
        default_value='simulation',
        description='Bus type: i2c, uart, or simulation'
    )
    
    device_port_arg = DeclareLaunchArgument(
        'device_port',
        default_value='',
        description='Device port (auto-detected if empty)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to configuration YAML file'
    )
    
    safety_enabled_arg = DeclareLaunchArgument(
        'safety_enabled',
        default_value='true',
        description='Enable safety limits'
    )
    
    watchdog_enabled_arg = DeclareLaunchArgument(
        'watchdog_enabled',
        default_value='true',
        description='Enable watchdog timer'
    )
    
    # Get config file path
    pkg_share = FindPackageShare('dog_hardware_cpp')
    default_config = os.path.join(
        pkg_share.find('dog_hardware_cpp'),
        'config',
        'servo_params.yaml'
    )
    
    # Servo driver lifecycle node
    servo_node = LifecycleNode(
        package='dog_hardware_cpp',
        executable='servo_driver_node',
        name='servo_driver_node',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file') if LaunchConfiguration('config_file') else default_config,
            {
                'bus_type': LaunchConfiguration('bus_type'),
                'device_port': LaunchConfiguration('device_port'),
                'safety_enabled': LaunchConfiguration('safety_enabled'),
                'enable_watchdog': LaunchConfiguration('watchdog_enabled'),
            }
        ],
        remappings=[
            ('/joint_states', '/dog/joint_states'),
            ('/joint_commands', '/dog/joint_commands'),
            ('/joint_trajectory', '/dog/joint_trajectory'),
            ('/emergency_stop', '/dog/emergency_stop'),
            ('/emergency_stop_trigger', '/dog/emergency_stop_trigger'),
            ('/servo_enable', '/dog/servo_enable'),
        ]
    )
    
    # Lifecycle transitions
    servo_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=servo_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    servo_activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=servo_node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )
    
    return LaunchDescription([
        bus_type_arg,
        device_port_arg,
        config_file_arg,
        safety_enabled_arg,
        watchdog_enabled_arg,
        servo_node,
        servo_configure,
        servo_activate,
    ])
