"""
Launch file for servo driver node.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for servo driver."""
    
    # Get package path
    pkg_path = get_package_share_directory('dog_hardware')
    
    # Default config file
    default_config = os.path.join(pkg_path, 'config', 'servo_config.yaml')
    
    # Launch arguments
    declare_config = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to servo configuration YAML file'
    )
    
    declare_i2c_bus = DeclareLaunchArgument(
        'i2c_bus',
        default_value='0',
        description='I2C bus number for PCA9685'
    )
    
    declare_i2c_address = DeclareLaunchArgument(
        'i2c_address',
        default_value='64',  # 0x40 = 64
        description='I2C address for PCA9685 (decimal)'
    )
    
    declare_use_mock = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Use mock mode without hardware'
    )
    
    # Servo driver node
    servo_driver_node = Node(
        package='dog_hardware',
        executable='servo_driver_node',
        name='servo_driver_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'i2c_address': LaunchConfiguration('i2c_address'),
            'use_mock': LaunchConfiguration('use_mock'),
        }],
        remappings=[
            ('/joint_commands', '/joint_commands'),
            ('/joint_states', '/joint_states'),
        ]
    )
    
    return LaunchDescription([
        declare_config,
        declare_i2c_bus,
        declare_i2c_address,
        declare_use_mock,
        servo_driver_node,
    ])
