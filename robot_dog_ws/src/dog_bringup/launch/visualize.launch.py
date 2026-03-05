#!/usr/bin/env python3
"""
visualize.launch.py

Launch file for robot dog visualization in simulation mode.
Launches all necessary nodes for testing and visualizing the robot dog.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config = LaunchConfiguration('rviz_config', default='')
    
    # Find package share directories
    dog_bringup_share = FindPackageShare('dog_bringup')
    dog_description_share = FindPackageShare('dog_description')
    
    # URDF file path
    urdf_path = PathJoinSubstitution([
        dog_description_share,
        'urdf',
        'dog.urdf.xacro'
    ])
    
    # Default RViz config path
    default_rviz_config = PathJoinSubstitution([
        dog_bringup_share,
        'rviz',
        'dog_config.rviz'
    ])
    
    # Use provided rviz config or default
    rviz_config_path = LaunchConfiguration('rviz_config')
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher (for testing without real hardware)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=None  # Only when no real hardware
    )
    
    # Joint State Publisher GUI (optional, for manual joint control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # IMU Sensor Node (simulation mode)
    imu_node = Node(
        package='dog_sensors_cpp',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'simulation_mode': True,
            'publish_rate': 100.0
        }],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/mag', '/imu/mag'),
            ('/imu/temp', '/imu/temp')
        ]
    )
    
    # Gait Controller Node
    gait_controller_node = Node(
        package='dog_control_cpp',
        executable='gait_controller_node',
        name='gait_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'gait_type': 'trot',
            'step_height': 0.05,
            'step_length': 0.1,
            'cycle_period': 0.5
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/joint_states', '/joint_states'),
            ('/servo_commands', '/servo_commands')
        ]
    )
    
    # Balance Controller Node
    balance_controller_node = Node(
        package='dog_control_cpp',
        executable='balance_controller_node',
        name='balance_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'kp_roll': 0.5,
            'kp_pitch': 0.5,
            'kp_yaw': 0.3,
            'enable_balance': True
        }],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/balance_correction', '/balance_correction')
        ]
    )
    
    # IMU Simulator Node (for testing without real IMU)
    imu_simulator_node = Node(
        package='dog_control_cpp',
        executable='imu_simulator_node',
        name='imu_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': 100.0,
            'simulate_noise': True
        }],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/imu/data_raw', '/imu/data_raw')
        ]
    )
    
    # Servo Driver Node (simulation mode)
    servo_driver_node = Node(
        package='dog_hardware_cpp',
        executable='servo_driver_node',
        name='servo_driver',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'simulation_mode': True,
            'servo_rate': 50.0,
            'num_servos': 12,
            'servo_ids': [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        }],
        remappings=[
            ('/servo_commands', '/servo_commands'),
            ('/joint_states', '/joint_states'),
            ('/servo_feedback', '/servo_feedback')
        ]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=None
    )
    
    # Alternative RViz node with default config if not specified
    rviz_node_default = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Path to RViz configuration file'
        ),
        
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        imu_node,
        gait_controller_node,
        balance_controller_node,
        imu_simulator_node,
        servo_driver_node,
        rviz_node_default,
    ])
