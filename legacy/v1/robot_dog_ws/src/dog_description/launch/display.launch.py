#!/usr/bin/env python3
"""
Launch file for visualizing the robot dog in RViz.

Usage:
    ros2 launch dog_description display.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for robot visualization."""
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='urdf/dog.urdf.xacro',
        description='Path to the URDF/xacro file relative to package root'
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_path = LaunchConfiguration('urdf_path')
    
    # Get package share directory
    pkg_share = FindPackageShare('dog_description')
    
    # Construct full path to URDF
    urdf_file = PathJoinSubstitution([pkg_share, urdf_path])
    
    # Process xacro to generate robot_description
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher GUI node (for interactive joint control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        urdf_path_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
