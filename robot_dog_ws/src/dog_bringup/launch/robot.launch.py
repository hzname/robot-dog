import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    dog_description_dir = get_package_share_directory('dog_description')
    dog_bringup_dir = get_package_share_directory('dog_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_teleop = LaunchConfiguration('use_teleop', default='false')
    
    # URDF file
    urdf_file = os.path.join(dog_description_dir, 'urdf', 'dog.urdf.xacro')
    
    # Config file
    config_file = os.path.join(dog_bringup_dir, 'config', 'robot_config.yaml')
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher (for simulation without real hardware)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # Gait controller
    gait_controller_node = Node(
        package='dog_control',
        executable='gait_controller',
        name='gait_controller',
        output='screen',
        parameters=[config_file]
    )
    
    # Teleop keyboard (optional)
    teleop_node = Node(
        package='dog_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        parameters=[config_file]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(dog_description_dir, 'urdf', 'dog.rviz')] 
                  if os.path.exists(os.path.join(dog_description_dir, 'urdf', 'dog.rviz')) 
                  else []
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz if true'
        ),
        DeclareLaunchArgument(
            'use_teleop',
            default_value='false',
            description='Launch teleop keyboard if true'
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        gait_controller_node,
        teleop_node,
        rviz_node
    ])
