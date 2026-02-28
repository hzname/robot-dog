from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch IMU and balance controller together.
    
    Usage:
      ros2 launch dog_sensors imu_balance.launch.py
      ros2 launch dog_sensors imu_balance.launch.py use_mock:=true
    """
    
    # Launch arguments
    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Use mock IMU data'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Include IMU launch
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dog_sensors'),
                'launch',
                'imu.launch.py'
            ])
        ]),
        launch_arguments={
            'use_mock': LaunchConfiguration('use_mock'),
        }.items()
    )
    
    # Include balance controller launch
    balance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dog_control'),
                'launch',
                'balance_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    return LaunchDescription([
        use_mock_arg,
        use_sim_time_arg,
        imu_launch,
        balance_launch,
    ])
