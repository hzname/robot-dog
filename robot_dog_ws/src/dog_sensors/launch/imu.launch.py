from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for IMU node."""
    
    # Launch arguments
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='1',
        description='I2C bus number for MPU6050'
    )
    
    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Use mock data instead of real sensor'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='TF frame ID for IMU'
    )
    
    # IMU node
    imu_node = Node(
        package='dog_sensors',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'use_mock': LaunchConfiguration('use_mock'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': 100.0,
        }],
        remappings=[
            ('/imu/data', '/imu/data'),
        ]
    )
    
    return LaunchDescription([
        i2c_bus_arg,
        use_mock_arg,
        frame_id_arg,
        imu_node,
    ])
