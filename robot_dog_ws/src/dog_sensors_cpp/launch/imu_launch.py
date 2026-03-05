from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Launch arguments
    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='false',
        description='Use simulation mode instead of hardware'
    )
    
    driver_type_arg = DeclareLaunchArgument(
        'driver_type',
        default_value='auto',
        description='Driver type: auto, mpu6050, simulator, none'
    )
    
    motion_mode_arg = DeclareLaunchArgument(
        'sim_motion_mode',
        default_value='idle',
        description='Simulation motion mode: idle, walking, trotting, turning, pacing'
    )
    
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='/dev/i2c-1',
        description='I2C bus path for MPU6050'
    )
    
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='0x68',
        description='I2C address for MPU6050 (0x68 or 0x69)'
    )

    # IMU Node as lifecycle node
    imu_node = LifecycleNode(
        package='dog_sensors_cpp',
        executable='imu_node',
        name='imu_node',
        namespace='',
        output='screen',
        parameters=[{
            'frame_id': 'imu_link',
            'publish_rate_hz': 100.0,
            'simulate': LaunchConfiguration('simulate'),
            'driver_type': LaunchConfiguration('driver_type'),
            'publish_pose': True,
            'linear_accel_bias': [0.0, 0.0, 0.0],
            'angular_vel_bias': [0.0, 0.0, 0.0],
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'i2c_address': LaunchConfiguration('i2c_address'),
            'sim_motion_mode': LaunchConfiguration('sim_motion_mode'),
            'sim_walk_frequency': 1.0,
            'sim_walk_amplitude': 0.15,
        }],
    )
    
    # Configure transition
    imu_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=imu_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    # Activate transition
    imu_activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=imu_node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )
    
    return LaunchDescription([
        simulate_arg,
        driver_type_arg,
        motion_mode_arg,
        i2c_bus_arg,
        i2c_address_arg,
        imu_node,
        imu_configure,
        imu_activate,
    ])
