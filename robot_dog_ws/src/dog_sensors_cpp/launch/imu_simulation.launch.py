from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Launch file for testing IMU node in simulation mode (no hardware required)."""
    
    # Launch arguments
    motion_mode_arg = DeclareLaunchArgument(
        'motion_mode',
        default_value='walking',
        description='Simulation motion mode: idle, walking, trotting, turning, pacing'
    )
    
    walk_freq_arg = DeclareLaunchArgument(
        'walk_frequency',
        default_value='1.0',
        description='Walking gait frequency in Hz'
    )

    # IMU Node in simulation mode
    imu_node = LifecycleNode(
        package='dog_sensors_cpp',
        executable='imu_node',
        name='imu_node',
        namespace='',
        output='screen',
        parameters=[{
            'frame_id': 'imu_link',
            'publish_rate_hz': 100.0,
            'simulate': True,
            'driver_type': 'simulator',
            'publish_pose': True,
            'linear_accel_bias': [0.0, 0.0, 0.0],
            'angular_vel_bias': [0.0, 0.0, 0.0],
            'sim_motion_mode': LaunchConfiguration('motion_mode'),
            'sim_walk_frequency': LaunchConfiguration('walk_frequency'),
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
        motion_mode_arg,
        walk_freq_arg,
        imu_node,
        imu_configure,
        imu_activate,
    ])
