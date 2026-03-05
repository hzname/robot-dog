"""
Hardware launch file for dog_control_cpp
Launches: gait controller + balance controller (uses real IMU)
"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, LogInfo
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config files
    balance_config = PathJoinSubstitution([
        FindPackageShare('dog_control_cpp'),
        'config',
        'balance_params.yaml'
    ])
    
    gait_config = PathJoinSubstitution([
        FindPackageShare('dog_control_cpp'),
        'config',
        'gait_params.yaml'
    ])

    # Gait Controller (lifecycle node)
    gait_node = LifecycleNode(
        package='dog_control_cpp',
        executable='gait_controller',
        name='gait_controller',
        output='screen',
        parameters=[gait_config],
    )
    
    # Balance Controller (lifecycle node)
    balance_node = LifecycleNode(
        package='dog_control_cpp',
        executable='balance_controller',
        name='balance_controller',
        output='screen',
        parameters=[balance_config],
    )
    
    # Lifecycle transitions for gait controller
    gait_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=gait_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    gait_activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=gait_node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )
    
    # Lifecycle transitions for balance controller
    balance_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=balance_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    balance_activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=balance_node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )
    
    return LaunchDescription([
        LogInfo(msg="Starting Dog Control (Hardware Mode)..."),
        LogInfo(msg="Make sure IMU is publishing to /imu/data"),
        
        gait_node,
        balance_node,
        
        gait_configure,
        gait_activate,
        balance_configure,
        balance_activate,
    ])
