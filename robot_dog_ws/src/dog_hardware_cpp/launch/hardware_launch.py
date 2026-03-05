from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    servo_node = LifecycleNode(
        package='dog_hardware_cpp',
        executable='servo_driver_node',
        name='servo_driver_node',
        output='screen',
        parameters=[{
            'pwm_frequency': 50,
            'publish_rate': 100.0,
            'safety_enabled': True,
            'emergency_stop_on_timeout': True,
            'command_timeout': 0.5,
        }],
    )
    
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
        servo_node,
        servo_configure,
        servo_activate,
    ])
