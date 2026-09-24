"""Robot Dog 2.0 in Gazebo (Harmonic on Jazzy, Jetty on Lyrical).

  ros2 launch dog_gazebo sim.launch.py              # GUI + web teleop on :8080
  ros2 launch dog_gazebo sim.launch.py headless:=true gamepad:=true

The same locomotion node and teleop nodes as on the robot are used; only the
servo driver is replaced by Gazebo joint controllers.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dog_description.urdf import build_urdf, joint_names, load_config, sim_command_topic

NS = 'dog'


def _bridge_config():
    entries = [
        {'ros_topic_name': '/clock', 'gz_topic_name': '/clock',
         'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'/{NS}/joint_states', 'gz_topic_name': f'/{NS}/sim/joint_states',
         'ros_type_name': 'sensor_msgs/msg/JointState', 'gz_type_name': 'gz.msgs.Model',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'/{NS}/imu/data', 'gz_topic_name': f'/{NS}/sim/imu',
         'ros_type_name': 'sensor_msgs/msg/Imu', 'gz_type_name': 'gz.msgs.IMU',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'/{NS}/odom', 'gz_topic_name': f'/{NS}/sim/odom',
         'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'gz.msgs.Odometry',
         'direction': 'GZ_TO_ROS'},
    ]
    for j in joint_names():
        topic = sim_command_topic(NS, j)
        entries.append({'ros_topic_name': topic, 'gz_topic_name': topic,
                        'ros_type_name': 'std_msgs/msg/Float64', 'gz_type_name': 'gz.msgs.Double',
                        'direction': 'ROS_TO_GZ'})
    fd, path = tempfile.mkstemp(prefix='dog_gz_bridge_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(entries, f)
    return path


def _setup(context):
    cfg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    on = lambda name: cfg(name).lower() in ('1', 'true', 'yes')  # noqa: E731
    bringup = get_package_share_directory('dog_bringup')
    robot_yaml = os.path.join(bringup, 'config', 'robot.yaml')
    teleop_yaml = os.path.join(bringup, 'config', 'teleop.yaml')
    world = cfg('world') or os.path.join(get_package_share_directory('dog_gazebo'), 'worlds', 'flat.sdf')

    geometry, description = load_config(robot_yaml)
    urdf = build_urdf(geometry, description, gazebo=True, namespace=NS)
    sim_time = {'use_sim_time': True}

    gz_args = f"-r {'-s --headless-rendering ' if on('headless') else ''}-v 2 {world}"
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=NS,
             parameters=[{'robot_description': urdf}, sim_time]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'dog', '-topic', f'/{NS}/robot_description',
                        '-z', cfg('spawn_z')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': _bridge_config()}, sim_time]),
        Node(package='dog_gazebo', executable='joint_command_bridge', namespace=NS,
             parameters=[sim_time]),
        Node(package='dog_control', executable='locomotion_node', name='locomotion', namespace=NS,
             parameters=[robot_yaml, sim_time], output='screen'),
    ]
    if on('gamepad'):
        actions += [
            Node(package='dog_teleop', executable='gamepad_node', name='gamepad', namespace=NS,
                 parameters=[teleop_yaml]),
            Node(package='dog_teleop', executable='joy_teleop_node', name='joy_teleop',
                 namespace=NS, parameters=[teleop_yaml]),
        ]
    if on('web'):
        actions.append(Node(package='dog_web', executable='web_teleop', name='web_teleop',
                            namespace=NS, parameters=[teleop_yaml], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', description='server only, no GUI'),
        DeclareLaunchArgument('world', default_value='', description='SDF world (default: flat)'),
        DeclareLaunchArgument('spawn_z', default_value='0.25', description='spawn height [m]'),
        DeclareLaunchArgument('web', default_value='true'),
        DeclareLaunchArgument('gamepad', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
