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

from dog_description.urdf import (build_urdf, joint_names, load_config, sensor_frames, sim_command_topic,
                                   sim_sensor_topic, stand_angles)
from dog_gazebo import terrain

NS = 'dog'


def _bridge_config(sensors=None):
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
    for frame, kind, _, _ in sensor_frames(sensors):
        # lidars straight to their ROS topic, ToF cones to tof_bridge
        ros = f'/{NS}/{frame}/scan' if kind in ('lidar', 'gs2') else sim_sensor_topic(NS, frame)
        entries.append({'ros_topic_name': ros, 'gz_topic_name': sim_sensor_topic(NS, frame),
                        'ros_type_name': 'sensor_msgs/msg/LaserScan', 'gz_type_name': 'gz.msgs.LaserScan',
                        'direction': 'GZ_TO_ROS'})
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
    kind, level = cfg('terrain'), float(cfg('level'))
    spawn_z, spawn_pitch = float(cfg('spawn_z')), 0.0
    if kind != 'flat':
        fd, world = tempfile.mkstemp(prefix=f'dog_{kind}_', suffix='.sdf')
        with os.fdopen(fd, 'w') as f:
            f.write(terrain.world(kind, level, int(cfg('seed'))))
        spawn_z, spawn_pitch = terrain.spawn_pose(kind, level, spawn_z)

    geometry, description = load_config(robot_yaml)
    with open(robot_yaml) as f:
        stand_h = yaml.safe_load(f)['/**']['ros__parameters']['stance']['stand_height']
    # Joint controllers hold a standing pose from the first step, like a robot
    # placed on the ground by hand, instead of dropping on straight legs.
    initial = stand_angles(geometry, stand_h)
    urdf = build_urdf(geometry, description, gazebo=True, namespace=NS, initial=initial)
    sim_time = {'use_sim_time': True}
    overrides = {'slope.compensation': on('slope_compensation'), 'heading.hold': on('heading_hold')}
    if on('dead_reckoning'):
        # the robot's own odometry beside Gazebo's true pose ("odom"), with a
        # heading from the gyro (plus a bias: a real gyro drifts)
        overrides.update({'odom.publish': True, 'odom.topic': 'odom_dr', 'odom.yaw_source': 'gyro',
                          'odom.gyro_bias_dps': float(cfg('gyro_bias'))})
    if cfg('step_height'):
        overrides['gait.step_height'] = float(cfg('step_height'))

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
                        # '-P=value': a separate negative value is taken for a flag
                        '-z', f'{spawn_z:.4f}', f'-P={spawn_pitch:.5f}']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': _bridge_config(description.get('sensors'))}, sim_time]),
        Node(package='dog_gazebo', executable='joint_command_bridge', namespace=NS,
             parameters=[sim_time]),
        Node(package='dog_control', executable='locomotion_node', name='locomotion', namespace=NS,
             parameters=[robot_yaml, sim_time, overrides],
             output='screen'),
    ]
    sensors = description.get('sensors', {})
    if sensors.get('tof'):
        actions.append(Node(package='dog_gazebo', executable='tof_bridge', namespace=NS,
                            parameters=[{'names': sensors['tof_names'], 'fov_deg': sensors['tof_fov_deg']},
                                        sim_time]))
    if on('perception'):
        actions.append(Node(package='dog_perception', executable='perception_node', name='perception',
                            namespace=NS, output='screen',
                            parameters=[robot_yaml, sim_time,
                                        {'perception.reference': cfg('perception_reference'),
                                         'perception.guard': on('guard'),
                                         'perception.debug_topics': True}]
                            + ([{'perception.threshold': float(cfg('perception_threshold'))}]
                               if cfg('perception_threshold') else [])
                            + ([{'perception.tof_threshold': [float(v) for v in cfg('tof_threshold').split(',')]}]
                               if cfg('tof_threshold') else [])))
    if on('localization'):
        actions.append(Node(package='dog_perception', executable='localization_node', name='localization',
                            namespace=NS, output='screen',
                            parameters=[robot_yaml, sim_time,
                                        {'localization.map': cfg('map'), 'localization.mode': cfg('localization_mode'),
                                         'localization.loop_closure': on('loop_closure'),
                                         'localization.debug_dump': cfg('loc_debug_dump'),
                                         'localization.scale_estimation': on('loc_scale')}],
                            remappings=[('odom', 'odom_dr')] if on('dead_reckoning') else []))
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
        DeclareLaunchArgument('terrain', default_value='flat',
                              description='flat | slope (level = deg) | waves | rough | steps | wall (level = mm)'),
        DeclareLaunchArgument('level', default_value='0', description='slope angle or obstacle height'),
        DeclareLaunchArgument('seed', default_value='0', description='random layout for rough'),
        DeclareLaunchArgument('slope_compensation', default_value='true',
                              description='IMU-based slope compensation in the gait'),
        DeclareLaunchArgument('heading_hold', default_value='true',
                              description='hold the heading with the IMU gyro'),
        DeclareLaunchArgument('step_height', default_value='',
                              description='override gait.step_height [m] (robot.yaml by default)'),
        DeclareLaunchArgument('perception', default_value='false',
                              description='start dog_perception (X lidars + ToF processing)'),
        DeclareLaunchArgument('perception_reference', default_value='auto',
                              description='ground reference: auto (lidar plane, legs as fallback) | feet'),
        DeclareLaunchArgument('perception_threshold', default_value='',
                              description='lidar hazard threshold [m] (default 0.02)'),
        DeclareLaunchArgument('tof_threshold', default_value='',
                              description='ToF thresholds [m], comma separated per sensor (fl,fr,fc,rc)'),
        DeclareLaunchArgument('guard', default_value='true',
                              description='with perception: slow down / step high / stop at hazards'),
        DeclareLaunchArgument('dead_reckoning', default_value='false',
                              description="locomotion's odometry on odom_dr (the true pose stays on odom)"),
        DeclareLaunchArgument('gyro_bias', default_value='0.0', description='dead reckoning gyro bias [deg/s]'),
        DeclareLaunchArgument('localization', default_value='false', description='start localization_node'),
        DeclareLaunchArgument('localization_mode', default_value='auto', description='auto | mapping | localize'),
        DeclareLaunchArgument('map', default_value='/tmp/dog_sim_map', description='map file without extension'),
        DeclareLaunchArgument('loop_closure', default_value='true', description='localization: close loops'),
        DeclareLaunchArgument('loc_scale', default_value='true',
                              description="localization: learn dead reckoning's scale"),
        DeclareLaunchArgument('loc_debug_dump', default_value='',
                              description='localization: write each relocalization cloud to <this>.<n>.txt'),
        DeclareLaunchArgument('web', default_value='true'),
        DeclareLaunchArgument('gamepad', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
