"""Robot Dog 2.0 bringup.

Real robot (Banana Pi):
  ros2 launch dog_bringup robot.launch.py
PC without hardware (servo output mocked, optional RViz):
  ros2 launch dog_bringup robot.launch.py backend:=mock rviz:=true

Keyboard control runs in its own terminal (needs a TTY):
  ros2 run dog_teleop keyboard_teleop --ros-args -r __ns:=/dog \
      --params-file $(ros2 pkg prefix dog_bringup)/share/dog_bringup/config/teleop.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dog_description.urdf import build_urdf, load_config

NS = 'dog'


def _setup(context):
    cfg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    on = lambda name: cfg(name).lower() in ('1', 'true', 'yes')  # noqa: E731
    share = get_package_share_directory('dog_bringup')
    robot_yaml = cfg('robot_config') or os.path.join(share, 'config', 'robot.yaml')
    servos_yaml = cfg('servo_config') or os.path.join(share, 'config', 'servos.yaml')
    teleop_yaml = os.path.join(share, 'config', 'teleop.yaml')
    teleop_files = [teleop_yaml]
    if cfg('gamepad_profile') == 'ps':
        teleop_files.append(os.path.join(share, 'config', 'teleop_ps.yaml'))

    geometry, description = load_config(robot_yaml)
    urdf = build_urdf(geometry, description)

    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=NS, parameters=[{'robot_description': urdf}]),
        Node(package='dog_control', executable='locomotion_node', name='locomotion',
             namespace=NS, parameters=[robot_yaml], output='screen'),
        Node(package='dog_hardware', executable='servo_driver_node', name='servo_driver',
             namespace=NS, parameters=[servos_yaml, {'backend': cfg('backend')}],
             output='screen'),
    ]
    if on('gamepad'):
        actions += [
            Node(package='dog_teleop', executable='gamepad_node', name='gamepad',
                 namespace=NS, parameters=teleop_files, output='screen'),
            Node(package='dog_teleop', executable='joy_teleop_node', name='joy_teleop',
                 namespace=NS, parameters=teleop_files, output='screen'),
        ]
    if on('web'):
        actions.append(
            Node(package='dog_web', executable='web_teleop', name='web_teleop',
                 namespace=NS, parameters=[teleop_yaml, {'port': int(cfg('web_port'))}],
                 output='screen'))
    if on('rviz'):
        actions.append(
            Node(package='rviz2', executable='rviz2', name='rviz',
                 arguments=['-d', os.path.join(share, 'config', 'dog.rviz')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='pca9685',
                              description='servo output: pca9685 (I2C) or mock'),
        DeclareLaunchArgument('gamepad', default_value='true',
                              description='start gamepad reader + joy_teleop'),
        DeclareLaunchArgument('gamepad_profile', default_value='xbox',
                              description='button layout: xbox or ps'),
        DeclareLaunchArgument('web', default_value='true', description='start the web teleop page'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('rviz', default_value='false', description='start RViz (PC only)'),
        DeclareLaunchArgument('robot_config', default_value='',
                              description='override path to robot.yaml'),
        DeclareLaunchArgument('servo_config', default_value='',
                              description='override path to servos.yaml'),
        OpaqueFunction(function=_setup),
    ])
