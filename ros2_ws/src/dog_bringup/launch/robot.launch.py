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
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dog_description.urdf import build_urdf, load_config

# profile.py is installed next to this file (CMakeLists installs both into share/launch)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import profile as profiles  # noqa: E402

NS = 'dog'


def _setup(context):
    cfg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    log = lambda text: print('[dog_bringup] ' + text, flush=True)  # noqa: E731
    on = lambda name: cfg(name).lower() in ('1', 'true', 'yes')  # noqa: E731
    share = get_package_share_directory('dog_bringup')
    robot_yaml = cfg('robot_config') or os.path.join(share, 'config', 'robot.yaml')
    servos_yaml = cfg('servo_config') or os.path.join(share, 'config', 'servos.yaml')
    # The measured calibration lands in servos.local.yaml: it is not tracked, so a
    # `git pull` cannot overwrite what the robot was calibrated to (CAL-05). The
    # overlay is passed second, so it wins over the reference values in git.
    servos_local = os.path.join(os.path.dirname(servos_yaml), 'servos.local.yaml')
    servo_files = [servos_yaml]
    if os.path.isfile(servos_local):
        servo_files.append(servos_local)
    power_yaml = os.path.join(share, 'config', 'power.yaml')
    # Power sensor: probe the bus on the robot; off for mock runs unless asked.
    power_backend = cfg('power') or ('auto' if cfg('backend') == 'pca9685' else 'off')
    imu_yaml = os.path.join(share, 'config', 'imu.yaml')
    imu_backend = cfg('imu') or ('auto' if cfg('backend') == 'pca9685' else 'off')
    # On hardware the driver waits for ~/arm: the robot is always started by hand
    # (D-06). A simulation has nobody to press it, so the mock backend arms itself.
    armed = (on('armed_on_start') if cfg('armed_on_start') else cfg('backend') == 'mock')
    profile_name = profiles.check(cfg('profile'))
    calibrate = on('calibrate')
    teleop_yaml = os.path.join(share, 'config', 'teleop.yaml')
    teleop_files = [teleop_yaml]
    if cfg('gamepad_profile') == 'ps':
        teleop_files.append(os.path.join(share, 'config', 'teleop_ps.yaml'))

    log(profiles.summary(profile_name))
    log('servo calibration: ' + ', '.join(servo_files))
    geometry, description = load_config(robot_yaml)
    urdf = build_urdf(geometry, description)

    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=NS, parameters=[{'robot_description': urdf}]),
        # odom: dead-reckoned by locomotion (there is no odometry sensor); the
        # perception guard and map need a pose
        Node(package='dog_control', executable='locomotion_node', name='locomotion',
             namespace=NS,
             parameters=[robot_yaml, {'odom.publish': True},
                         profiles.params(profile_name, 'locomotion')], output='screen'),
        Node(package='dog_hardware', executable='servo_driver_node', name='servo_driver',
             namespace=NS, parameters=servo_files + [{'backend': cfg('backend'),
                                                      'armed_on_start': armed,
                                                      # 0 = no fault; >0 makes the mock bus
                                                      # start failing N seconds in, so the
                                                      # fault drill runs on a healthy stack
                                                      'mock.fail_after': float(cfg('bus_fail_after'))}],
             output='screen'),
    ]
    if power_backend != 'off':
        actions.append(
            Node(package='dog_hardware', executable='power_monitor_node', name='power_monitor',
                 namespace=NS, parameters=[power_yaml, {'backend': power_backend}], output='screen'))
    if imu_backend != 'off':
        actions.append(
            Node(package='dog_hardware', executable='imu_node', name='imu',
                 namespace=NS, parameters=[imu_yaml, {'backend': imu_backend}], output='screen'))
    if on('perception'):
        # lidars / GS2 / VL53L1X drivers publish lidar_left/scan, lidar_right/scan,
        # gs2/scan, tof/<name> (DEPLOYMENT.md, stage 14); the guard slows / stops
        # the gait at hazards
        actions.append(
            Node(package='dog_perception', executable='perception_node', name='perception',
                 namespace=NS, parameters=[robot_yaml], output='screen'))
    if on('localization'):
        # map of the room from the lidars; where the robot is in it (LOCALIZATION.md)
        actions.append(
            Node(package='dog_perception', executable='localization_node', name='localization',
                 namespace=NS, parameters=[robot_yaml]
                 + ([{'localization.map': cfg('map')}] if cfg('map') else []), output='screen'))
    if on('gamepad'):
        # The reader can be left out while joy_teleop stays: the bench drills feed /dog/joy
        # themselves (tools/joy_replay) to test the timeouts without a physical pad.
        if on('gamepad_reader'):
            actions.append(
                Node(package='dog_teleop', executable='gamepad_node', name='gamepad',
                     namespace=NS, parameters=teleop_files, output='screen'))
        actions.append(
            Node(package='dog_teleop', executable='joy_teleop_node', name='joy_teleop',
                 namespace=NS,
                 parameters=teleop_files + [profiles.params(profile_name, 'joy_teleop')],
                 output='screen'))
    if on('web'):
        actions.append(
            Node(package='dog_web', executable='web_teleop', name='web_teleop',
                 namespace=NS,
                 parameters=[teleop_yaml, {'port': int(cfg('web_port'))},
                             {'allow_calibration': profiles.allow_calibration(profile_name, calibrate)},
                             profiles.params(profile_name, 'web_teleop')],
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
        DeclareLaunchArgument('bus_fail_after', default_value='0.0',
                              description='mock backend only: the bus starts failing after N s'),
        DeclareLaunchArgument('gamepad', default_value='true',
                              description='start gamepad reader + joy_teleop'),
        DeclareLaunchArgument('gamepad_reader', default_value='true',
                              description='read /dev/input/js*; false when another publisher '
                                          'feeds /dog/joy (fault drill, joy replay)'),
        DeclareLaunchArgument('gamepad_profile', default_value='xbox',
                              description='button layout: xbox or ps'),
        DeclareLaunchArgument('web', default_value='true', description='start the web teleop page'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('rviz', default_value='false', description='start RViz (PC only)'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='map of the walls from the lidars and localization in it'),
        DeclareLaunchArgument('map', default_value='',
                              description='map file without extension (default: localization.map)'),
        DeclareLaunchArgument('power', default_value='',
                              description='current sensor: auto (probe I2C), mock or off; '
                                          'default auto on the robot, off with backend:=mock'),
        DeclareLaunchArgument('imu', default_value='',
                              description='IMU (MPU6050): auto (probe I2C), mock or off; '
                                          'default auto on the robot, off with backend:=mock'),
        DeclareLaunchArgument('armed_on_start', default_value='',
                              description='power the servos at start: default false on hardware '
                                          '(call ~/arm), true with backend:=mock'),
        DeclareLaunchArgument('perception', default_value='false',
                              description='terrain perception + hazard guard (needs the sensor drivers)'),
        DeclareLaunchArgument('robot_config', default_value='',
                              description='override path to robot.yaml'),
        DeclareLaunchArgument('servo_config', default_value='',
                              description='override path to servos.yaml'),
        DeclareLaunchArgument('profile', default_value='bench',
                              description='bench (table) or floor (first walk outside: 0.05 m/s, '
                                          'IMU required, web view-only, no calibration)'),
        DeclareLaunchArgument('calibrate', default_value='false',
                              description='open the web calibration channel for a calibration '
                                          'session; ignored by the floor profile'),
        OpaqueFunction(function=_setup),
    ])
