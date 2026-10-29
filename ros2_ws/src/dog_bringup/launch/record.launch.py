"""One command records a floor session the same way every time (CAL-10).

    ros2 launch dog_bringup record.launch.py
    ros2 launch dog_bringup record.launch.py out:=/tmp/bags

The topic list is the contract: it lives in ``TOPICS`` so a test can check every
name against the sources - a renamed topic must not silently drop out of the bag.
What is recorded: the state machine and commands, the joint targets and measured
pulses, the IMU, the power rail, the gamepad, the profile and the terrain estimate.
That is what a post-mortem needs after a fall.
"""

import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

TOPICS = [
    '/dog/state',
    '/dog/command',
    '/dog/estop',
    '/dog/cmd_vel',
    '/dog/joint_states',
    '/dog/joint_commands',
    '/dog/servo_pulses',
    '/dog/guard',
    '/dog/imu/data',
    '/dog/power',
    '/dog/joy',
    '/dog/body_pose',
    '/dog/profile',
    '/dog/terrain/profile',
]


def default_output(now=None):
    """~/dog-bags/<date>-<time> - a new directory per run, nothing overwritten."""
    stamp = time.strftime('%Y%m%d-%H%M%S', time.localtime(now))
    return os.path.join(os.path.expanduser('~'), 'dog-bags', stamp)


def generate_launch_description():
    out = LaunchConfiguration('out')
    return LaunchDescription([
        DeclareLaunchArgument('out', default_value=default_output(),
                              description='directory for this bag'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', out,
                 '--compression-mode', 'file', '--compression-format', 'zstd'] + TOPICS,
            output='screen'),
    ])
