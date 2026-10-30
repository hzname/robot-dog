#!/usr/bin/env python3
"""The interactive half of the IMU check: subscribe, walk the operator through the steps,
then hand the recorded samples to imu_axes.judge (CAL-08).

Run it on the robot while the stack is up and the robot is on its stand:

    docker compose exec robot python3 /ws/tools/imu_axes/run_ros.py --out /tmp/imu-check

It asks for four still moments (rest, nose down, left side up, counter-clockwise turn),
prints the verdict, and writes one CSV per step so the check can be attached to the
calibration record. It never writes to imu.yaml: the owner edits the config.

The angles are taken from `orientation` - the same field locomotion reads - and compared
against the acceleration, so a remap that lies shows up as a disagreement.
"""

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import imu_axes  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402


def quaternion_to_roll_pitch(qx, qy, qz, qw):
    """REP-103: roll about x, pitch about y, both from the orientation quaternion."""
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    return roll * 180.0 / math.pi, math.asin(sin_pitch) * 180.0 / math.pi


def accel_to_roll_pitch(ax, ay, az):
    """The same two angles, straight from gravity: an independent witness."""
    roll = math.atan2(ay, az) if (ay or az) else 0.0
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) if ax else 0.0
    return roll * 180.0 / math.pi, pitch * 180.0 / math.pi


class ImuProbe(Node):
    def __init__(self):
        super().__init__('imu_axes_check')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.samples = []
        self.calibrated = None
        self.mismatch = 0.0
        # SensorDataQoS on the publisher: best effort, depth 5. A reliable subscriber is
        # incompatible with it and silently receives nothing at all.
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, '/dog/imu/data', self.on_imu, sensor)
        self.create_subscription(Bool, '/dog/imu/calibrated', self.on_cal, latched)

    def on_imu(self, msg):
        roll, pitch = quaternion_to_roll_pitch(msg.orientation.x, msg.orientation.y,
                                               msg.orientation.z, msg.orientation.w)
        a = msg.linear_acceleration
        a_roll, a_pitch = accel_to_roll_pitch(a.x, a.y, a.z)
        self.mismatch = max(abs(roll - a_roll), abs(pitch - a_pitch))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append(imu_axes.Sample(t, a.x, a.y, a.z,
                                            msg.angular_velocity.x, msg.angular_velocity.y,
                                            msg.angular_velocity.z, roll, pitch))

    def on_cal(self, msg):
        self.calibrated = bool(msg.data)

    def collect(self, seconds):
        """Samples from the next `seconds` of wall time, with the queue cleared first."""
        self.samples = []
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        got = list(self.samples)
        self.samples = []
        return got


STEPS = [
    ('rest', 'Поставьте робота ровно и не двигайте'),
    ('nose_down', 'Наклоните робота носом вниз (градусов на 15) и держите'),
    ('left_up', 'Поднимите левый бок (градусов на 15) и держите'),
    ('ccw', 'Крутите робота против часовой стрелки, если смотреть сверху'),
]


def ask(text):
    print('\n' + text)
    try:
        input('  [Enter], когда готово: ')
    except EOFError:
        pass


def main(argv=None):
    ap = argparse.ArgumentParser(description='Проверка осей IMU по трём движениям.')
    ap.add_argument('--seconds', type=float, default=2.0, help='сколько записывать на шаг')
    ap.add_argument('--out', help='каталог для CSV (по одному файлу на шаг)')
    ap.add_argument('--topic', default='/dog/imu/data')
    a = ap.parse_args(argv)

    rclpy.init()
    probe = ImuProbe()
    if a.topic != '/dog/imu/data':
        probe.create_subscription(Imu, a.topic, probe.on_imu,
                                  QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    print('Проверка IMU: четыре неподвижных положения, %.0f с каждое.' % a.seconds)
    print('Стек должен быть запущен; робот стоит на подставке, сервы не нужны.')
    # Give discovery a moment, then make sure somebody is publishing at all.
    warm = probe.collect(1.0)
    if len(warm) < 10:
        print('НЕТ ДАННЫХ: /dog/imu/data молчит. Проверьте, что imu_node поднялся '
              '(`ros2 topic hz /dog/imu/data`).', file=sys.stderr)
        probe.destroy_node()
        rclpy.shutdown()
        return 2

    recorded = {}
    for name, hint in STEPS:
        ask(hint)
        recorded[name] = probe.collect(a.seconds)
        print('  записано %d отсчётов' % len(recorded[name]))
        if probe.mismatch > 5.0:
            print('  ВНИМАНИЕ: orientation расходится с акселерометром на %.1f° — '
                  'это дефект пересчёта, а не монтажа' % probe.mismatch)

    if probe.calibrated is None:
        print('\nimu/calibrated не приходил: нода не отметила замер смещения гироскопа.')
    else:
        print('\nimu/calibrated: %s' % ('да' if probe.calibrated else 'ещё нет'))

    verdict = imu_axes.judge(recorded['rest'], recorded['nose_down'],
                             recorded['left_up'], recorded['ccw'])
    print('\n' + imu_axes.render(verdict))

    if a.out:
        os.makedirs(a.out, exist_ok=True)
        for name, rows in recorded.items():
            path = os.path.join(a.out, name + '.csv')
            with open(path, 'w', encoding='utf-8') as f:
                for s in rows:
                    f.write('%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f\n' % s)
        print('\nзаписи: %s' % a.out)

    probe.destroy_node()
    rclpy.shutdown()
    return 0 if verdict['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
