"""CAL-08: no `imu/calibrated` until a human has verified the axes.

Two nodes with the mock backend, identical except for the trust mark: one is what a fresh
robot looks like (`axes_verified: false`, every mock-behaviour switch off detail below), the
other is a robot whose axes went through tools/imu_axes. The first must stay silent on
`imu/calibrated` while still publishing attitude, and its diagnostics must say why; the
second must report itself calibrated.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

NAMESPACE = 'imu_gate'


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        # A robot nobody has checked: the mock backend, but its trust switch is off, which
        # is exactly the state of a real board before the four positions are walked.
        launch_ros.actions.Node(
            package='dog_hardware', executable='imu_node', name='imu', namespace=NAMESPACE,
            parameters=[{'backend': 'mock', 'mock.trust_axes': False, 'bias_time': 0.3,
                         'rate': 50.0}]),
        # The same node after the check: the mark is the only difference that matters.
        launch_ros.actions.Node(
            package='dog_hardware', executable='imu_node', name='imu_checked',
            namespace='imu_gate_ok',
            parameters=[{'backend': 'mock', 'axes_verified': True, 'bias_time': 0.3,
                         'rate': 50.0}]),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestImuGate(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('imu_gate_tester')
        self.untrusted = []          # imu/calibrated under NAMESPACE
        self.trusted = []
        self.attitude = 0
        self.diagnostics = []
        self.node.create_subscription(
            Bool, '/%s/imu/calibrated' % NAMESPACE, lambda m: self.untrusted.append(m.data), 10)
        self.node.create_subscription(
            Bool, '/imu_gate_ok/imu/calibrated', lambda m: self.trusted.append(m.data), 10)
        self.node.create_subscription(
            DiagnosticArray, '/%s/diagnostics' % NAMESPACE, self.diagnostics.append, 10)
        # The driver publishes with SensorDataQoS: a reliable subscriber would be
        # incompatible with it and would silently receive nothing.
        self.node.create_subscription(
            Imu, '/%s/imu/data' % NAMESPACE, self._on_imu,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _on_imu(self, _msg):
        self.attitude += 1

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, seconds, until=None):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def test_the_gate_needs_a_human_not_a_timer(self):
        # One method, one context: rclpy.init per test method makes the second context
        # discover nothing, which reads as "the node is silent" and is a lie about the code.
        self.assertTrue(self.spin_for(8.0, lambda: self.diagnostics),
                        'диагностика не пришла')
        self.assertTrue(self.spin_for(1.5))

        self.assertFalse(any(self.untrusted),
                         'imu/calibrated отправлен без проверки осей: %s' % self.untrusted)
        self.assertGreater(self.attitude, 10,
                           'отношение публикуется и без отметки: инструменту нужны данные')

        said = ' '.join(
            status.message for msg in self.diagnostics
            for status in msg.status if status.name.endswith('imu'))
        self.assertIn('axes not verified', said,
                      'диагностика обязана назвать причину: %r' % said[:400])

        self.assertTrue(self.spin_for(8.0, lambda: any(self.trusted)),
                        'с проверенными осями отметка не появилась: %s' % self.trusted)
