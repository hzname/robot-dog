"""Fall protection end to end, with the real binaries and a mock IMU (SAF-05).

The chain under test: the IMU is tilted past the hard threshold, the controller
latches FALLEN, the driver is told to release through the estop topic, the
diagnostics carry the same words the log does, and ~/recover refuses to raise a
robot that is still tilted - then succeeds once it is level.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='dog_hardware', executable='servo_driver_node', name='servo_driver',
            namespace='dog',
            parameters=[{'backend': 'mock', 'update_rate': 100.0, 'armed_on_start': True}]),
        launch_ros.actions.Node(
            package='dog_hardware', executable='imu_node', name='imu', namespace='dog',
            parameters=[{'backend': 'mock', 'rate': 100.0, 'bias_time': 0.2}]),
        launch_ros.actions.Node(
            package='dog_control', executable='locomotion_node', name='locomotion',
            namespace='dog',
            parameters=[{'safety.profile': 'bench', 'safety.soft_deg': 22.0,
                         'safety.hard_deg': 32.0, 'safety.idle_lie_s': 0.0}]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestFallProtection(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('safety_tester', namespace='dog')
        self.estops = []
        self.states = []
        self.diags = []
        latched = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Bool, 'estop', lambda m: self.estops.append(m.data), 10)
        self.node.create_subscription(String, 'state', lambda m: self.states.append(m.data), latched)
        self.node.create_subscription(DiagnosticArray, '/dog/diagnostics', self.on_diagnostics, 10)
        self.cmd = self.node.create_publisher(String, 'command', 10)

    def on_diagnostics(self, msg):
        for status in msg.status:
            # rclpy hands a uint8 field back as one byte, not as an int.
            level = status.level[0] if isinstance(status.level, bytes) else status.level
            self.diags.append((status.name, level, status.message))

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------------ helpers
    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def tilt(self, degrees):
        client = self.node.create_client(SetParameters, '/dog/imu/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), 'imu set_parameters is missing')
        request = SetParameters.Request()
        request.parameters = [ParameterMsg(
            name='mock.roll_deg',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(degrees)))]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), 'setting the tilt did not answer')

    def call(self, name):
        client = self.node.create_client(Trigger, name)
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), '%s is missing' % name)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), '%s did not answer' % name)
        return future.result()

    def last_state(self):
        return self.states[-1] if self.states else ''

    # -------------------------------------------------------------------- tests
    def test_a_hard_tilt_falls_the_robot_and_recover_refuses_until_level(self):
        self.spin(3.0)  # discovery and the first diagnostics
        self.cmd.publish(String(data='stand'))
        self.spin(3.0)
        self.assertEqual(self.last_state(), 'stand', 'the robot must stand before the test')
        self.assertIn('locomotion', [name for name, _, _ in self.diags],
                      'locomotion must publish diagnostics')

        self.tilt(40.0)
        self.spin(1.0)
        self.assertEqual(self.last_state(), 'fallen', 'a 40 deg tilt must latch FALLEN')
        self.assertIn(True, self.estops, 'the fall must be announced as estop=true')
        self.spin(1.5)  # diagnostics are published at 1 Hz
        fall = [msg for name, level, msg in self.diags if name == 'locomotion' and level == 2]
        self.assertTrue(fall, 'the fall must be an ERROR in diagnostics; got %s' % (self.diags[-6:],))
        self.assertIn('tilt', fall[-1].lower())

        res = self.call('/dog/locomotion/recover')
        self.assertFalse(res.success, 'a tilted robot must not be raised')
        self.assertIn('tilt', res.message.lower())

        self.tilt(0.0)
        self.spin(1.0)
        res = self.call('/dog/locomotion/recover')
        self.assertTrue(res.success, res.message)
        self.spin(3.0)
        self.assertIn(self.last_state(), ('lying', 'standing_up', 'stand'),
                      'after recover the robot must be on its way up: %s' % self.last_state())
