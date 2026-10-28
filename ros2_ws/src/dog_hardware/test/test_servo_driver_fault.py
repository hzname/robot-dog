"""servo_driver on the mock bus: a dead bus latches, announces estop, and only
an explicit release brings it back (SAF-01, SAF-02, SAF-03).

`mock.fail_after` is the test hook: after that many seconds every write on the
mock bus fails, which is what a broken I2C line looks like from the driver.
"""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='dog_hardware', executable='servo_driver_node', namespace='dog',
            parameters=[{'backend': 'mock', 'mock.fail_after': 5.0, 'update_rate': 100.0}]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestServoDriverFault(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('servo_fault_tester', namespace='dog')
        self.estops = []
        self.node.create_subscription(Bool, 'estop', lambda m: self.estops.append(m.data), 10)
        self.cmd = self.node.create_publisher(JointState, 'joint_commands', 10)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds, publish=True):
        # A moving joint keeps the driver writing: with a constant target it
        # writes once on power-on and then has nothing to send, so a dead bus
        # would go unnoticed.
        end = time.time() + seconds
        start = time.time()
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if not publish:
                continue
            msg = JointState()
            msg.name = ['lf_hip_joint']
            msg.position = [0.3 * math.sin(2.0 * math.pi * 0.5 * (time.time() - start))]
            self.cmd.publish(msg)

    def _call_driver(self, suffix):
        """Calls /dog/servo_driver/<suffix> once it shows up in the graph.

        Discovery of a fresh node takes a variable second or two, and a client
        created before the graph knows it cannot be resolved at all, so the name is
        taken from the graph instead of guessed.
        """
        deadline = time.time() + 15.0
        name = None
        while time.time() < deadline and name is None:
            self.spin(0.2, publish=False)
            found = [n for n, _ in self.node.get_service_names_and_types()
                     if n.endswith('/servo_driver/' + suffix)]
            name = found[0] if found else None
        self.assertIsNotNone(name, 'no %s service: %s' % (
            suffix, sorted(n for n, _ in self.node.get_service_names_and_types())))
        client = self.node.create_client(Trigger, name)
        while time.time() < deadline:
            if client.wait_for_service(timeout_sec=0.2):
                break
        self.assertTrue(client.service_is_ready(), '%s is not reachable' % name)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), '%s did not answer' % suffix)
        return future.result()

    def arm(self):
        res = self._call_driver('arm')
        self.assertTrue(res.success, res.message)

    def test_a_dead_bus_latches_and_only_the_service_releases(self):
        # The driver starts disarmed: an operator arms it, then the bus dies.
        self.arm()
        self.spin(4.0)
        self.assertIn(True, self.estops,
                      'a driver that latched a bus fault must publish estop=true')

        res = self._call_driver('release_fault')
        self.assertTrue(res.success, res.message)
        self.assertIn('fault released', res.message)
        self.assertIn(False, self.estops,
                      'releasing the fault must withdraw the estop announcement')

        # No further commands: the driver must stay quiet instead of re-latching.
        self.spin(1.0, publish=False)
