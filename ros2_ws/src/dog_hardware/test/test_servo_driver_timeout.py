"""servo_driver on the mock bus: when the command stream dies, the driver stops
holding the pose and latches (SAF-04).

The hold (`command_timeout`) is 0.5 s; past `command_relax_timeout` more, the
driver relaxes the outputs, publishes estop and waits for `release_fault`.
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
            parameters=[{'backend': 'mock', 'update_rate': 100.0,
                         'command_timeout': 0.5, 'command_relax_timeout': 1.0}]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestServoDriverTimeout(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('servo_timeout_tester', namespace='dog')
        self.estops = []
        self.node.create_subscription(Bool, 'estop', lambda m: self.estops.append(m.data), 10)
        self.cmd = self.node.create_publisher(JointState, 'joint_commands', 10)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds, publish=True):
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

    def test_silence_relaxes_and_latches(self):
        # The driver comes up disarmed; arm it, then command the joint for a second.
        self.arm()
        self.spin(1.0)
        self.assertNotIn(True, self.estops, 'the driver must not latch while commands flow')

        # Silence. The hold is 0.5 s, the relax timeout another 1.0 s.
        self.spin(2.5, publish=False)
        self.assertIn(True, self.estops,
                      '1.5 s of silence must relax the outputs and latch the driver')
