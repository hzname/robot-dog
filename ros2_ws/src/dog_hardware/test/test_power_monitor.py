"""power_monitor with the mock sensor: stall -> e-stop, sagging rail -> lie."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='dog_hardware', executable='power_monitor_node', namespace='dog',
            parameters=[{'backend': 'mock', 'mock.voltage': 6.0, 'mock.current': 1.0,
                         'overcurrent_a': 5.0, 'overcurrent_time': 0.5,
                         'undervoltage_v': 5.0, 'undervoltage_time': 0.3}]),
        launch_ros.actions.Node(
            package='dog_hardware', executable='power_monitor_node', namespace='absent',
            parameters=[{'backend': 'auto', 'i2c.device': '/dev/i2c-does-not-exist'}]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestPowerMonitor(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('power_tester', namespace='dog')
        self.power, self.estops, self.commands = [], [], []
        self.node.create_subscription(BatteryState, 'power', self.power.append, 10)
        self.node.create_subscription(Bool, 'estop', lambda m: self.estops.append(m.data), 10)
        self.node.create_subscription(String, 'command', lambda m: self.commands.append(m.data), 10)
        self.cli = self.node.create_client(SetParameters, '/dog/power_monitor/set_parameters')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def spin_until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if cond():
                return True
        return False

    def set(self, name, value):
        self.assertTrue(self.cli.wait_for_service(timeout_sec=10))
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=value))]
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5)
        self.assertTrue(fut.result().results[0].successful)

    def test_protection(self, proc_info):
        self.assertTrue(self.spin_until(lambda: len(self.power) > 3, 10))
        self.assertAlmostEqual(self.power[-1].voltage, 6.0, places=3)
        self.assertAlmostEqual(self.power[-1].current, -1.0, places=3)  # drawn = negative
        self.assertEqual(self.estops, [])

        self.set('mock.current', 7.0)
        self.assertTrue(self.spin_until(lambda: self.estops == [True], 3), self.estops)
        self.set('mock.current', 1.0)

        self.set('mock.voltage', 4.6)
        self.assertTrue(self.spin_until(lambda: 'lie' in self.commands, 3), self.commands)
        self.assertEqual(self.estops, [True])  # nothing else fired

        # SAF-07: the rail loss latches. The rail comes back and sags again, and the
        # robot must not send a second "lie" as if this were a passing event.
        self.spin(1.0)
        self.assertEqual(self.commands.count('lie'), 1, self.commands)
        self.set('mock.voltage', 6.0)
        self.spin(0.5)
        self.set('mock.voltage', 4.6)
        self.spin(1.5)
        self.assertEqual(self.commands.count('lie'), 1, 'a latched rail must not re-fire')

        # A human looked at the supply and cleared it; now it can fire again.
        cle = self.node.create_client(Trigger, '/dog/power_monitor/clear_power_fault')
        self.assertTrue(cle.wait_for_service(timeout_sec=10))
        fut = cle.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5)
        self.assertTrue(fut.result().success)
        self.assertTrue(self.spin_until(lambda: self.commands.count('lie') == 2, 3), self.commands)


class TestAbsentSensor(unittest.TestCase):

    def test_a_missing_sensor_is_announced_not_hidden(self):
        # SAF-07: without a sensor the node stays up and says "protection inactive".
        # A node that exits leaves "no news" to be read as "no problem".
        rclpy.init()
        node = rclpy.create_node('power_absent_tester')
        seen = []
        node.create_subscription(DiagnosticArray, '/absent/diagnostics',
                                 lambda m: seen.extend(m.status), 10)
        deadline = time.time() + 15.0
        found = False
        while time.time() < deadline and not found:
            rclpy.spin_once(node, timeout_sec=0.5)
            found = any('protection inactive' in st.message for st in seen
                        if st.level == DiagnosticStatus.ERROR)
        node.destroy_node()
        rclpy.shutdown()
        self.assertTrue(found, [st.message for st in seen])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_the_absent_sensor_node_is_alive_until_shutdown(self, proc_info):
        # It no longer finishes by itself: it is the only place that reports the rail
        # as unguarded, so it stays up until the launch ends.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
