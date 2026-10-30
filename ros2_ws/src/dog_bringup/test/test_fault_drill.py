"""SAF-18 on the mock stack: the chain of safety, proven by faults rather than asserted.

Three drills that need no robot:

  1. an operator stops the robot from a publisher and releases it again - the e-stop round
     trip, the same messages the page and the gamepad send;
  2. the state machine dies while the robot is standing - the driver notices that the
     commands stopped, holds, relaxes and latches, and announces an e-stop so nobody walks
     a robot whose servos stopped listening;
  3. the bus itself starts failing - the same latch, `~/arm` refuses while it is latched,
     and only `~/release_fault` clears it.

What stays on the bench: a physical kill -9 with servo-guard dropping the outputs (SAF-16,
a host-side watch that this stack cannot provide for itself), the SDA wire, the power
switch, and the tilt in the cradle.

Drill 2 only makes sense while the servos are holding: a robot lying passive has nothing to
lose when its state machine dies, and the driver is right not to cry wolf.
"""

import math
import os
import subprocess
import threading
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

BUS_FAIL_AFTER = 45.0        # seconds after the driver comes up, from the launch argument
BUS_DRILL_DEADLINE = 120.0   # the failure plus the latch, with room for a slow machine


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(get_package_share_directory('dog_bringup'), 'launch',
                               'robot.launch.py')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'backend': 'mock', 'gamepad': 'false', 'web': 'false',
                              'power': 'mock', 'imu': 'mock', 'profile': 'bench',
                              'bus_fail_after': str(BUS_FAIL_AFTER)}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class CommandStream(threading.Thread):
    """Stands in for the dead state machine: keeps the pose alive with a small wiggle, so
    the driver keeps writing to the bus and a failing write has something to fail."""

    def __init__(self, node, names, pose):
        super().__init__(daemon=True)
        self.node = node
        self.names = list(names)
        self.pose = list(pose)
        self.stop_flag = threading.Event()

    def run(self):
        pub = self.node.create_publisher(JointState, 'joint_commands', 10)
        while not self.stop_flag.is_set():
            t = time.monotonic()
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.name = self.names
            msg.position = [p + (0.05 * math.sin(2.0 * math.pi * 2.0 * t) if 'hip' in n else 0.0)
                            for n, p in zip(self.names, self.pose)]
            pub.publish(msg)
            time.sleep(0.05)

    def stop(self):
        self.stop_flag.set()
        self.join(timeout=2.0)


class FaultDrill(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fault_drill', namespace='dog')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.estop = None
        self.state = None
        self.joints = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Bool, 'estop', self._on_estop, 10)
        self.node.create_subscription(String, 'state', self._on_state, latched)
        self.node.create_subscription(JointState, 'joint_states', self._on_joints, 10)
        self.estop_pub = self.node.create_publisher(Bool, 'estop', 10)
        self.cmd_pub = self.node.create_publisher(String, 'command', 10)

    def _on_estop(self, msg):
        self.estop = bool(msg.data)

    def _on_state(self, msg):
        self.state = msg.data

    def _on_joints(self, msg):
        self.joints = msg

    def spin_until(self, predicate, timeout, publish=None):
        end = time.time() + timeout
        while time.time() < end:
            if publish:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def set_estop(self, active, seconds=0.6):
        """Publish the e-stop for a moment, not once: on a loaded runner discovery can be
        slower than a single message is alive, and a person keeps pressing too."""
        end = time.time() + seconds
        while time.time() < end:
            self.estop_pub.publish(Bool(data=active))
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.1)

    def say(self, word):
        def _publish():
            self.cmd_pub.publish(String(data=word))
        return _publish

    def call(self, service, timeout=10.0):
        client = self.node.create_client(Trigger, service)
        end = time.time() + timeout
        while time.time() < end and not client.service_is_ready():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(client.service_is_ready(), 'нет сервиса %s' % service)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result(), 'сервис %s не ответил' % service)
        return future.result()

    # --- 1. the operator's own path ------------------------------------------------
    def test_1_the_robot_stops_and_comes_back(self):
        self.assertTrue(self.spin_until(lambda: self.state == 'passive', 40.0),
                        'стек не поднялся, state=%s' % self.state)
        self.set_estop(True)
        self.assertTrue(self.spin_until(lambda: self.state == 'estop', 15.0),
                        'E-STOP не дошёл до состояния: %s' % self.state)
        self.set_estop(False)
        self.assertTrue(self.spin_until(lambda: self.state == 'passive', 15.0),
                        'робот не вернулся из E-STOP: %s' % self.state)

    # --- 2. the state machine dies while the servos are holding --------------------
    def test_2_a_dead_state_machine_latches_the_driver(self):
        # Each drill starts clean: a latch left by the previous one keeps the servos off
        # and then nothing can be proven about the driver's silence detection.
        self.call('/dog/servo_driver/release_fault')
        self.assertTrue(self.spin_until(lambda: self.state == 'stand', 30.0,
                                        self.say('stand')),
                        'робот не встал, state=%s' % self.state)
        self.assertTrue(self.spin_until(lambda: self.joints is not None, 5.0),
                        'нет /dog/joint_states')
        pose = list(self.joints.position)
        names = list(self.joints.name)
        self.assertGreater(len(pose), 0)

        subprocess.run(['pkill', '-f', 'locomotion_node'], check=False)
        # The driver holds for 0.5 s and relaxes after 2.5 s of silence, then latches.
        self.assertTrue(self.spin_until(lambda: self.estop is True, 30.0),
                        'драйвер не защёлкнулся после смерти локомотива')
        answer = self.call('/dog/servo_driver/release_fault')
        self.assertTrue(answer.success, answer.message)
        self.assertTrue(self.spin_until(lambda: self.estop is False, 15.0),
                        'защёлка не снялась сервисом')
        del names, pose

    # --- 3. the bus itself fails ---------------------------------------------------
    def test_3_a_failing_bus_latches_and_refuses_to_arm(self):
        self.call('/dog/servo_driver/release_fault')
        # Nobody drives the robot any more: the test stands in for the state machine, with
        # a small wiggle so the driver keeps writing to the bus for the failure to hit.
        self.assertTrue(self.spin_until(lambda: self.joints is not None, 20.0),
                        'нет /dog/joint_states: драйвер не публикует обратную связь')
        stream = CommandStream(self.node, self.joints.name, self.joints.position)
        stream.start()
        try:
            time.sleep(0.5)   # let the joints reach a pose worth holding
            self.assertTrue(self.call('/dog/servo_driver/arm').success,
                            'драйвер не вооружился при живом потоке команд')
            self.assertTrue(self.spin_until(lambda: self.estop is True, BUS_DRILL_DEADLINE),
                            'отказ шины не привёл к защёлке за %.0f с' % BUS_DRILL_DEADLINE)
            answer = self.call('/dog/servo_driver/arm')
            self.assertFalse(answer.success, 'армирование при защёлке должно отказывать')
            self.assertIn('fault', answer.message.lower())
            released = self.call('/dog/servo_driver/release_fault')
            self.assertTrue(released.success, released.message)
        finally:
            stream.stop()
