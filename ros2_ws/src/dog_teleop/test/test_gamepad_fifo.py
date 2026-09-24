"""gamepad_node + joy_teleop fed with raw Linux js_event structs through a FIFO,
the same byte stream /dev/input/jsN produces for a real pad."""

import errno
import os
import struct
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

JS_BUTTON, JS_AXIS, JS_INIT = 0x01, 0x02, 0x80
FIFO = os.path.join(tempfile.mkdtemp(), 'js0')


@pytest.mark.launch_test
def generate_test_description():
    os.mkfifo(FIFO)
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='dog_teleop', executable='gamepad_node', namespace='dog',
                                parameters=[{'device': FIFO}]),
        launch_ros.actions.Node(package='dog_teleop', executable='joy_teleop_node', namespace='dog'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestGamepad(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('pad_tester', namespace='dog')
        self.twists, self.commands, self.estops = [], [], []
        self.node.create_subscription(Twist, 'cmd_vel', self.twists.append, 10)
        self.node.create_subscription(String, 'command', lambda m: self.commands.append(m.data), 10)
        self.node.create_subscription(Bool, 'estop', lambda m: self.estops.append(m.data), 10)
        end = time.time() + 15
        while True:  # wait for gamepad_node to open the FIFO for reading
            try:
                self.fd = os.open(FIFO, os.O_WRONLY | os.O_NONBLOCK)
                break
            except OSError as e:
                if e.errno != errno.ENXIO or time.time() > end:
                    raise
                time.sleep(0.1)
        self.t0 = time.time()

    def tearDown(self):
        os.close(self.fd)
        self.node.destroy_node()
        rclpy.shutdown()

    def ev(self, kind, number, value):
        ms = int((time.time() - self.t0) * 1000)
        os.write(self.fd, struct.pack('IhBB', ms, value, kind, number))

    def spin_until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if cond():
                return True
        return False

    def test_xbox_layout(self):
        for a in range(8):
            self.ev(JS_AXIS | JS_INIT, a, 0)
        for b in range(11):
            self.ev(JS_BUTTON | JS_INIT, b, 0)
        time.sleep(1.0)  # discovery

        # A tapped within one poll period still yields "stand".
        self.ev(JS_BUTTON, 0, 1)
        self.ev(JS_BUTTON, 0, 0)
        self.assertTrue(self.spin_until(lambda: 'stand' in self.commands, 5))

        # Stick without deadman: no motion.
        self.ev(JS_AXIS, 1, -32767)  # left stick fully up
        self.assertFalse(self.spin_until(lambda: len(self.twists) > 0, 0.7))

        # LB held: half speed forward; RB: full speed.
        self.ev(JS_BUTTON, 4, 1)
        self.assertTrue(self.spin_until(
            lambda: self.twists and abs(self.twists[-1].linear.x - 0.075) < 1e-6, 3))
        self.ev(JS_BUTTON, 5, 1)
        self.assertTrue(self.spin_until(
            lambda: abs(self.twists[-1].linear.x - 0.15) < 1e-6, 3))
        # Right stick left = turn left (positive yaw rate).
        self.ev(JS_AXIS, 3, -32767)
        self.assertTrue(self.spin_until(lambda: self.twists[-1].angular.z > 0.5, 3))

        # Releasing the deadman sends a stop.
        self.ev(JS_BUTTON, 4, 0)
        self.assertTrue(self.spin_until(
            lambda: self.twists[-1].linear.x == 0.0 and self.twists[-1].angular.z == 0.0, 3))

        # Back = e-stop, Start = release.
        self.ev(JS_BUTTON, 6, 1)
        self.ev(JS_BUTTON, 6, 0)
        self.ev(JS_BUTTON, 7, 1)
        self.ev(JS_BUTTON, 7, 0)
        self.assertTrue(self.spin_until(lambda: self.estops == [True, False], 3), self.estops)
