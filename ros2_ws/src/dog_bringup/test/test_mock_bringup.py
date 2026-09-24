"""End-to-end: full bringup with the mock servo bus, driven through ROS topics
and through the web teleop WebSocket, exactly as an operator would."""

import base64
import json
import os
import socket
import struct
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

WEB_PORT = 18080


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(get_package_share_directory('dog_bringup'), 'launch', 'robot.launch.py')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'backend': 'mock', 'gamepad': 'false', 'web': 'true',
                              'web_port': str(WEB_PORT), 'power': 'mock'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class WsClient:
    """Tiny blocking WebSocket client (masked frames, text only)."""

    def __init__(self, port):
        self.sock = socket.create_connection(('127.0.0.1', port), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((
            'GET /ws HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
            f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n').encode())
        head = b''
        while b'\r\n\r\n' not in head:
            head += self.sock.recv(1)
        assert b'101' in head.split(b'\r\n')[0]

    def send(self, obj):
        data = json.dumps(obj).encode()
        mask = os.urandom(4)
        hdr = bytes([0x81, 0x80 | len(data)]) if len(data) < 126 else \
            bytes([0x81, 0x80 | 126]) + struct.pack('!H', len(data))
        self.sock.sendall(hdr + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def _exact(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError('closed')
            buf += chunk
        return buf

    def recv(self):
        b1, b2 = self._exact(2)
        n = b2 & 0x7F
        if n == 126:
            (n,) = struct.unpack('!H', self._exact(2))
        return json.loads(self._exact(n).decode())

    def recv_until(self, pred, limit=200):
        for _ in range(limit):
            m = self.recv()
            if pred(m):
                return m
        raise AssertionError('expected message not received')

    def recv_type(self, kind):
        m = self.recv_until(lambda m: m['type'] in (kind, 'error'))
        if m['type'] == 'error':
            raise AssertionError(m['message'])
        return m

    def close(self):
        self.sock.close()


class TestMockBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('bringup_tester', namespace='dog')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state = None
        self.joints = None
        self.node.create_subscription(String, 'state', self._on_state, latched)
        self.node.create_subscription(JointState, 'joint_states', self._on_joints, 10)
        self.cmd_pub = self.node.create_publisher(String, 'command', 10)
        self.vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.estop_pub = self.node.create_publisher(Bool, 'estop', QoSProfile(depth=10))

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

    def test_topics_then_web(self):
        # Wait for the stack to come up (latched state).
        self.assertTrue(self.spin_until(lambda: self.state == 'passive', 20.0),
                        f'locomotion not ready, state={self.state}')

        # --- calibration channel (robot passive): info, pose, live parameter change
        ws = WsClient(WEB_PORT)
        try:
            ws.send({'type': 'cal_hello'})
            info = ws.recv_type('cal_info')
            self.assertEqual(len(info['joints']), 12)
            lf = info['calibration']['lf_thigh_joint']
            self.assertEqual(lf['channel'], 1.0)
            self.assertEqual(lf['servo_arm_mm'], 0.0)
            self.assertAlmostEqual(info['geometry']['thigh'], 0.105)
            ws.send({'type': 'cal_pose', 'joints': {'lf_thigh_joint': 45.0}})
            ws.recv_type('cal_pose_ok')
            status = ws.recv_until(lambda m: m['type'] == 'cal_status'
                                   and m['pulses'].get('lf_thigh_joint', 0) > 1000)
            self.assertAlmostEqual(status['pulses']['lf_thigh_joint'], 1370.0, delta=1.0)  # offset 45
            ws.send({'type': 'cal_set', 'params': {'lf_thigh_joint.offset_deg': 40}})
            self.assertTrue(ws.recv_type('cal_set_result')['ok'])
            status = ws.recv_until(lambda m: m['type'] == 'cal_status'
                                   and abs(m['pulses']['lf_thigh_joint'] - 1370.0) > 20)
            self.assertAlmostEqual(status['pulses']['lf_thigh_joint'], 1370.0 + 5 * 1700 / 180, delta=1.0)
            ws.send({'type': 'cal_set', 'params': {'lf_thigh_joint.direction': 3}})
            self.assertFalse(ws.recv_type('cal_set_result')['ok'])
            ws.send({'type': 'cal_set', 'params': {'lf_thigh_joint.offset_deg': 45}})
            self.assertTrue(ws.recv_type('cal_set_result')['ok'])
        finally:
            ws.close()

        # --- topics: stand up
        self.assertTrue(self.spin_until(
            lambda: self.state == 'stand', 8.0,
            publish=lambda: self.cmd_pub.publish(String(data='stand'))
            if self.state == 'passive' else None))
        self.assertTrue(self.spin_until(lambda: self.joints is not None, 3.0))
        self.assertEqual(len(self.joints.name), 12)
        standing = list(self.joints.position)
        # Knees bent backwards (negative), thighs leaning back (positive).
        for leg in range(4):
            self.assertGreater(standing[leg * 3 + 1], 0.3)
            self.assertLess(standing[leg * 3 + 2], -0.8)

        # --- topics: walk forward, joints must move
        twist = Twist()
        twist.linear.x = 0.1
        self.assertTrue(self.spin_until(
            lambda: self.state == 'walk', 3.0, publish=lambda: self.vel_pub.publish(twist)))
        seen = set()
        self.spin_until(lambda: False, 1.0, publish=lambda: (
            self.vel_pub.publish(twist), seen.add(round(self.joints.position[1], 3))))
        self.assertGreater(len(seen), 10, 'thigh joint did not move while walking')

        # --- cmd_vel timeout: stop publishing, robot must return to stand
        self.assertTrue(self.spin_until(lambda: self.state == 'stand', 4.0))

        # --- web: connect, get hello + state, drive, e-stop
        ws = WsClient(WEB_PORT)
        try:
            hello = ws.recv()
            self.assertEqual(hello['type'], 'hello')
            self.assertAlmostEqual(hello['limits']['max_vx'], 0.15)
            # Power readings from the (mock) current sensor reach the page.
            msg = ws.recv()
            while msg['type'] != 'power':
                msg = ws.recv()
            self.assertAlmostEqual(msg['voltage'], 6.0, places=2)
            self.assertAlmostEqual(msg['current'], 1.0, places=2)
            drive = {'type': 'drive', 'vx': 1.0, 'vy': 0.0, 'wz': 0.0}
            self.assertTrue(self.spin_until(
                lambda: self.state == 'walk', 4.0, publish=lambda: ws.send(drive)))
            ws.send({'type': 'estop', 'active': True})
            self.assertTrue(self.spin_until(lambda: self.state == 'estop', 3.0))
            # Commands are rejected while the e-stop is engaged.
            self.cmd_pub.publish(String(data='stand'))
            self.assertFalse(self.spin_until(lambda: self.state != 'estop', 1.0))
            ws.send({'type': 'estop', 'active': False})
            self.assertTrue(self.spin_until(lambda: self.state == 'passive', 3.0))
            ws.send({'type': 'command', 'name': 'stand'})
            self.assertTrue(self.spin_until(lambda: self.state == 'stand', 8.0))
            # Calibration moves are refused while the robot stands.
            ws.send({'type': 'cal_pose', 'joints': {'lf_thigh_joint': 45.0}})
            msg = ws.recv_until(lambda m: m['type'] == 'error')
            self.assertIn('passive', msg['message'])
            # Bad input is answered with an error, not a crash.
            ws.send({'type': 'drive', 'vx': 'fast'})
            msg = ws.recv()
            while msg['type'] != 'error':
                msg = ws.recv()
            # Lie down via web.
            ws.send({'type': 'command', 'name': 'lie'})
            self.assertTrue(self.spin_until(lambda: self.state == 'lying', 8.0))
        finally:
            ws.close()

        # --- topic e-stop turns the (mock) servos off: joint_states keep publishing
        self.estop_pub.publish(Bool(data=True))
        self.assertTrue(self.spin_until(lambda: self.state == 'estop', 3.0))
        self.estop_pub.publish(Bool(data=False))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
