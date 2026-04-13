#!/usr/bin/env python3
"""
ROS2 Command Bridge - runs INSIDE the RobotDogQwen container
Listens on Unix socket for JSON commands and publishes to ROS2 topics
"""
import json
import math
import os
import socket
import sys
import threading

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')

import rclpy
from rclpy import executors
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray

SOCKET_PATH = '/tmp/robot_dog/ros_bridge.sock'
# Topic prefix — set DOG_TOPIC_PREFIX=/dog for namespaced topics, or leave empty for flat topics
TOPIC_PREFIX = os.environ.get('DOG_TOPIC_PREFIX', '')

def _t(name):
    """Prefix a topic name."""
    return f'{TOPIC_PREFIX}{name}'

class Bridge:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('web_bridge')
        self.cmd_vel_pub = self.node.create_publisher(Twist, _t('/cmd_vel'), 10)
        self.servo_pub = self.node.create_publisher(Bool, _t('/servo_enable'), 10)
        self.estop_pub = self.node.create_publisher(Bool, _t('/emergency_stop_trigger'), 10)
        self.pos_pub = self.node.create_publisher(Float64MultiArray, _t('/joint_position_command'), 10)
        self.gait_enable_pub = self.node.create_publisher(Bool, _t('/gait_enable'), 10)

        # Subscribe to joint states for feedback (BestEffort QoS)
        from sensor_msgs.msg import JointState
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        joint_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_positions = [0.0] * 12
        self.node.create_subscription(JointState, _t('/joint_states'), self._on_joints, joint_qos)

        # Subscribe to emergency stop state
        self.emergency_stopped = False
        self.node.create_subscription(Bool, _t('/emergency_stop'), self._on_estop, 10)

    def _on_joints(self, msg):
        self.joint_positions = list(msg.position) if len(msg.position) == 12 else self.joint_positions

    def _on_estop(self, msg):
        self.emergency_stopped = msg.data

    def handle(self, cmd):
        t = cmd.get('type')
        if t == 'cmd_vel':
            msg = Twist()
            msg.linear.x = float(cmd.get('linear_x', 0.0))
            msg.linear.y = float(cmd.get('linear_y', 0.0))
            msg.angular.z = float(cmd.get('angular_z', 0.0))
            # Reject NaN values
            if not (math.isfinite(msg.linear.x) and
                    math.isfinite(msg.linear.y) and
                    math.isfinite(msg.angular.z)):
                return {'error': 'cmd_vel values must be finite numbers'}
            self.cmd_vel_pub.publish(msg)
            return {'ok': True}
        elif t == 'stop':
            self.cmd_vel_pub.publish(Twist())
            return {'ok': True}
        elif t == 'servo_enable':
            msg = Bool()
            msg.data = bool(cmd.get('enable', True))
            self.servo_pub.publish(msg)
            return {'ok': True}
        elif t == 'emergency_stop':
            msg = Bool()
            msg.data = bool(cmd.get('stop', True))
            self.estop_pub.publish(msg)
            return {'ok': True}
        elif t == 'gait_enable':
            msg = Bool()
            msg.data = bool(cmd.get('enable', True))
            self.gait_enable_pub.publish(msg)
            return {'ok': True}
        elif t == 'joint_command':
            positions = cmd.get('positions', [])
            if len(positions) != 12:
                return {'error': f'joint_command requires exactly 12 positions, got {len(positions)}'}
            msg = Float64MultiArray()
            msg.data = [float(v) for v in positions]
            self.pos_pub.publish(msg)
            return {'ok': True}
        elif t == 'get_state':
            return {
                'ok': True,
                'joint_positions': self.joint_positions,
                'emergency_stopped': self.emergency_stopped
            }
        return {'error': 'unknown command: ' + str(t)}


def _handle_connection(bridge, conn):
    """Handle a single client connection in its own thread."""
    try:
        # Read full message with length-prefix framing
        # First 4 bytes = message length (big-endian), rest = JSON payload
        header = b''
        while len(header) < 4:
            chunk = conn.recv(4 - len(header))
            if not chunk:
                return
            header += chunk
        msg_len = int.from_bytes(header, 'big')
        if msg_len > 65536:
            conn.send(json.dumps({'error': 'message too large'}).encode())
            return
        data = b''
        while len(data) < msg_len:
            chunk = conn.recv(msg_len - len(data))
            if not chunk:
                break
            data += chunk
        if data:
            cmd = json.loads(data.decode())
            result = bridge.handle(cmd)
            response = json.dumps(result).encode()
            conn.send(len(response).to_bytes(4, 'big') + response)
    except json.JSONDecodeError:
        try:
            conn.send(json.dumps({'error': 'invalid JSON'}).encode())
        except Exception:
            pass
    except Exception as e:
        try:
            conn.send(json.dumps({'error': str(e)}).encode())
        except Exception:
            pass
    finally:
        conn.close()


def main():
    bridge = Bridge()

    # Spin ROS2 in background thread
    executor = executors.SingleThreadedExecutor()
    executor.add_node(bridge.node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Setup Unix socket
    os.makedirs(os.path.dirname(SOCKET_PATH), exist_ok=True)
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(5)
    server.settimeout(1.0)
    os.chmod(SOCKET_PATH, 0o666)

    print(f'Bridge listening on {SOCKET_PATH}', flush=True)

    while rclpy.ok():
        try:
            conn, _ = server.accept()
            t = threading.Thread(target=_handle_connection, args=(bridge, conn), daemon=True)
            t.start()
        except socket.timeout:
            continue
        except Exception as e:
            print(f'Bridge accept error: {e}', flush=True)


if __name__ == '__main__':
    main()
