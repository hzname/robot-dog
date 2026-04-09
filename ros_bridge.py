#!/usr/bin/env python3
"""
ROS2 Command Bridge - runs INSIDE the robot_dog container
Listens on Unix socket for JSON commands and publishes to ROS2 topics
"""
import json
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

class Bridge:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('web_bridge')
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_pub = self.node.create_publisher(Bool, '/servo_enable', 10)
        self.estop_pub = self.node.create_publisher(Bool, '/emergency_stop_trigger', 10)
        self.pos_pub = self.node.create_publisher(Float64MultiArray, '/joint_position_command', 10)
        self.gait_enable_pub = self.node.create_publisher(Bool, '/gait_enable', 10)
        
        # Subscribe to joint states for feedback (BestEffort QoS)
        from sensor_msgs.msg import JointState
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        joint_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_positions = [0.0] * 12
        self.node.create_subscription(JointState, '/joint_states', self._on_joints, joint_qos)
        
        # Subscribe to emergency stop state
        self.emergency_stopped = False
        self.node.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)

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
            # Stop gait while sending direct joint commands
            self.gait_enable_pub.publish(Bool(data=False))
            msg = Float64MultiArray()
            msg.data = [float(v) for v in cmd.get('positions', [])]
            self.pos_pub.publish(msg)
            return {'ok': True}
        elif t == 'get_state':
            return {
                'ok': True,
                'joint_positions': self.joint_positions,
                'emergency_stopped': self.emergency_stopped
            }
        return {'error': 'unknown command: ' + str(t)}


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
            data = conn.recv(4096)
            if data:
                cmd = json.loads(data.decode())
                result = bridge.handle(cmd)
                conn.send(json.dumps(result).encode())
            conn.close()
        except socket.timeout:
            continue
        except Exception as e:
            try:
                conn.send(json.dumps({'error': str(e)}).encode())
            except:
                pass
            try:
                conn.close()
            except:
                pass


if __name__ == '__main__':
    main()
