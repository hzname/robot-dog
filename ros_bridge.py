#!/usr/bin/env python3
"""
ROS2 Command Bridge - runs INSIDE the robot_dog container
Listens on a Unix socket for JSON commands and executes them via rclpy
"""
import json
import os
import socket
import sys
import threading

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray

SOCKET_PATH = '/tmp/robot_dog/ros_bridge.sock'

class BridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_pub = self.create_publisher(Bool, '/servo_enable', 10)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop_trigger', 10)
        self.pos_pub = self.create_publisher(Float64MultiArray, '/joint_position_command', 10)

    def handle(self, cmd):
        t = cmd.get('type')
        if t == 'cmd_vel':
            msg = Twist()
            msg.linear.x = cmd.get('linear_x', 0.0)
            msg.linear.y = cmd.get('linear_y', 0.0)
            msg.angular.z = cmd.get('angular_z', 0.0)
            self.cmd_vel_pub.publish(msg)
            return {'ok': True}
        elif t == 'stop':
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            return {'ok': True}
        elif t == 'servo_enable':
            msg = Bool()
            msg.data = cmd.get('enable', True)
            self.servo_pub.publish(msg)
            return {'ok': True}
        elif t == 'emergency_stop':
            msg = Bool()
            msg.data = cmd.get('stop', True)
            self.estop_pub.publish(msg)
            return {'ok': True}
        elif t == 'joint_command':
            msg = Float64MultiArray()
            msg.data = cmd.get('positions', [])
            self.pos_pub.publish(msg)
            return {'ok': True}
        return {'error': 'unknown command'}


def main():
    rclpy.init()
    node = BridgeNode()
    
    # Spin in background
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    
    # Clean old socket
    os.makedirs(os.path.dirname(SOCKET_PATH), exist_ok=True)
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)
    
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(5)
    server.settimeout(1.0)
    
    print(f'Bridge listening on {SOCKET_PATH}', flush=True)
    
    while True:
        try:
            conn, _ = server.accept()
            data = conn.recv(4096)
            if data:
                cmd = json.loads(data.decode())
                result = node.handle(cmd)
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
