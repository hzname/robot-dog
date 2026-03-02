#!/usr/bin/env python3
"""
ROS2 Bridge for Rust Nodes
Reads YAML output from Rust nodes and publishes to ROS2 topics
"""

import subprocess
import sys
import os
import yaml
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
import threading


def get_rust_binary_path(package_name, binary_name):
    """Get path to Rust binary using ROS2 package paths"""
    # Try ros2 pkg prefix first
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', package_name],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            install_dir = result.stdout.strip()
            binary_path = os.path.join(install_dir, 'lib', package_name, binary_name)
            if os.path.exists(binary_path):
                return binary_path
    except Exception:
        pass
    
    # Fallback to workspace install
    ws_install_paths = [
        os.path.expanduser('~/robot_dog_ws/install'),
        os.path.expanduser('~/dev-workspace/robot_dog_ws/install'),
        '/opt/ros/humble',
    ]
    
    for ws_path in ws_install_paths:
        binary_path = os.path.join(ws_path, 'lib', package_name, binary_name)
        if os.path.exists(binary_path):
            return binary_path
    
    # Fallback to build directory
    ws_build_paths = [
        os.path.expanduser('~/robot_dog_ws/build'),
        os.path.expanduser('~/dev-workspace/robot_dog_ws/build'),
    ]
    
    for ws_path in ws_build_paths:
        binary_path = os.path.join(ws_path, package_name, binary_name)
        if os.path.exists(binary_path):
            return binary_path
    
    # Last resort: try in PATH
    return binary_name


class RustBridgeNode(Node):
    def __init__(self):
        super().__init__('rust_bridge')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Start threads for each Rust node
        self.running = True
        self.threads = []
        
    def run_gait_bridge(self):
        """Bridge gait controller output to /joint_states"""
        binary_path = get_rust_binary_path('dog_control_rust', 'gait_controller_rust')
        self.get_logger().info(f'Starting gait controller: {binary_path}')
        
        process = subprocess.Popen(
            [binary_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=False,
            text=True
        )
        
        buffer = []
        while self.running:
            line = process.stdout.readline()
            if not line:
                break
                
            if line.strip() == '---':
                # Parse accumulated YAML
                if buffer:
                    try:
                        yaml_text = ''.join(buffer)
                        data = yaml.safe_load(yaml_text)
                        if data and 'position' in data:
                            msg = JointState()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = data.get('frame_id', 'base_link')
                            msg.name = data.get('name', [])
                            msg.position = [float(p) for p in data.get('position', [])]
                            msg.velocity = [float(v) for v in data.get('velocity', [])]
                            msg.effort = [float(e) for e in data.get('effort', [])]
                            self.joint_pub.publish(msg)
                    except Exception as e:
                        self.get_logger().debug(f'Parse error: {e}')
                buffer = []
            else:
                buffer.append(line)
    
    def run_imu_bridge(self):
        """Bridge IMU output to /imu/data"""
        binary_path = get_rust_binary_path('dog_sensors_rust', 'imu_node_rust')
        self.get_logger().info(f'Starting IMU node: {binary_path}')
        
        process = subprocess.Popen(
            [binary_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=False,
            text=True
        )
        
        while self.running:
            line = process.stdout.readline()
            if not line:
                break
                
            # Parse simple IMU output: "Roll: X | Pitch: Y | Yaw: Z"
            if 'Roll:' in line and 'Pitch:' in line:
                try:
                    parts = line.strip().split('|')
                    roll = float(parts[0].split(':')[1].strip())
                    pitch = float(parts[1].split(':')[1].strip())
                    yaw = float(parts[2].split(':')[1].strip())
                    
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'imu_link'
                    
                    # Convert euler to quaternion
                    cy = math.cos(yaw * 0.5)
                    sy = math.sin(yaw * 0.5)
                    cp = math.cos(pitch * 0.5)
                    sp = math.sin(pitch * 0.5)
                    cr = math.cos(roll * 0.5)
                    sr = math.sin(roll * 0.5)
                    
                    msg.orientation.w = cr * cp * cy + sr * sp * sy
                    msg.orientation.x = sr * cp * cy - cr * sp * sy
                    msg.orientation.y = cr * sp * cy + sr * cp * sy
                    msg.orientation.z = cr * cp * sy - sr * sp * cy
                    
                    self.imu_pub.publish(msg)
                except Exception as e:
                    self.get_logger().debug(f'Parse error: {e}')
    
    def start(self):
        """Start all bridge threads"""
        self.threads = [
            threading.Thread(target=self.run_gait_bridge),
            threading.Thread(target=self.run_imu_bridge),
        ]
        
        for t in self.threads:
            t.start()
            
        self.get_logger().info('Rust bridge started')
        self.get_logger().info('Publishing /joint_states and /imu/data')
    
    def stop(self):
        """Stop all threads"""
        self.running = False
        for t in self.threads:
            t.join(timeout=1.0)


def main():
    rclpy.init()
    node = RustBridgeNode()
    
    try:
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
