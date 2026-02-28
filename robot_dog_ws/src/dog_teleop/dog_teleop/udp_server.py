#!/usr/bin/env python3
"""
UDP server for remote robot dog teleoperation.

Receives control commands via UDP from a remote PC and publishes
ROS2 messages for the robot dog.

Protocol:
    JSON messages over UDP with the following format:
    
    Movement command:
    {
        "type": "velocity",
        "linear_x": float,   # -1.0 to 1.0
        "linear_y": float,   # -1.0 to 1.0
        "angular_z": float   # -1.0 to 1.0
    }
    
    Pose command:
    {
        "type": "pose",
        "pose": "stand" | "sit" | "lie"
    }
    
    Gait command:
    {
        "type": "gait",
        "active": bool
    }
    
    Speed command:
    {
        "type": "speed",
        "linear": float,     # 0.1 to 1.0
        "angular": float     # 0.1 to 1.0
    }
    
    Emergency stop:
    {
        "type": "stop"
    }

Published topics:
    - /cmd_vel (Twist)          : Velocity commands
    - /gait/start_stop (Bool)   : Start/stop trot gait
    - /body_pose (String)       : Body pose command

Usage:
    ros2 run dog_teleop udp_server
    
Parameters:
    - port (int): UDP port to listen on (default: 8888)
    - max_clients (int): Maximum number of clients (default: 4)
    - timeout_sec (float): Client timeout in seconds (default: 2.0)
    - linear_speed (float): Default linear speed (default: 0.5)
    - angular_speed (float): Default angular speed (default: 0.8)
"""

import json
import socket
import struct
import threading
import time
from typing import Dict, Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class UDPClient:
    """Represents a connected UDP client."""
    
    def __init__(self, address: Tuple[str, int], timeout_sec: float = 2.0):
        self.address = address
        self.last_seen = time.time()
        self.timeout_sec = timeout_sec
        self.active = True
    
    def update(self):
        """Update last seen timestamp."""
        self.last_seen = time.time()
        self.active = True
    
    def is_expired(self) -> bool:
        """Check if client has timed out."""
        expired = (time.time() - self.last_seen) > self.timeout_sec
        if expired and self.active:
            self.active = False
        return expired


class TeleopUDPServer(Node):
    """
    UDP server for remote robot dog teleoperation.
    
    Receives JSON commands via UDP and publishes ROS2 messages.
    Supports multiple clients with timeout handling.
    """
    
    def __init__(self):
        super().__init__('teleop_udp_server')
        
        # Declare parameters
        self.declare_parameter('port', 8888)
        self.declare_parameter('max_clients', 4)
        self.declare_parameter('timeout_sec', 2.0)
        self.declare_parameter('buffer_size', 1024)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('smoothing_factor', 0.2)
        self.declare_parameter('publish_rate', 20.0)
        
        # Load parameters
        self.port = self.get_parameter('port').value
        self.max_clients = self.get_parameter('max_clients').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.smoothing = self.get_parameter('smoothing_factor').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gait_pub = self.create_publisher(Bool, '/gait/start_stop', 10)
        self.pose_pub = self.create_publisher(String, '/body_pose', 10)
        
        # Socket setup
        self.socket: Optional[socket.socket] = None
        self.running = False
        
        # Client management
        self.clients: Dict[Tuple[str, int], UDPClient] = {}
        self.clients_lock = threading.Lock()
        
        # Current velocity targets
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        
        # Smoothed velocities
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # State
        self.gait_active = False
        self.current_pose = 'stand'
        
        # Statistics
        self.messages_received = 0
        self.messages_dropped = 0
        self.last_stats_time = time.time()
        
        # Create timers
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_cmd_vel)
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_clients)
        self.stats_timer = self.create_timer(10.0, self.print_stats)
        
        # Start UDP server thread
        self.server_thread: Optional[threading.Thread] = None
        
        self.get_logger().info('UDP Teleop Server initialized')
    
    def start(self):
        """Start the UDP server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(('0.0.0.0', self.port))
            self.socket.setblocking(False)
            
            self.running = True
            self.server_thread = threading.Thread(target=self.receive_loop)
            self.server_thread.daemon = True
            self.server_thread.start()
            
            self.get_logger().info(f'UDP server started on port {self.port}')
            self.get_logger().info(f'Max clients: {self.max_clients}, '
                                   f'Timeout: {self.timeout_sec}s')
        except Exception as e:
            self.get_logger().error(f'Failed to start UDP server: {e}')
            raise
    
    def stop(self):
        """Stop the UDP server."""
        self.running = False
        
        if self.socket:
            self.socket.close()
            self.socket = None
        
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=2.0)
        
        # Send stop commands
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        msg = Bool()
        msg.data = False
        self.gait_pub.publish(msg)
        
        self.get_logger().info('UDP server stopped')
    
    def receive_loop(self):
        """Main receive loop running in separate thread."""
        while self.running and rclpy.ok():
            try:
                # Non-blocking receive with timeout
                self.socket.settimeout(0.1)
                data, address = self.socket.recvfrom(self.buffer_size)
                
                # Process message
                self.handle_message(data, address)
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().debug(f'Receive error: {e}')
    
    def handle_message(self, data: bytes, address: Tuple[str, int]):
        """Handle incoming UDP message."""
        # Check client limit
        with self.clients_lock:
            if address not in self.clients:
                if len(self.clients) >= self.max_clients:
                    self.messages_dropped += 1
                    return
                self.clients[address] = UDPClient(address, self.timeout_sec)
                self.get_logger().info(f'New client connected: {address[0]}:{address[1]}')
            
            self.clients[address].update()
        
        # Parse JSON
        try:
            message = json.loads(data.decode('utf-8'))
            self.process_command(message)
            self.messages_received += 1
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid JSON from {address}: {e}')
        except Exception as e:
            self.get_logger().warn(f'Error processing message: {e}')
    
    def process_command(self, message: Dict[str, Any]):
        """Process a command message."""
        msg_type = message.get('type', '')
        
        if msg_type == 'velocity':
            self.handle_velocity_command(message)
        elif msg_type == 'pose':
            self.handle_pose_command(message)
        elif msg_type == 'gait':
            self.handle_gait_command(message)
        elif msg_type == 'speed':
            self.handle_speed_command(message)
        elif msg_type == 'stop':
            self.handle_stop_command()
        else:
            self.get_logger().warn(f'Unknown command type: {msg_type}')
    
    def handle_velocity_command(self, message: Dict[str, Any]):
        """Handle velocity command."""
        try:
            self.target_linear_x = float(message.get('linear_x', 0.0))
            self.target_linear_y = float(message.get('linear_y', 0.0))
            self.target_angular_z = float(message.get('angular_z', 0.0))
            
            # Clamp to [-1, 1]
            self.target_linear_x = max(-1.0, min(1.0, self.target_linear_x))
            self.target_linear_y = max(-1.0, min(1.0, self.target_linear_y))
            self.target_angular_z = max(-1.0, min(1.0, self.target_angular_z))
            
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'Invalid velocity values: {e}')
    
    def handle_pose_command(self, message: Dict[str, Any]):
        """Handle pose command."""
        pose = message.get('pose', '')
        valid_poses = ['stand', 'sit', 'lie']
        
        if pose in valid_poses:
            self.current_pose = pose
            msg = String()
            msg.data = pose
            self.pose_pub.publish(msg)
            self.get_logger().info(f'Pose set to: {pose}')
        else:
            self.get_logger().warn(f'Invalid pose: {pose}. Valid: {valid_poses}')
    
    def handle_gait_command(self, message: Dict[str, Any]):
        """Handle gait command."""
        try:
            active = bool(message.get('active', False))
            self.set_gait(active)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'Invalid gait value: {e}')
    
    def handle_speed_command(self, message: Dict[str, Any]):
        """Handle speed command."""
        try:
            linear = float(message.get('linear', self.linear_speed))
            angular = float(message.get('angular', self.angular_speed))
            
            self.linear_speed = max(0.1, min(1.0, linear))
            self.angular_speed = max(0.1, min(1.0, angular))
            
            self.get_logger().info(f'Speed set: linear={self.linear_speed:.2f}, '
                                   f'angular={self.angular_speed:.2f}')
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'Invalid speed values: {e}')
    
    def handle_stop_command(self):
        """Handle emergency stop command."""
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        
        if self.gait_active:
            self.set_gait(False)
        
        # Send immediate stop
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().warn('Emergency stop received via UDP')
    
    def set_gait(self, active: bool):
        """Set gait state."""
        self.gait_active = active
        msg = Bool()
        msg.data = active
        self.gait_pub.publish(msg)
        status = "ON" if active else "OFF"
        self.get_logger().info(f'Gait: {status}')
    
    def publish_cmd_vel(self):
        """Publish smoothed velocity commands."""
        # Apply smoothing
        self.current_linear_x = (
            self.smoothing * self.target_linear_x + 
            (1 - self.smoothing) * self.current_linear_x
        )
        self.current_linear_y = (
            self.smoothing * self.target_linear_y + 
            (1 - self.smoothing) * self.current_linear_y
        )
        self.current_angular_z = (
            self.smoothing * self.target_angular_z + 
            (1 - self.smoothing) * self.current_angular_z
        )
        
        # Create and publish Twist
        twist = Twist()
        twist.linear.x = self.current_linear_x * self.linear_speed
        twist.linear.y = self.current_linear_y * self.linear_speed
        twist.angular.z = self.current_angular_z * self.angular_speed
        
        self.cmd_vel_pub.publish(twist)
    
    def cleanup_clients(self):
        """Remove expired clients."""
        with self.clients_lock:
            expired = [
                addr for addr, client in self.clients.items() 
                if client.is_expired()
            ]
            for addr in expired:
                del self.clients[addr]
                self.get_logger().info(f'Client disconnected (timeout): {addr[0]}:{addr[1]}')
    
    def print_stats(self):
        """Print server statistics."""
        with self.clients_lock:
            active_clients = len(self.clients)
        
        elapsed = time.time() - self.last_stats_time
        rate = self.messages_received / elapsed if elapsed > 0 else 0
        
        self.get_logger().info(
            f'Stats: {active_clients} clients, '
            f'{self.messages_received} msgs, '
            f'{self.messages_dropped} dropped, '
            f'{rate:.1f} msg/s'
        )
        
        # Reset counters
        self.messages_received = 0
        self.messages_dropped = 0
        self.last_stats_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopUDPServer()
    
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
