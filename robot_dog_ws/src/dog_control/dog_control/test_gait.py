#!/usr/bin/env python3
"""
Test script for gait controller.

Usage:
    ros2 run dog_control test_gait.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time


class GaitTester(Node):
    """Test node for gait controller."""
    
    def __init__(self):
        super().__init__('gait_tester')
        
        # Publishers
        self.start_stop_pub = self.create_publisher(Bool, '/gait/start_stop', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/gait/command', 10)
        
        self.get_logger().info('Gait Tester initialized')
    
    def test_trot_gait(self):
        """Run trot gait test sequence."""
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('   Trot Gait Test Sequence')
        self.get_logger().info('═══════════════════════════════════════')
        
        # Wait a bit for system to stabilize
        self.get_logger().info('Waiting 2 seconds...')
        time.sleep(2)
        
        # Test 1: Start gait
        self.get_logger().info('Test 1: Starting gait...')
        msg = Bool()
        msg.data = True
        self.start_stop_pub.publish(msg)
        time.sleep(3)
        
        # Test 2: Forward movement
        self.get_logger().info('Test 2: Forward movement...')
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        time.sleep(3)
        
        # Test 3: Stop movement
        self.get_logger().info('Test 3: Stopping movement...')
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(2)
        
        # Test 4: Stop gait
        self.get_logger().info('Test 4: Stopping gait...')
        msg.data = False
        self.start_stop_pub.publish(msg)
        time.sleep(2)
        
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('   Test sequence completed!')
        self.get_logger().info('═══════════════════════════════════════')


def main(args=None):
    rclpy.init(args=args)
    tester = GaitTester()
    
    try:
        tester.test_trot_gait()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
