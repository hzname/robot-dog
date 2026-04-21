#!/usr/bin/env python3
"""
Unit tests for ROS2 Bridge module.

This module provides comprehensive unit tests for the ros_bridge.py module,
covering command handling, validation, state management, and error cases.

Test Categories:
- Command validation (NaN, Infinity, wrong types)
- State management (joint positions, IMU, e-stop)
- Error handling (invalid JSON, unknown commands)
- Edge cases (empty arrays, boundary values)

Usage:
    python -m pytest tests/test_ros_bridge.py -v
    python -m unittest tests/test_ros_bridge.py

Requirements:
    pytest (optional, falls back to unittest)
    ROS2 Jazzy (mocked for testing)
"""

import json
import math
import socket
import sys
import threading
import time
import unittest
from typing import Any, Dict, List
from unittest.mock import MagicMock, Mock, patch

# Add workspace to path
sys.path.insert(0, '/workspace')


class MockROS2Modules:
    """Mock ROS2 modules for testing without ROS2 runtime."""
    
    class MockNode:
        def __init__(self):
            self.publishers = []
            self.subscriptions = []
        
        def create_publisher(self, msg_type, topic, qos):
            pub = Mock()
            self.publishers.append((msg_type, topic, qos, pub))
            return pub
        
        def create_subscription(self, msg_type, topic, callback, qos):
            sub = Mock()
            self.subscriptions.append((msg_type, topic, callback, qos, sub))
            return sub
        
        def destroy_node(self):
            pass
    
    class Twist:
        def __init__(self):
            self.linear = Mock()
            self.linear.x = 0.0
            self.linear.y = 0.0
            self.angular = Mock()
            self.angular.z = 0.0
    
    class Bool:
        def __init__(self):
            self.data = False
    
    class Float64MultiArray:
        def __init__(self):
            self.data = []
    
    class Imu:
        def __init__(self):
            self.orientation = Mock(x=0, y=0, z=0, w=1)
            self.angular_velocity = Mock(x=0, y=0, z=0)
            self.linear_acceleration = Mock(x=0, y=0, z=0)
    
    class JointState:
        def __init__(self):
            self.position = [0.0] * 12


def setup_mock_ros2():
    """Setup mock ROS2 modules before importing ros_bridge."""
    sys.modules['rclpy'] = MagicMock()
    sys.modules['rclpy.executors'] = MagicMock()
    sys.modules['rclpy.qos'] = MagicMock()
    sys.modules['geometry_msgs.msg'] = MagicMock()
    sys.modules['std_msgs.msg'] = MagicMock()
    sys.modules['sensor_msgs.msg'] = MagicMock()
    
    # Configure mocks
    mock_rclpy = sys.modules['rclpy']
    mock_rclpy.init = Mock()
    mock_rclpy.shutdown = Mock()
    mock_rclpy.ok = Mock(return_value=True)
    mock_rclpy.create_node = Mock(return_value=MockROS2Modules.MockNode())
    
    sys.modules['geometry_msgs.msg'].Twist = MockROS2Modules.Twist
    sys.modules['std_msgs.msg'].Bool = MockROS2Modules.Bool
    sys.modules['std_msgs.msg'].Float64MultiArray = MockROS2Modules.Float64MultiArray
    sys.modules['sensor_msgs.msg'].Imu = MockROS2Modules.Imu
    sys.modules['sensor_msgs.msg'].JointState = MockROS2Modules.JointState
    
    # QoS mocks
    mock_qos = sys.modules['rclpy.qos']
    mock_qos.QoSProfile = Mock()
    mock_qos.ReliabilityPolicy = Mock()
    mock_qos.ReliabilityPolicy.BEST_EFFORT = 'BEST_EFFORT'
    mock_qos.ReliabilityPolicy.RELIABLE = 'RELIABLE'


class TestBridgeCommandHandling(unittest.TestCase):
    """Test cases for Bridge.handle() method."""
    
    def setUp(self):
        """Set up test fixtures."""
        setup_mock_ros2()
        # Re-import to get fresh instance with mocks
        if 'ros_bridge' in sys.modules:
            del sys.modules['ros_bridge']
        import ros_bridge
        self.bridge_module = ros_bridge
        self.bridge = ros_bridge.Bridge()
    
    def test_cmd_vel_valid(self):
        """Test valid cmd_vel command."""
        cmd = {
            'type': 'cmd_vel',
            'linear_x': 0.5,
            'linear_y': 0.3,
            'angular_z': 0.2
        }
        result = self.bridge.handle(cmd)
        self.assertEqual(result['ok'], True)
        self.assertNotIn('error', result)
    
    def test_cmd_vel_nan_rejected(self):
        """Test that NaN values in cmd_vel are rejected."""
        cmd = {
            'type': 'cmd_vel',
            'linear_x': float('nan'),
            'linear_y': 0.3,
            'angular_z': 0.2
        }
        result = self.bridge.handle(cmd)
        self.assertIn('error', result)
        self.assertEqual(result['error'], 'cmd_vel values must be finite numbers')
    
    def test_cmd_vel_infinity_rejected(self):
        """Test that infinite values in cmd_vel are rejected."""
        cmd = {
            'type': 'cmd_vel',
            'linear_x': float('inf'),
            'linear_y': 0.3,
            'angular_z': 0.2
        }
        result = self.bridge.handle(cmd)
        self.assertIn('error', result)
        self.assertEqual(result['error'], 'cmd_vel values must be finite numbers')
    
    def test_cmd_vel_negative_infinity_rejected(self):
        """Test that negative infinity is rejected."""
        cmd = {
            'type': 'cmd_vel',
            'linear_x': float('-inf'),
            'linear_y': 0.0,
            'angular_z': 0.0
        }
        result = self.bridge.handle(cmd)
        self.assertIn('error', result)
    
    def test_stop_command(self):
        """Test stop command publishes zero velocity."""
        cmd = {'type': 'stop'}
        result = self.bridge.handle(cmd)
        self.assertEqual(result['ok'], True)
    
    def test_servo_enable_true(self):
        """Test servo enable command."""
        cmd = {'type': 'servo_enable', 'enable': True}
        result = self.bridge.handle(cmd)
        self.assertEqual(result['ok'], True)
    
    def test_servo_enable_false(self):
        """Test servo disable command."""
        cmd = {'type': 'servo_enable', 'enable': False}
        result = self.bridge.handle(cmd)
        self.assertEqual(result['ok'], True)
    
    def test_emergency_stop_trigger(self):
        """Test emergency stop trigger."""
        cmd = {'type': 'emergency_stop', 'stop': True}
        result = self.bridge.handle(cmd)
        self.assertEqual(result['ok'], True)
    
    def test_gait_enable(self):
        """Test gait enable command."""
        cmd = {'type': 'gait_enable', 'enable': True}
        result = self.bridge.handle(cmd)
        self.assertEqual(result['ok'], True)
    
    def test_joint_command_valid(self):
        """Test valid joint command with 12 positions."""
        cmd = {
            'type': 'joint_command',
            'positions': [0.0] * 12
        }
        result = self.bridge.handle(cmd)
        self.assertEqual(result['ok'], True)
    
    def test_joint_command_wrong_length(self):
        """Test joint command with wrong number of positions."""
        cmd = {
            'type': 'joint_command',
            'positions': [0.0] * 11  # Should be 12
        }
        result = self.bridge.handle(cmd)
        self.assertIn('error', result)
        self.assertIn('12 positions', result['error'])
    
    def test_joint_command_nan_rejected(self):
        """Test joint command with NaN value is rejected."""
        positions = [0.0] * 12
        positions[5] = float('nan')
        cmd = {
            'type': 'joint_command',
            'positions': positions
        }
        result = self.bridge.handle(cmd)
        self.assertIn('error', result)
        self.assertIn('finite number', result['error'])
    
    def test_joint_command_infinity_rejected(self):
        """Test joint command with infinity is rejected."""
        positions = [0.0] * 12
        positions[3] = float('inf')
        cmd = {
            'type': 'joint_command',
            'positions': positions
        }
        result = self.bridge.handle(cmd)
        self.assertIn('error', result)
    
    def test_get_state(self):
        """Test get_state command returns current state."""
        cmd = {'type': 'get_state'}
        result = self.bridge.handle(cmd)
        self.assertEqual(result['ok'], True)
        self.assertIn('joint_positions', result)
        self.assertIn('emergency_stopped', result)
        self.assertIsInstance(result['joint_positions'], list)
        self.assertEqual(len(result['joint_positions']), 12)
    
    def test_get_imu_no_data(self):
        """Test get_imu when no data available."""
        cmd = {'type': 'get_imu'}
        result = self.bridge.handle(cmd)
        self.assertEqual(result['ok'], False)
        self.assertIn('error', result)
    
    def test_unknown_command(self):
        """Test unknown command type."""
        cmd = {'type': 'unknown_command'}
        result = self.bridge.handle(cmd)
        self.assertIn('error', result)
        self.assertIn('unknown command', result['error'])
    
    def test_empty_command(self):
        """Test command without type field."""
        cmd = {}
        result = self.bridge.handle(cmd)
        self.assertIn('error', result)


class TestBridgeCallbacks(unittest.TestCase):
    """Test cases for Bridge callback methods."""
    
    def setUp(self):
        """Set up test fixtures."""
        setup_mock_ros2()
        if 'ros_bridge' in sys.modules:
            del sys.modules['ros_bridge']
        import ros_bridge
        self.bridge = ros_bridge.Bridge()
    
    def test_on_joints_valid(self):
        """Test _on_joints callback with valid data."""
        msg = MockROS2Modules.JointState()
        msg.position = [0.1 * i for i in range(12)]
        self.bridge._on_joints(msg)
        self.assertEqual(len(self.bridge.joint_positions), 12)
        self.assertAlmostEqual(self.bridge.joint_positions[5], 0.5)
    
    def test_on_joints_invalid_length(self):
        """Test _on_joints callback with invalid length."""
        msg = MockROS2Modules.JointState()
        msg.position = [0.1] * 11  # Wrong length
        initial_positions = self.bridge.joint_positions.copy()
        self.bridge._on_joints(msg)
        # Should not update on invalid length
        self.assertEqual(self.bridge.joint_positions, initial_positions)
    
    def test_on_estop_activate(self):
        """Test _on_estop callback activating e-stop."""
        msg = MockROS2Modules.Bool()
        msg.data = True
        self.bridge._on_estop(msg)
        self.assertTrue(self.bridge.emergency_stopped)
    
    def test_on_estop_deactivate(self):
        """Test _on_estop callback deactivating e-stop."""
        # First activate
        msg = MockROS2Modules.Bool()
        msg.data = True
        self.bridge._on_estop(msg)
        self.assertTrue(self.bridge.emergency_stopped)
        
        # Then deactivate
        msg.data = False
        self.bridge._on_estop(msg)
        self.assertFalse(self.bridge.emergency_stopped)
    
    def test_on_imu(self):
        """Test _on_imu callback stores IMU data."""
        msg = MockROS2Modules.Imu()
        msg.orientation.w = 0.98
        msg.angular_velocity.z = 0.5
        msg.linear_acceleration.x = 9.81
        
        self.bridge._on_imu(msg)
        
        self.assertIsNotNone(self.bridge.latest_imu)
        self.assertEqual(self.bridge.latest_imu['orientation']['w'], 0.98)
        self.assertEqual(self.bridge.latest_imu['angular_velocity']['z'], 0.5)
        self.assertEqual(self.bridge.latest_imu['linear_acceleration']['x'], 9.81)


class TestHandleConnection(unittest.TestCase):
    """Test cases for _handle_connection function."""
    
    def setUp(self):
        """Set up test fixtures."""
        setup_mock_ros2()
        if 'ros_bridge' in sys.modules:
            del sys.modules['ros_bridge']
        import ros_bridge
        self.bridge = ros_bridge.Bridge()
        self.ros_bridge = sys.modules['ros_bridge']
    
    def test_valid_command_roundtrip(self):
        """Test sending a command and receiving response."""
        # Create mock socket
        mock_socket = Mock()
        
        # Setup recv to return a valid command
        cmd = {'type': 'get_state'}
        cmd_bytes = json.dumps(cmd).encode('utf-8')
        header = len(cmd_bytes).to_bytes(4, 'big')
        
        recv_data = header + cmd_bytes
        mock_socket.recv = Mock(side_effect=[
            recv_data[:4],  # Header
            recv_data[4:],  # Payload
            b''  # EOF
        ])
        
        # Call handler
        self.ros_bridge._handle_connection(self.bridge, mock_socket)
        
        # Verify send was called with response
        self.assertTrue(mock_socket.send.called)
        self.assertTrue(mock_socket.close.called)
    
    def test_oversized_message(self):
        """Test rejection of oversized messages."""
        mock_socket = Mock()
        
        # Create header with oversized length
        oversized_len = self.ros_bridge.SOCKET_MAX_MESSAGE_SIZE + 1
        header = oversized_len.to_bytes(4, 'big')
        
        mock_socket.recv = Mock(return_value=header)
        
        self.ros_bridge._handle_connection(self.bridge, mock_socket)
        
        # Verify error response was sent
        self.assertTrue(mock_socket.send.called)
        call_args = mock_socket.send.call_args[0][0]
        response = json.loads(call_args[4:].decode('utf-8'))
        self.assertIn('error', response)
        self.assertIn('too large', response['error'])
    
    def test_invalid_json(self):
        """Test handling of invalid JSON."""
        mock_socket = Mock()
        
        invalid_json = b'{invalid json}'
        header = len(invalid_json).to_bytes(4, 'big')
        
        mock_socket.recv = Mock(side_effect=[
            header,
            invalid_json,
            b''
        ])
        
        self.ros_bridge._handle_connection(self.bridge, mock_socket)
        
        # Verify error response
        self.assertTrue(mock_socket.send.called)
        call_args = mock_socket.send.call_args[0][0]
        response = json.loads(call_args[4:].decode('utf-8'))
        self.assertIn('error', response)
        self.assertIn('JSON', response['error'])
    
    def test_client_disconnect_during_header(self):
        """Test handling of client disconnect during header read."""
        mock_socket = Mock()
        mock_socket.recv = Mock(return_value=b'')  # Immediate disconnect
        
        self.ros_bridge._handle_connection(self.bridge, mock_socket)
        
        # Should close without error
        self.assertTrue(mock_socket.close.called)
    
    def test_client_disconnect_during_payload(self):
        """Test handling of client disconnect during payload read."""
        mock_socket = Mock()
        
        # Send partial payload then disconnect
        cmd = '{"type": "get_state"'  # Incomplete JSON
        header = len(cmd).to_bytes(4, 'big')
        
        mock_socket.recv = Mock(side_effect=[
            header,
            cmd.encode(),
            b''  # Disconnect
        ])
        
        self.ros_bridge._handle_connection(self.bridge, mock_socket)
        
        # Should close gracefully
        self.assertTrue(mock_socket.close.called)


class TestTopicPrefix(unittest.TestCase):
    """Test cases for topic prefix functionality."""
    
    def test_topic_prefix_empty(self):
        """Test topic prefix with empty string."""
        import os
        os.environ['DOG_TOPIC_PREFIX'] = ''
        
        setup_mock_ros2()
        if 'ros_bridge' in sys.modules:
            del sys.modules['ros_bridge']
        import ros_bridge
        
        self.assertEqual(ros_bridge._t('/cmd_vel'), '/cmd_vel')
    
    def test_topic_prefix_with_namespace(self):
        """Test topic prefix with namespace."""
        import os
        os.environ['DOG_TOPIC_PREFIX'] = '/dog'
        
        setup_mock_ros2()
        if 'ros_bridge' in sys.modules:
            del sys.modules['ros_bridge']
        import ros_bridge
        
        self.assertEqual(ros_bridge._t('/cmd_vel'), '/dog/cmd_vel')
    
    def test_topic_prefix_multiple_levels(self):
        """Test topic prefix with multiple levels."""
        import os
        os.environ['DOG_TOPIC_PREFIX'] = '/robot/dog'
        
        setup_mock_ros2()
        if 'ros_bridge' in sys.modules:
            del sys.modules['ros_bridge']
        import ros_bridge
        
        self.assertEqual(ros_bridge._t('/cmd_vel'), '/robot/dog/cmd_vel')


class TestConstants(unittest.TestCase):
    """Test cases for module constants."""
    
    def test_socket_max_message_size(self):
        """Test SOCKET_MAX_MESSAGE_SIZE constant."""
        setup_mock_ros2()
        if 'ros_bridge' in sys.modules:
            del sys.modules['ros_bridge']
        import ros_bridge
        
        self.assertEqual(ros_bridge.SOCKET_MAX_MESSAGE_SIZE, 65536)
    
    def test_connection_backlog(self):
        """Test CONNECTION_BACKLOG constant."""
        setup_mock_ros2()
        if 'ros_bridge' in sys.modules:
            del sys.modules['ros_bridge']
        import ros_bridge
        
        self.assertEqual(ros_bridge.CONNECTION_BACKLOG, 5)
    
    def test_socket_timeout(self):
        """Test SOCKET_TIMEOUT constant."""
        setup_mock_ros2()
        if 'ros_bridge' in sys.modules:
            del sys.modules['ros_bridge']
        import ros_bridge
        
        self.assertEqual(ros_bridge.SOCKET_TIMEOUT, 1.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
