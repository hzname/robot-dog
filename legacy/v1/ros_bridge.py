#!/usr/bin/env python3
"""
ROS2 Command Bridge - runs INSIDE the RobotDogQwen container.

This module provides a bidirectional bridge between external clients (web UI, CLI tools)
and ROS2 topics. It listens on a Unix domain socket for JSON-encoded commands and
publishes them to appropriate ROS2 topics. It also subscribes to sensor feedback
(joint states, IMU, emergency stop status) and returns it to clients.

Features:
- Length-prefix framing for reliable message parsing
- Thread-safe connection handling (one thread per client)
- QoS configuration for real-time sensor data (BEST_EFFORT for high-frequency data)
- NaN validation for all numeric inputs
- Graceful shutdown with proper resource cleanup

Example command format (JSON):
    {"type": "cmd_vel", "linear_x": 0.5, "angular_z": 0.3}
    {"type": "joint_command", "positions": [0.0, 0.1, ...]}  # 12 values
    {"type": "get_state"}
    {"type": "get_imu"}

Topic mapping:
    cmd_vel -> /cmd_vel (geometry_msgs/Twist)
    servo_enable -> /servo_enable (std_msgs/Bool)
    emergency_stop -> /emergency_stop_trigger (std_msgs/Bool)
    joint_command -> /joint_position_command (std_msgs/Float64MultiArray)
    gait_enable -> /gait_enable (std_msgs/Bool)

Subscriptions:
    /joint_states (sensor_msgs/JointState) - Current joint positions
    /imu/data (sensor_msgs/Imu) - IMU orientation and acceleration
    /emergency_stop (std_msgs/Bool) - Emergency stop state

Author: RobotDogQwen Team
License: MIT
"""

import json
import logging
import math
import os
import signal
import socket
import sys
import threading
from typing import Any, Dict, List, Optional

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')

import rclpy
from rclpy import executors
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Configuration constants
SOCKET_PATH = '/tmp/robot_dog/ros_bridge.sock'
SOCKET_MAX_MESSAGE_SIZE = 65536  # Maximum JSON payload size in bytes
CONNECTION_BACKLOG = 5  # Maximum queued connections
SOCKET_TIMEOUT = 1.0  # Socket accept timeout in seconds

# QoS profiles for different data types
SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
RELIABLE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

# Topic prefix — set DOG_TOPIC_PREFIX=/dog for namespaced topics, or leave empty for flat topics
TOPIC_PREFIX = os.environ.get('DOG_TOPIC_PREFIX', '')

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('ros_bridge')


def _t(name: str) -> str:
    """
    Prefix a topic name with the configured namespace.
    
    Args:
        name: The base topic name (e.g., '/cmd_vel')
    
    Returns:
        The fully qualified topic name (e.g., '/dog/cmd_vel' or '/cmd_vel')
    """
    return f'{TOPIC_PREFIX}{name}'

class Bridge:
    """
    ROS2 Bridge that connects Unix socket commands to ROS2 topics.
    
    This class manages publishers for command topics and subscribers for sensor feedback.
    It maintains internal state for joint positions, IMU data, and emergency stop status,
    which can be queried by clients via the 'get_state' and 'get_imu' commands.
    
    Thread Safety:
        - All callbacks (_on_imu, _on_joints, _on_estop) are thread-safe
        - State variables are simple types (list, dict, bool) with atomic updates
        - No locks needed for read/write due to Python's GIL and simple data structures
    
    Attributes:
        node: ROS2 node instance
        cmd_vel_pub: Publisher for velocity commands
        servo_pub: Publisher for servo enable/disable
        estop_pub: Publisher for emergency stop trigger
        pos_pub: Publisher for joint position commands
        gait_enable_pub: Publisher for gait enable/disable
        joint_positions: List of 12 joint positions (radians)
        emergency_stopped: Boolean flag for emergency stop state
        latest_imu: Dictionary with IMU orientation, velocity, and acceleration
    """
    
    def __init__(self):
        """
        Initialize the ROS2 bridge with all publishers and subscribers.
        
        Sets up:
        - 5 publishers for robot commands (velocity, servo, e-stop, joints, gait)
        - 3 subscribers for sensor feedback (joints, e-stop state, IMU)
        - Internal state variables initialized to safe defaults
        """
        rclpy.init()
        self.node = rclpy.create_node('web_bridge')
        
        # Command publishers with reliable QoS for critical commands
        self.cmd_vel_pub = self.node.create_publisher(Twist, _t('/cmd_vel'), 10)
        self.servo_pub = self.node.create_publisher(Bool, _t('/servo_enable'), 10)
        self.estop_pub = self.node.create_publisher(Bool, _t('/emergency_stop_trigger'), 10)
        self.pos_pub = self.node.create_publisher(Float64MultiArray, _t('/joint_position_command'), 10)
        self.gait_enable_pub = self.node.create_publisher(Bool, _t('/gait_enable'), 10)

        # Subscribe to joint states for feedback (BestEffort QoS for high-frequency data)
        self.joint_positions: List[float] = [0.0] * 12
        self.node.create_subscription(JointState, _t('/joint_states'), self._on_joints, SENSOR_QOS)
        logger.debug("Subscribed to /joint_states")

        # Subscribe to emergency stop state
        self.emergency_stopped: bool = False
        self.node.create_subscription(Bool, _t('/emergency_stop'), self._on_estop, RELIABLE_QOS)
        logger.debug("Subscribed to /emergency_stop")

        # Subscribe to IMU data (BestEffort QoS for high-frequency sensor data)
        self.latest_imu: Optional[Dict[str, Any]] = None
        self.node.create_subscription(Imu, _t('/imu/data'), self._on_imu, SENSOR_QOS)
        logger.debug("Subscribed to /imu/data")
        
        logger.info("ROS2 Bridge initialized successfully")

    def _on_imu(self, msg: Imu) -> None:
        """
        Callback for IMU data messages.
        
        Stores the latest IMU reading including orientation (quaternion),
        angular velocity, and linear acceleration. Used for balance control
        and state reporting to clients.
        
        Args:
            msg: IMU message from sensor driver
        """
        self.latest_imu = {
            'orientation': {
                'x': msg.orientation.x, 
                'y': msg.orientation.y,
                'z': msg.orientation.z, 
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x, 
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x, 
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }

    def _on_joints(self, msg: JointState) -> None:
        """
        Callback for joint state messages.
        
        Updates the internal cache of joint positions. Validates that exactly
        12 joint values are present before updating to prevent corruption.
        
        Args:
            msg: JointState message with position array
        """
        if len(msg.position) == 12:
            self.joint_positions = list(msg.position)
            logger.debug(f"Updated joint positions: {[f'{p:.3f}' for p in self.joint_positions]}")
        else:
            logger.warning(f"Invalid joint state length: {len(msg.position)}, expected 12")

    def _on_estop(self, msg: Bool) -> None:
        """
        Callback for emergency stop state messages.
        
        Updates the internal emergency stop flag. When True, all motion
        commands should be blocked until the flag is cleared.
        
        Args:
            msg: Boolean message indicating emergency stop state
        """
        prev_state = self.emergency_stopped
        self.emergency_stopped = msg.data
        if self.emergency_stopped and not prev_state:
            logger.warning("EMERGENCY STOP ACTIVATED")
        elif not self.emergency_stopped and prev_state:
            logger.info("Emergency stop cleared")

    def handle(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process an incoming command and execute the appropriate action.
        
        This is the main command dispatcher that handles all client requests.
        Each command type is validated before execution to prevent invalid data
        from reaching ROS2 topics.
        
        Command types:
        - 'cmd_vel': Set robot velocity (linear_x, linear_y, angular_z)
        - 'stop': Stop all motion (publish zero velocity)
        - 'servo_enable': Enable/disable servo motors (enable: bool)
        - 'emergency_stop': Trigger emergency stop (stop: bool)
        - 'gait_enable': Enable/disable gait controller (enable: bool)
        - 'joint_command': Set joint positions (positions: list of 12 floats)
        - 'get_state': Get current state (joint positions, e-stop status)
        - 'get_imu': Get latest IMU reading
        
        Args:
            cmd: Dictionary with 'type' key and command-specific parameters
        
        Returns:
            Dictionary with 'ok' key on success, or 'error' key on failure
        
        Raises:
            No exceptions are raised; all errors are returned as error dictionaries
        """
        t = cmd.get('type')
        
        if t == 'cmd_vel':
            msg = Twist()
            msg.linear.x = float(cmd.get('linear_x', 0.0))
            msg.linear.y = float(cmd.get('linear_y', 0.0))
            msg.angular.z = float(cmd.get('angular_z', 0.0))
            
            # Reject NaN and infinite values to prevent undefined behavior
            if not (math.isfinite(msg.linear.x) and
                    math.isfinite(msg.linear.y) and
                    math.isfinite(msg.angular.z)):
                logger.warning(f"Rejected cmd_vel with non-finite values: {msg}")
                return {'error': 'cmd_vel values must be finite numbers'}
            
            self.cmd_vel_pub.publish(msg)
            logger.debug(f"Published cmd_vel: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, z={msg.angular.z:.2f}")
            return {'ok': True}
        
        elif t == 'stop':
            self.cmd_vel_pub.publish(Twist())
            logger.info("Velocity stopped")
            return {'ok': True}
        
        elif t == 'servo_enable':
            enable = bool(cmd.get('enable', True))
            msg = Bool()
            msg.data = enable
            self.servo_pub.publish(msg)
            logger.info(f"Servo enable: {enable}")
            return {'ok': True}
        
        elif t == 'emergency_stop':
            stop = bool(cmd.get('stop', True))
            msg = Bool()
            msg.data = stop
            self.estop_pub.publish(msg)
            logger.warning(f"Emergency stop triggered: {stop}")
            return {'ok': True}
        
        elif t == 'gait_enable':
            enable = bool(cmd.get('enable', True))
            msg = Bool()
            msg.data = enable
            self.gait_enable_pub.publish(msg)
            logger.info(f"Gait enable: {enable}")
            return {'ok': True}
        
        elif t == 'joint_command':
            positions = cmd.get('positions', [])
            if len(positions) != 12:
                logger.warning(f"Invalid joint_command length: {len(positions)}, expected 12")
                return {'error': f'joint_command requires exactly 12 positions, got {len(positions)}'}
            
            # Validate all positions are finite numbers
            for i, pos in enumerate(positions):
                if not math.isfinite(float(pos)):
                    logger.warning(f"Joint {i} has non-finite value: {pos}")
                    return {'error': f'joint_command position {i} must be a finite number'}
            
            msg = Float64MultiArray()
            msg.data = [float(v) for v in positions]
            self.pos_pub.publish(msg)
            logger.debug(f"Published joint_command: {[f'{p:.3f}' for p in positions]}")
            return {'ok': True}
        
        elif t == 'get_state':
            return {
                'ok': True,
                'joint_positions': self.joint_positions,
                'emergency_stopped': self.emergency_stopped
            }
        
        elif t == 'get_imu':
            if self.latest_imu:
                return {'ok': True, 'imu': self.latest_imu}
            logger.debug("IMU data requested but not yet available")
            return {'ok': False, 'error': 'No IMU data yet'}
        
        else:
            logger.warning(f"Unknown command type: {t}")
            return {'error': f'unknown command: {t}'}


def _handle_connection(bridge: Bridge, conn: socket.socket) -> None:
    """
    Handle a single client connection in its own thread.
    
    This function implements a request-response protocol with length-prefix framing:
    1. Read 4-byte header (big-endian uint32) containing payload length
    2. Validate payload length against maximum (64KB)
    3. Read JSON payload
    4. Process command via bridge.handle()
    5. Send response with same length-prefix framing
    
    Thread Safety:
        - Each connection runs in its own thread
        - Bridge methods are thread-safe for concurrent access
        - Socket operations are isolated per connection
    
    Error Handling:
        - JSON decode errors return error response
        - Network errors are logged and connection is closed
        - Oversized messages are rejected without reading full payload
    
    Args:
        bridge: Bridge instance for command processing
        conn: Client socket connection
    """
    try:
        # Read full message with length-prefix framing
        # First 4 bytes = message length (big-endian), rest = JSON payload
        header = b''
        while len(header) < 4:
            chunk = conn.recv(4 - len(header))
            if not chunk:
                logger.debug("Client disconnected during header read")
                return
            header += chunk
        
        msg_len = int.from_bytes(header, 'big')
        
        # Validate message size before reading payload
        if msg_len > SOCKET_MAX_MESSAGE_SIZE:
            logger.warning(f"Message too large: {msg_len} bytes (max: {SOCKET_MAX_MESSAGE_SIZE})")
            response = json.dumps({'error': f'message too large: {msg_len} > {SOCKET_MAX_MESSAGE_SIZE}'}).encode()
            conn.send(len(response).to_bytes(4, 'big') + response)
            return
        
        # Read payload
        data = b''
        while len(data) < msg_len:
            chunk = conn.recv(msg_len - len(data))
            if not chunk:
                logger.debug("Client disconnected during payload read")
                break
            data += chunk
        
        if not data:
            return
        
        # Parse and process command
        cmd = json.loads(data.decode('utf-8'))
        logger.debug(f"Received command: {cmd.get('type', 'unknown')}")
        result = bridge.handle(cmd)
        
        # Send response
        response = json.dumps(result).encode('utf-8')
        conn.send(len(response).to_bytes(4, 'big') + response)
        logger.debug(f"Sent response: {result.get('ok', False)}")
        
    except json.JSONDecodeError as e:
        logger.warning(f"Invalid JSON from client: {e}")
        try:
            response = json.dumps({'error': 'invalid JSON format'}).encode()
            conn.send(len(response).to_bytes(4, 'big') + response)
        except Exception:
            pass
    except Exception as e:
        logger.error(f"Connection handler error: {type(e).__name__}: {e}", exc_info=True)
        try:
            response = json.dumps({'error': f'{type(e).__name__}: {str(e)}'}).encode()
            conn.send(len(response).to_bytes(4, 'big') + response)
        except Exception:
            pass
    finally:
        conn.close()
        logger.debug("Connection closed")


def main() -> None:
    """
    Main entry point for the ROS2 Bridge service.
    
    Initializes the ROS2 bridge, sets up Unix socket server, and starts
    accepting client connections. Implements graceful shutdown on SIGINT/SIGTERM.
    
    Lifecycle:
    1. Initialize ROS2 and create Bridge instance
    2. Start ROS2 executor in background thread
    3. Create and bind Unix domain socket
    4. Accept client connections in main loop
    5. On shutdown signal: cleanup resources in order
    
    Graceful Shutdown Sequence:
    1. Set shutdown flag to stop accepting new connections
    2. Close server socket to unblock accept()
    3. Stop ROS2 executor and shutdown ROS2
    4. Remove Unix socket file
    5. Log completion message
    
    Signal Handling:
        - SIGINT (Ctrl+C): Initiate graceful shutdown
        - SIGTERM: Initiate graceful shutdown (systemd, Docker)
    
    Environment Variables:
        DOG_TOPIC_PREFIX: Optional namespace prefix for topics (default: empty)
    
    Exit Codes:
        0: Normal shutdown
        1: Initialization error (ROS2, socket)
    """
    bridge: Optional[Bridge] = None
    server: Optional[socket.socket] = None
    shutdown_event = threading.Event()
    
    def signal_handler(signum, frame):
        """Handle shutdown signals gracefully."""
        sig_name = 'SIGINT' if signum == signal.SIGINT else 'SIGTERM'
        logger.info(f"Received {sig_name}, initiating graceful shutdown...")
        shutdown_event.set()
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Initialize bridge and ROS2
        logger.info("Initializing ROS2 Bridge...")
        bridge = Bridge()

        # Start ROS2 executor in background thread
        executor = executors.SingleThreadedExecutor()
        executor.add_node(bridge.node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        logger.debug("ROS2 executor started")

        # Setup Unix socket
        os.makedirs(os.path.dirname(SOCKET_PATH), exist_ok=True)
        if os.path.exists(SOCKET_PATH):
            logger.info(f"Removing stale socket file: {SOCKET_PATH}")
            os.unlink(SOCKET_PATH)

        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(SOCKET_PATH)
        server.listen(CONNECTION_BACKLOG)
        server.settimeout(SOCKET_TIMEOUT)
        os.chmod(SOCKET_PATH, 0o666)

        logger.info(f"Bridge listening on {SOCKET_PATH}")
        logger.info("Press Ctrl+C to shutdown")

        # Main accept loop
        while rclpy.ok() and not shutdown_event.is_set():
            try:
                conn, addr = server.accept()
                logger.debug(f"New connection from {addr}")
                t = threading.Thread(
                    target=_handle_connection, 
                    args=(bridge, conn), 
                    daemon=True,
                    name=f"conn-{conn.fileno()}"
                )
                t.start()
            except socket.timeout:
                continue
            except OSError as e:
                if shutdown_event.is_set():
                    # Expected during shutdown when socket is closed
                    logger.debug("Socket closed during shutdown")
                    break
                logger.error(f"Socket accept error: {e}")
            except Exception as e:
                logger.error(f"Unexpected accept error: {type(e).__name__}: {e}", exc_info=True)

    except Exception as e:
        logger.error(f"Initialization failed: {type(e).__name__}: {e}", exc_info=True)
        return 1
    
    finally:
        # Graceful shutdown sequence
        logger.info("Shutting down ROS2 Bridge...")
        
        # 1. Close server socket to stop accepting connections
        if server:
            try:
                server.close()
                logger.debug("Server socket closed")
            except Exception as e:
                logger.warning(f"Error closing server socket: {e}")
        
        # 2. Remove Unix socket file
        if os.path.exists(SOCKET_PATH):
            try:
                os.unlink(SOCKET_PATH)
                logger.info(f"Removed socket file: {SOCKET_PATH}")
            except Exception as e:
                logger.warning(f"Error removing socket file: {e}")
        
        # 3. Stop ROS2 executor and shutdown
        if bridge and hasattr(bridge, 'node'):
            try:
                executor.shutdown()
                bridge.node.destroy_node()
                rclpy.shutdown()
                logger.info("ROS2 shutdown complete")
            except Exception as e:
                logger.warning(f"Error during ROS2 shutdown: {e}")
        
        logger.info("ROS2 Bridge shutdown complete")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
