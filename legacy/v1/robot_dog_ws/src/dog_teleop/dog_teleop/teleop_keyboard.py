#!/usr/bin/env python3
"""
Keyboard teleoperation node for the robot dog.

Controls:
    w/s      : Move forward/backward
    a/d      : Strafe left/right  
    q/e      : Rotate left/right (yaw)
    space    : Emergency stop (zero velocity)
    
    1        : Stand pose
    2        : Sit pose
    3        : Lie down pose
    
    g        : Toggle trot gait on/off
    
    +/- or =/: Increase/decrease speed
    
    h        : Show help
    Ctrl+C   : Quit

Published topics:
    - /cmd_vel (Twist)          : Velocity commands
    - /gait/start_stop (Bool)   : Start/stop trot gait
    - /body_pose (String)       : Body pose command (stand/sit/lie)
"""

import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class TeleopKeyboard(Node):
    """
    Keyboard teleoperation for robot dog with full control support.
    
    Features:
    - WASD movement control
    - Q/E rotation
    - Pose switching (stand/sit/lie)
    - Gait toggle (trot)
    - Speed adjustment
    - Smooth velocity ramping
    """
    
    # Movement key bindings: (linear_x, linear_y, angular_z)
    MOVE_BINDINGS = {
        'w': (1.0, 0.0, 0.0),   # forward
        's': (-1.0, 0.0, 0.0),  # backward
        'a': (0.0, 1.0, 0.0),   # strafe left
        'd': (0.0, -1.0, 0.0),  # strafe right
        'q': (0.0, 0.0, 1.0),   # rotate left
        'e': (0.0, 0.0, -1.0),  # rotate right
    }
    
    # Pose bindings
    POSE_BINDINGS = {
        '1': 'stand',
        '2': 'sit',
        '3': 'lie',
    }
    
    # Speed adjustment step
    SPEED_STEP = 0.1
    MIN_SPEED = 0.1
    MAX_SPEED = 1.0
    
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gait_pub = self.create_publisher(Bool, '/gait/start_stop', 10)
        self.pose_pub = self.create_publisher(String, '/body_pose', 10)
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('smoothing_factor', 0.2)
        self.declare_parameter('update_rate', 20.0)  # Hz
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.smoothing = self.get_parameter('smoothing_factor').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Current target velocities (from keyboard input)
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        
        # Smoothed current velocities
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # State
        self.gait_active = False
        self.current_pose = 'stand'
        self.running = True
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Timer for publishing at fixed rate
        self.timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(self.timer_period, self.update)
        
        self.get_logger().info('Teleop keyboard node started')
        self.print_help()
    
    def print_help(self):
        """Print usage instructions."""
        msg = """
╔════════════════════════════════════════════════════════════════╗
║           Robot Dog Teleop - Keyboard Control                  ║
╠════════════════════════════════════════════════════════════════╣
║ Movement:           │ Poses:           │ Gait Control:         ║
║   w/s : Forward/Back│   1 : Stand      │   g : Toggle trot     ║
║   a/d : Strafe L/R  │   2 : Sit        │                       ║
║   q/e : Rotate L/R  │   3 : Lie down   │ Speed Control:        ║
║                     │                  │   +/= : Increase      ║
║   space : Stop      │                  │   -/_ : Decrease      ║
║                     │                  │                       ║
║   h : Show help     │   Ctrl+C : Quit  │ Current: {:.1f}        ║
╚════════════════════════════════════════════════════════════════╝
""".format(self.linear_speed)
        print(msg)
    
    def get_key(self, timeout: float = 0.1) -> str:
        """Read a single keypress from stdin with timeout."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = ''
        if rlist:
            key = sys.stdin.read(1)
            # Handle escape sequences (arrow keys)
            if key == '\x1b':
                key += sys.stdin.read(2) if select.select([sys.stdin], [], [], 0)[0] else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def update(self):
        """Main update loop - publishes smoothed commands."""
        # Smooth velocity ramping
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
        
        # Publish velocity command
        twist = Twist()
        twist.linear.x = self.current_linear_x * self.linear_speed
        twist.linear.y = self.current_linear_y * self.linear_speed
        twist.angular.z = self.current_angular_z * self.angular_speed
        
        self.cmd_vel_pub.publish(twist)
    
    def set_target_velocity(self, linear_x: float, linear_y: float, angular_z: float):
        """Set target velocity from keyboard input."""
        self.target_linear_x = linear_x
        self.target_linear_y = linear_y
        self.target_angular_z = angular_z
    
    def stop_movement(self):
        """Stop all movement."""
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
    
    def toggle_gait(self):
        """Toggle trot gait on/off."""
        self.gait_active = not self.gait_active
        msg = Bool()
        msg.data = self.gait_active
        self.gait_pub.publish(msg)
        status = "ON" if self.gait_active else "OFF"
        print(f"[GAIT] Trot: {status}")
    
    def set_pose(self, pose: str):
        """Set body pose."""
        self.current_pose = pose
        msg = String()
        msg.data = pose
        self.pose_pub.publish(msg)
        print(f"[POSE] Set to: {pose.upper()}")
    
    def increase_speed(self):
        """Increase linear speed."""
        self.linear_speed = min(self.MAX_SPEED, self.linear_speed + self.SPEED_STEP)
        print(f"[SPEED] Linear: {self.linear_speed:.1f}")
    
    def decrease_speed(self):
        """Decrease linear speed."""
        self.linear_speed = max(self.MIN_SPEED, self.linear_speed - self.SPEED_STEP)
        print(f"[SPEED] Linear: {self.linear_speed:.1f}")
    
    def emergency_stop(self):
        """Emergency stop - stop gait and movement."""
        if self.gait_active:
            self.gait_active = False
            msg = Bool()
            msg.data = False
            self.gait_pub.publish(msg)
        self.stop_movement()
        
        # Send zero velocity immediately
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        print("[STOP] Emergency stop executed")
    
    def run(self):
        """Main keyboard input loop."""
        try:
            while rclpy.ok() and self.running:
                key = self.get_key(timeout=0.05)
                
                if not key:
                    continue
                
                # Ctrl+C to quit
                if key == '\x03':
                    print("\n[EXIT] Shutting down...")
                    break
                
                # Movement keys (set target)
                if key in self.MOVE_BINDINGS:
                    x, y, yaw = self.MOVE_BINDINGS[key]
                    self.set_target_velocity(x, y, yaw)
                
                # Pose selection
                elif key in self.POSE_BINDINGS:
                    self.set_pose(self.POSE_BINDINGS[key])
                
                # Gait toggle
                elif key == 'g' or key == 'G':
                    self.toggle_gait()
                
                # Speed adjustment
                elif key in ['+', '=', ']):
                    self.increase_speed()
                elif key in ['-', '_']:
                    self.decrease_speed()
                
                # Emergency stop
                elif key == ' ':
                    self.emergency_stop()
                
                # Help
                elif key == 'h' or key == 'H':
                    self.print_help()
                
                # Release key - stop movement (only for movement keys)
                elif key == '\r' or key == '\n':
                    pass  # Ignore enter
                else:
                    # Any other key stops movement
                    self.set_target_velocity(0.0, 0.0, 0.0)
        
        except Exception as e:
            self.get_logger().error(f'Error in input loop: {e}')
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup and restore terminal settings."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.stop_movement()
        
        # Send final stop commands
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        msg = Bool()
        msg.data = False
        self.gait_pub.publish(msg)
        
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    
    node = TeleopKeyboard()
    
    # Run keyboard input in separate thread
    input_thread = threading.Thread(target=node.run)
    input_thread.daemon = True
    input_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        input_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
