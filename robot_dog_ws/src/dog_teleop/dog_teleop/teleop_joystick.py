#!/usr/bin/env python3
"""
Joystick/gamepad teleoperation node for the robot dog.

Supports Xbox and PlayStation controllers.

Default mappings:
    Left stick (axes 0,1)  : Linear X/Y movement
    Right stick (axes 3,2) : Angular Z (rotation)
    
    Xbox: A/B/X/Y buttons     : Pose selection
    PS4: Cross/Circle/Sq/Tri  : Pose selection
    
    Xbox: LB/RB               : Speed decrease/increase  
    PS4: L1/R1                : Speed decrease/increase
    
    Xbox: Start/Back or RT/LT : Toggle gait
    PS4: Options/Share or R2/L2: Toggle gait

Published topics:
    - /cmd_vel (Twist)          : Velocity commands
    - /gait/start_stop (Bool)   : Start/stop trot gait
    - /body_pose (String)       : Body pose command (stand/sit/lie)
    
Subscribed topics:
    - /joy (Joy)                : Joystick input from joy_node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class TeleopJoystick(Node):
    """
    Joystick teleoperation node for robot dog.
    
    Supports Xbox and PS4 controllers with automatic detection.
    Features:
    - Dual stick control (left=movement, right=rotation)
    - Button mapping for poses and gait
    - Speed adjustment buttons
    - Deadzone filtering
    - Smooth velocity ramping
    """
    
    # Controller types
    CONTROLLER_XBOX = 'xbox'
    CONTROLLER_PS4 = 'ps4'
    CONTROLLER_GENERIC = 'generic'
    
    # Default axis mappings (can be overridden via parameters)
    DEFAULT_AXES = {
        'linear_x': 1,    # Left stick Y
        'linear_y': 0,    # Left stick X
        'angular_z': 3,   # Right stick X (or 2 for some configs)
    }
    
    # Button mappings for different controllers
    BUTTON_MAPS = {
        CONTROLLER_XBOX: {
            'pose_stand': 0,   # A button
            'pose_sit': 1,     # B button  
            'pose_lie': 2,     # X button
            'pose_custom': 3,  # Y button
            'speed_down': 4,   # LB
            'speed_up': 5,     # RB
            'gait_toggle': 7,  # Start button
            'emergency_stop': 6,  # Back button
        },
        CONTROLLER_PS4: {
            'pose_stand': 0,   # Cross (X)
            'pose_sit': 1,     # Circle
            'pose_lie': 2,     # Square
            'pose_custom': 3,  # Triangle
            'speed_down': 4,   # L1
            'speed_up': 5,     # R1
            'gait_toggle': 7,  # Options
            'emergency_stop': 6,  # Share
        },
        CONTROLLER_GENERIC: {
            'pose_stand': 0,
            'pose_sit': 1,
            'pose_lie': 2,
            'pose_custom': 3,
            'speed_down': 4,
            'speed_up': 5,
            'gait_toggle': 7,
            'emergency_stop': 6,
        }
    }
    
    # Speed adjustment step
    SPEED_STEP = 0.1
    MIN_SPEED = 0.1
    MAX_SPEED = 1.0
    
    def __init__(self):
        super().__init__('teleop_joystick')
        
        # Declare parameters
        self.declare_parameter('controller_type', 'auto')  # auto, xbox, ps4, generic
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('smoothing_factor', 0.2)
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_angular_z', 3)
        self.declare_parameter('invert_linear_x', False)
        self.declare_parameter('invert_linear_y', False)
        self.declare_parameter('invert_angular_z', False)
        
        # Load parameters
        self.controller_type = self.get_parameter('controller_type').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.smoothing = self.get_parameter('smoothing_factor').value
        
        # Axis mappings from parameters
        self.axis_linear_x = self.get_parameter('axis_linear_x').value
        self.axis_linear_y = self.get_parameter('axis_linear_y').value
        self.axis_angular_z = self.get_parameter('axis_angular_z').value
        self.invert_linear_x = self.get_parameter('invert_linear_x').value
        self.invert_linear_y = self.get_parameter('invert_linear_y').value
        self.invert_angular_z = self.get_parameter('invert_angular_z').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gait_pub = self.create_publisher(Bool, '/gait/start_stop', 10)
        self.pose_pub = self.create_publisher(String, '/body_pose', 10)
        
        # Subscribe to joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # State
        self.current_pose = 'stand'
        self.gait_active = False
        self.controller_detected = False
        self.button_map = self.BUTTON_MAPS[self.CONTROLLER_GENERIC]
        
        # Current velocities (for smoothing)
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # Previous button states (for detecting presses)
        self.prev_buttons = []
        
        # Timer for continuous publishing
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_cmd_vel)
        
        # Target velocities (set by joystick callback)
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        
        self.get_logger().info('Joystick teleop node initialized')
        self.get_logger().info('Waiting for joystick input on /joy...')
        self.print_help()
    
    def print_help(self):
        """Print usage instructions."""
        msg = """
╔════════════════════════════════════════════════════════════════╗
║          Robot Dog Teleop - Joystick Control                   ║
╠════════════════════════════════════════════════════════════════╣
║ Left Stick            │ Poses (A/✕, B/○, X/□)                 ║
║   ↑/↓ : Forward/Back  │   A/✕ : Stand pose                     ║
║   ←/→ : Strafe        │   B/○ : Sit pose                       ║
║                       │   X/□ : Lie down pose                  ║
║ Right Stick (↔)       │                                        ║
║   ←/→ : Rotate        │ Gait: Start/Back or R2/L2 = Toggle     ║
║                       │                                        ║
║ Speed: LB/R1 (-),     │ Emergency: Back/Share button           ║
║        RB/R2 (+)      │                                        ║
╚════════════════════════════════════════════════════════════════╝
"""
        print(msg)
    
    def detect_controller(self, msg: Joy):
        """Auto-detect controller type based on button count."""
        if self.controller_type == 'auto':
            num_buttons = len(msg.buttons)
            num_axes = len(msg.axes)
            
            # Xbox typically has 11 buttons, 8 axes
            # PS4 typically has 13 buttons, 8 axes
            if num_buttons >= 11:
                if num_buttons >= 13:
                    detected = self.CONTROLLER_PS4
                else:
                    detected = self.CONTROLLER_XBOX
            else:
                detected = self.CONTROLLER_GENERIC
            
            self.controller_type = detected
            self.button_map = self.BUTTON_MAPS[detected]
            self.get_logger().info(f'Detected controller: {detected.upper()} '
                                   f'({num_buttons} buttons, {num_axes} axes)')
        else:
            self.button_map = self.BUTTON_MAPS.get(
                self.controller_type, self.BUTTON_MAPS[self.CONTROLLER_GENERIC]
            )
    
    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to joystick axis value."""
        if abs(value) < self.deadzone:
            return 0.0
        # Rescale to full range after deadzone
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def get_axis(self, msg: Joy, axis_index: int, invert: bool = False) -> float:
        """Safely get axis value with bounds checking."""
        if axis_index < 0 or axis_index >= len(msg.axes):
            return 0.0
        value = msg.axes[axis_index]
        if invert:
            value = -value
        return self.apply_deadzone(value)
    
    def joy_callback(self, msg: Joy):
        """Process joystick input."""
        # Auto-detect controller on first message
        if not self.controller_detected:
            self.detect_controller(msg)
            self.controller_detected = True
            self.prev_buttons = list(msg.buttons)
            return
        
        # Get axis values
        linear_x = self.get_axis(msg, self.axis_linear_x, self.invert_linear_x)
        linear_y = self.get_axis(msg, self.axis_linear_y, self.invert_linear_y)
        angular_z = self.get_axis(msg, self.axis_angular_z, self.invert_angular_z)
        
        # Set target velocities
        self.target_linear_x = linear_x
        self.target_linear_y = linear_y
        self.target_angular_z = angular_z
        
        # Process button presses (detect rising edge)
        for i, (prev, curr) in enumerate(zip(self.prev_buttons, msg.buttons)):
            if not prev and curr:  # Button pressed
                self.handle_button_press(i)
        
        self.prev_buttons = list(msg.buttons)
    
    def handle_button_press(self, button_index: int):
        """Handle button press events."""
        # Pose selection
        if button_index == self.button_map.get('pose_stand'):
            self.set_pose('stand')
        elif button_index == self.button_map.get('pose_sit'):
            self.set_pose('sit')
        elif button_index == self.button_map.get('pose_lie'):
            self.set_pose('lie')
        
        # Speed control
        elif button_index == self.button_map.get('speed_up'):
            self.increase_speed()
        elif button_index == self.button_map.get('speed_down'):
            self.decrease_speed()
        
        # Gait toggle
        elif button_index == self.button_map.get('gait_toggle'):
            self.toggle_gait()
        
        # Emergency stop
        elif button_index == self.button_map.get('emergency_stop'):
            self.emergency_stop()
    
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
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = self.current_linear_x * self.linear_speed
        twist.linear.y = self.current_linear_y * self.linear_speed
        twist.angular.z = self.current_angular_z * self.angular_speed
        
        self.cmd_vel_pub.publish(twist)
    
    def set_pose(self, pose: str):
        """Set body pose."""
        self.current_pose = pose
        msg = String()
        msg.data = pose
        self.pose_pub.publish(msg)
        self.get_logger().info(f'Pose set to: {pose.upper()}')
    
    def toggle_gait(self):
        """Toggle trot gait."""
        self.gait_active = not self.gait_active
        msg = Bool()
        msg.data = self.gait_active
        self.gait_pub.publish(msg)
        status = "ON" if self.gait_active else "OFF"
        self.get_logger().info(f'Gait toggled: {status}')
    
    def increase_speed(self):
        """Increase speed."""
        self.linear_speed = min(self.MAX_SPEED, self.linear_speed + self.SPEED_STEP)
        self.angular_speed = min(self.MAX_SPEED, self.angular_speed + self.SPEED_STEP)
        self.get_logger().info(f'Speed increased: linear={self.linear_speed:.1f}, '
                               f'angular={self.angular_speed:.1f}')
    
    def decrease_speed(self):
        """Decrease speed."""
        self.linear_speed = max(self.MIN_SPEED, self.linear_speed - self.SPEED_STEP)
        self.angular_speed = max(self.MIN_SPEED, self.angular_speed - self.SPEED_STEP)
        self.get_logger().info(f'Speed decreased: linear={self.linear_speed:.1f}, '
                               f'angular={self.angular_speed:.1f}')
    
    def emergency_stop(self):
        """Emergency stop."""
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        if self.gait_active:
            self.gait_active = False
            msg = Bool()
            msg.data = False
            self.gait_pub.publish(msg)
        
        # Send zero velocity
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().warn('EMERGENCY STOP!')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoystick()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop commands before shutdown
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        
        msg = Bool()
        msg.data = False
        node.gait_pub.publish(msg)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
