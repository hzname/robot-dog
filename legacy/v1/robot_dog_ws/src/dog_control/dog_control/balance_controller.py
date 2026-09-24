#!/usr/bin/env python3
"""
Balance controller for quadruped robot dog.
Maintains body orientation using IMU feedback with PID control.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import math
import numpy as np


class PIDController:
    """Simple PID controller with anti-windup."""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_limit: float = None, integral_limit: float = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
    
    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
    
    def update(self, error: float, current_time) -> float:
        """Update PID controller and return output."""
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0
        
        dt = (current_time - self.prev_time)
        if dt <= 0:
            return 0.0
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, 
                              min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        
        # Update state
        self.prev_error = error
        self.prev_time = current_time
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limit
        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))
        
        return output


def euler_from_quaternion(q):
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


class BalanceController(Node):
    """
    Controller for robot balance using PID regulation.
    
    Compensates for body tilt by adjusting leg positions in real-time.
    """
    
    # Leg indices
    FL = 0  # Front Left
    FR = 1  # Front Right
    RL = 2  # Rear Left
    RR = 3  # Rear Right
    
    def __init__(self):
        super().__init__('balance_controller')
        
        # PID Parameters
        self.declare_parameter('pid.roll.kp', 0.5)
        self.declare_parameter('pid.roll.ki', 0.01)
        self.declare_parameter('pid.roll.kd', 0.1)
        
        self.declare_parameter('pid.pitch.kp', 0.5)
        self.declare_parameter('pid.pitch.ki', 0.01)
        self.declare_parameter('pid.pitch.kd', 0.1)
        
        self.declare_parameter('pid.output_limit', 0.05)  # meters
        self.declare_parameter('pid.integral_limit', 0.1)
        
        # Balance parameters
        self.declare_parameter('balance.enabled', True)
        self.declare_parameter('balance.leg_span_x', 0.3)  # meters (front-back)
        self.declare_parameter('balance.leg_span_y', 0.2)  # meters (left-right)
        self.declare_parameter('balance.max_correction', 0.03)  # meters
        
        # Read parameters
        roll_kp = self.get_parameter('pid.roll.kp').value
        roll_ki = self.get_parameter('pid.roll.ki').value
        roll_kd = self.get_parameter('pid.roll.kd').value
        
        pitch_kp = self.get_parameter('pid.pitch.kp').value
        pitch_ki = self.get_parameter('pid.pitch.ki').value
        pitch_kd = self.get_parameter('pid.pitch.kd').value
        
        output_limit = self.get_parameter('pid.output_limit').value
        integral_limit = self.get_parameter('pid.integral_limit').value
        
        self.balance_enabled = self.get_parameter('balance.enabled').value
        self.leg_span_x = self.get_parameter('balance.leg_span_x').value
        self.leg_span_y = self.get_parameter('balance.leg_span_y').value
        self.max_correction = self.get_parameter('balance.max_correction').value
        
        # Initialize PID controllers
        # Roll controls left/right tilt (affects left vs right legs)
        self.roll_pid = PIDController(
            roll_kp, roll_ki, roll_kd,
            output_limit=output_limit,
            integral_limit=integral_limit
        )
        
        # Pitch controls forward/backward tilt (affects front vs back legs)
        self.pitch_pid = PIDController(
            pitch_kp, pitch_ki, pitch_kd,
            output_limit=output_limit,
            integral_limit=integral_limit
        )
        
        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.tilt_sub = self.create_subscription(
            Float32MultiArray,
            '/imu/tilt',
            self.tilt_callback,
            10
        )
        
        # Publishers for leg position corrections
        self.leg_correction_pub = self.create_publisher(
            Float32MultiArray,
            '/balance/leg_corrections',
            10
        )
        
        # Publishers for individual legs (x, y, z offsets)
        self.fl_pub = self.create_publisher(Point, '/balance/fl_correction', 10)
        self.fr_pub = self.create_publisher(Point, '/balance/fr_correction', 10)
        self.rl_pub = self.create_publisher(Point, '/balance/rl_correction', 10)
        self.rr_pub = self.create_publisher(Point, '/balance/rr_correction', 10)
        
        # Current orientation
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Current angular velocities
        self.angular_vel = np.zeros(3)
        
        # Target orientation (0 = level)
        self.target_roll = 0.0
        self.target_pitch = 0.0
        
        self.get_logger().info('Balance controller initialized with PID')
        self.get_logger().info(f'Roll PID: Kp={roll_kp}, Ki={roll_ki}, Kd={roll_kd}')
        self.get_logger().info(f'Pitch PID: Kp={pitch_kp}, Ki={pitch_ki}, Kd={pitch_kd}')
    
    def imu_callback(self, msg: Imu):
        """Process IMU data and compute balance corrections."""
        # Extract orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(msg.orientation)
        
        # Extract angular velocity
        self.angular_vel[0] = msg.angular_velocity.x
        self.angular_vel[1] = msg.angular_velocity.y
        self.angular_vel[2] = msg.angular_velocity.z
        
        # Compute balance corrections
        if self.balance_enabled:
            self.compute_balance_corrections()
    
    def tilt_callback(self, msg: Float32MultiArray):
        """Receive direct tilt angles in degrees."""
        if len(msg.data) >= 2:
            # Convert to radians for consistency
            self.roll = math.radians(msg.data[0])
            self.pitch = math.radians(msg.data[1])
    
    def compute_balance_corrections(self):
        """Compute leg position corrections using PID control."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Calculate errors (target is always 0 for level body)
        roll_error = self.target_roll - self.roll
        pitch_error = self.target_pitch - self.pitch
        
        # Update PID controllers
        roll_correction = self.roll_pid.update(roll_error, current_time)
        pitch_correction = self.pitch_pid.update(pitch_error, current_time)
        
        # Apply limits
        roll_correction = max(-self.max_correction, min(self.max_correction, roll_correction))
        pitch_correction = max(-self.max_correction, min(self.max_correction, pitch_correction))
        
        # Distribute corrections to legs
        # Roll: left vs right (left legs get +, right legs get -)
        # Pitch: front vs back (front legs get +, back legs get -)
        
        corrections = self._distribute_corrections(roll_correction, pitch_correction)
        
        # Publish corrections
        self._publish_corrections(corrections)
    
    def _distribute_corrections(self, roll_corr: float, pitch_corr: float) -> list:
        """
        Distribute balance corrections to each leg.
        
        Returns list of (x, y, z) corrections for [FL, FR, RL, RR]
        """
        corrections = []
        
        # Leg positions relative to body center
        # FL: (+x, +y), FR: (+x, -y), RL: (-x, +y), RR: (-x, -y)
        
        # Front Left: compensate for both roll and pitch
        fl_z = -pitch_corr + roll_corr
        corrections.append((0.0, 0.0, fl_z))
        
        # Front Right: pitch positive, roll negative
        fr_z = -pitch_corr - roll_corr
        corrections.append((0.0, 0.0, fr_z))
        
        # Rear Left: pitch negative, roll positive
        rl_z = pitch_corr + roll_corr
        corrections.append((0.0, 0.0, rl_z))
        
        # Rear Right: pitch negative, roll negative
        rr_z = pitch_corr - roll_corr
        corrections.append((0.0, 0.0, rr_z))
        
        return corrections
    
    def _publish_corrections(self, corrections: list):
        """Publish leg corrections to ROS topics."""
        # Publish as array [FL_z, FR_z, RL_z, RR_z]
        leg_msg = Float32MultiArray()
        leg_msg.data = [c[2] for c in corrections]
        self.leg_correction_pub.publish(leg_msg)
        
        # Publish individual leg corrections
        pubs = [self.fl_pub, self.fr_pub, self.rl_pub, self.rr_pub]
        names = ['FL', 'FR', 'RL', 'RR']
        
        for i, ((x, y, z), pub, name) in enumerate(zip(corrections, pubs, names)):
            point_msg = Point()
            point_msg.x = x
            point_msg.y = y
            point_msg.z = z
            pub.publish(point_msg)
            
            # Debug logging (throttled)
            if i == 0 and self.get_clock().now().nanoseconds % 2_000_000_000 < 100_000_000:
                self.get_logger().debug(
                    f'Corrections: {name}=({x:.4f}, {y:.4f}, {z:.4f}) | '
                    f'Roll={math.degrees(self.roll):.2f}°, '
                    f'Pitch={math.degrees(self.pitch):.2f}°'
                )
    
    def set_target_orientation(self, roll: float, pitch: float):
        """Set target orientation in radians."""
        self.target_roll = roll
        self.target_pitch = pitch
        self.roll_pid.reset()
        self.pitch_pid.reset()
    
    def enable_balance(self, enabled: bool):
        """Enable or disable balance control."""
        self.balance_enabled = enabled
        if not enabled:
            # Reset PID when disabling
            self.roll_pid.reset()
            self.pitch_pid.reset()
            # Publish zero corrections
            self._publish_corrections([(0, 0, 0)] * 4)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Disable balance and cleanup
        node.enable_balance(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
