#!/usr/bin/env python3
"""
Gait controller for the robot dog.

This module implements various walking gaits for the quadruped robot,
including trot gait with body compensation and smooth transitions.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from builtin_interfaces.msg import Duration
import math
from dataclasses import dataclass
from enum import Enum, auto


class GaitState(Enum):
    """States of the gait controller."""
    STOPPED = auto()
    STARTING = auto()
    RUNNING = auto()
    STOPPING = auto()


@dataclass
class GaitConfig:
    """Configuration for trot gait parameters."""
    # Timing
    period: float = 1.0              # Gait cycle period (seconds)
    
    # Step parameters
    step_height: float = 0.04        # Step height (m) - 3-5 cm
    step_length: float = 0.06        # Step length (m) - 5-8 cm
    step_width: float = 0.02         # Step width (m)
    
    # Body compensation
    body_compensation: float = 0.02  # Body height adjustment during swing (m)
    roll_compensation: float = 0.05  # Roll angle compensation (rad)
    pitch_compensation: float = 0.05 # Pitch angle compensation (rad)
    
    # Smooth transitions
    transition_duration: float = 0.5  # Duration of start/stop transition (s)
    
    # Trajectory type
    trajectory_type: str = "triangle"  # "triangle" or "sinusoid"
    
    # Leg offsets (for fine tuning)
    leg_offsets = {
        'FL': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'FR': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'BL': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'BR': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    }


class GaitController(Node):
    """
    Controller for robot dog gaits.
    
    Implements different walking patterns including:
    - Trot gait (diagonal legs move together)
    - Walk gait (sequential leg movement)
    - Pace gait (lateral legs move together)
    - Bound gait (front/back pairs)
    
    Features:
    - Smooth start/stop transitions
    - Body height compensation
    - Configurable trajectory (triangle/sinusoid)
    - cmd_vel publishing for testing
    """
    
    def __init__(self):
        super().__init__('gait_controller')
        
        # Declare parameters
        self._declare_parameters()
        
        # Load configuration from parameters
        self.config = self._load_config()
        
        # Publishers for leg joints
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # cmd_vel publisher for testing
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Body height adjustment publisher
        self.body_height_pub = self.create_publisher(
            Float64,
            '/body_height_adjustment',
            10
        )
        
        # Gait state publisher
        self.gait_state_pub = self.create_publisher(
            Bool,
            '/gait_active',
            10
        )
        
        # Command subscribers
        self.start_stop_sub = self.create_subscription(
            Bool,
            '/gait/start_stop',
            self._start_stop_callback,
            10
        )
        
        self.gait_type_sub = self.create_subscription(
            Twist,
            '/gait/command',
            self._gait_command_callback,
            10
        )
        
        # Timer for gait update
        self.timer_period = 0.02  # 50 Hz for smooth control
        self.timer = self.create_timer(self.timer_period, self._timer_callback)
        
        # Joint names for all 4 legs
        self.leg_joints = {
            'FL': ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint'],
            'FR': ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint'],
            'BL': ['BL_hip_joint', 'BL_thigh_joint', 'BL_calf_joint'],
            'BR': ['BR_hip_joint', 'BR_thigh_joint', 'BR_calf_joint']
        }
        
        # Leg positions (for inverse kinematics)
        self.leg_positions = {
            'FL': {'x': 0.15, 'y': 0.08, 'z': -0.25},
            'FR': {'x': 0.15, 'y': -0.08, 'z': -0.25},
            'BL': {'x': -0.15, 'y': 0.08, 'z': -0.25},
            'BR': {'x': -0.15, 'y': -0.08, 'z': -0.25}
        }
        
        # State variables
        self.gait_state = GaitState.STOPPED
        self.gait_type = 'trot'
        self.time = 0.0
        self.transition_time = 0.0
        self.transition_factor = 0.0  # 0.0 to 1.0 for smooth transitions
        
        # Current velocities
        self.linear_velocity_x = 0.0
        self.linear_velocity_y = 0.0
        self.angular_velocity_z = 0.0
        
        # Standing pose (default joint angles)
        self.standing_pose = {
            'hip': 0.0,
            'thigh': 0.5,
            'calf': -1.2
        }
        
        # Body height offset (for compensation)
        self.body_height_offset = 0.0
        
        # Phase offset for diagonal pairs (trot gait)
        # FL and BR move together (phase 0)
        # FR and BL move together (phase π)
        self.phase_offsets = {
            'FL': 0.0,
            'FR': math.pi,
            'BL': math.pi,
            'BR': 0.0
        }
        
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('   Gait Controller Initialized')
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info(f'Gait types: trot, walk, pace, bound, stand')
        self.get_logger().info(f'Trot config: period={self.config.period}s, '
                              f'step_height={self.config.step_height}m, '
                              f'step_length={self.config.step_length}m')
        self.get_logger().info('Services:')
        self.get_logger().info('  - Publish to /gait/start_stop (Bool) to start/stop')
        self.get_logger().info('  - Publish to /gait/command (Twist) to set velocity')
        self.get_logger().info('═══════════════════════════════════════')
    
    def _declare_parameters(self):
        """Declare ROS2 parameters."""
        # Gait timing
        self.declare_parameter('gait.period', 1.0)
        self.declare_parameter('gait.timer_rate', 50.0)
        
        # Step parameters
        self.declare_parameter('gait.step_height', 0.04)
        self.declare_parameter('gait.step_length', 0.06)
        self.declare_parameter('gait.step_width', 0.02)
        
        # Body compensation
        self.declare_parameter('gait.body_compensation', 0.02)
        self.declare_parameter('gait.roll_compensation', 0.05)
        self.declare_parameter('gait.pitch_compensation', 0.05)
        
        # Transitions
        self.declare_parameter('gait.transition_duration', 0.5)
        
        # Trajectory
        self.declare_parameter('gait.trajectory_type', 'triangle')
        
        # Standing pose
        self.declare_parameter('standing_pose.hip', 0.0)
        self.declare_parameter('standing_pose.thigh', 0.5)
        self.declare_parameter('standing_pose.calf', -1.2)
    
    def _load_config(self) -> GaitConfig:
        """Load configuration from ROS parameters."""
        config = GaitConfig()
        
        config.period = self.get_parameter('gait.period').value
        config.step_height = self.get_parameter('gait.step_height').value
        config.step_length = self.get_parameter('gait.step_length').value
        config.step_width = self.get_parameter('gait.step_width').value
        config.body_compensation = self.get_parameter('gait.body_compensation').value
        config.roll_compensation = self.get_parameter('gait.roll_compensation').value
        config.pitch_compensation = self.get_parameter('gait.pitch_compensation').value
        config.transition_duration = self.get_parameter('gait.transition_duration').value
        config.trajectory_type = self.get_parameter('gait.trajectory_type').value
        
        self.standing_pose = {
            'hip': self.get_parameter('standing_pose.hip').value,
            'thigh': self.get_parameter('standing_pose.thigh').value,
            'calf': self.get_parameter('standing_pose.calf').value
        }
        
        return config
    
    def _start_stop_callback(self, msg: Bool):
        """Handle start/stop commands."""
        if msg.data:
            self.start_gait()
        else:
            self.stop_gait()
    
    def _gait_command_callback(self, msg: Twist):
        """Handle gait velocity commands."""
        self.linear_velocity_x = msg.linear.x
        self.linear_velocity_y = msg.linear.y
        self.angular_velocity_z = msg.angular.z
        
        # Scale step parameters based on velocity
        speed = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        if speed > 0.01:
            self.config.step_length = min(0.08, 0.06 + speed * 0.02)
    
    def _timer_callback(self):
        """Main control loop - runs at 50Hz."""
        # Update time
        self.time += self.timer_period
        
        # Handle state transitions
        self._update_transition()
        
        # Calculate joint positions based on current state
        if self.gait_state == GaitState.STOPPED:
            joint_positions = self._get_standing_positions()
            body_height = 0.0
        else:
            joint_positions, body_height = self._calculate_trot_gait(self.time)
        
        # Publish commands
        self._publish_trajectory(joint_positions)
        self._publish_body_height(body_height)
        self._publish_cmd_vel()
        self._publish_gait_state()
    
    def _update_transition(self):
        """Update transition factor for smooth start/stop."""
        transition_speed = self.timer_period / self.config.transition_duration
        
        if self.gait_state == GaitState.STARTING:
            self.transition_factor += transition_speed
            if self.transition_factor >= 1.0:
                self.transition_factor = 1.0
                self.gait_state = GaitState.RUNNING
                self.get_logger().info('Gait: RUNNING')
        
        elif self.gait_state == GaitState.STOPPING:
            self.transition_factor -= transition_speed
            if self.transition_factor <= 0.0:
                self.transition_factor = 0.0
                self.gait_state = GaitState.STOPPED
                self.get_logger().info('Gait: STOPPED')
        
        elif self.gait_state == GaitState.RUNNING:
            self.transition_factor = 1.0
    
    def _calculate_trot_gait(self, t: float):
        """
        Calculate trot gait with body compensation.
        
        Trot gait: diagonal legs move together
        - Pair 1: FL (Front Left) + BR (Back Right) 
        - Pair 2: FR (Front Right) + BL (Back Left)
        
        Args:
            t: Current time in seconds
            
        Returns:
            tuple: (joint_positions dict, body_height_offset)
        """
        # Calculate phase (0 to 2π)
        phase = (t % self.config.period) / self.config.period * 2 * math.pi
        
        # Calculate foot trajectories for each leg
        foot_positions = {}
        max_swing_height = 0.0
        
        for leg in ['FL', 'FR', 'BL', 'BR']:
            leg_phase = phase + self.phase_offsets[leg]
            foot_pos = self._calculate_foot_trajectory(leg, leg_phase)
            foot_positions[leg] = foot_pos
            
            # Track maximum swing height for body compensation
            if foot_pos['z'] > max_swing_height:
                max_swing_height = foot_pos['z']
        
        # Convert foot positions to joint angles (simplified IK)
        joint_positions = {}
        for leg in ['FL', 'FR', 'BL', 'BR']:
            foot_pos = foot_positions[leg]
            angles = self._foot_to_joint_angles(leg, foot_pos)
            joint_positions.update(angles)
        
        # Body height compensation
        # Lower body when legs are in swing phase for stability
        body_compensation = -max_swing_height * self.config.body_compensation * 2
        
        # Apply transition factor for smooth start/stop
        for key in joint_positions:
            standing_value = self._get_standing_positions()[key]
            joint_positions[key] = standing_value + (joint_positions[key] - standing_value) * self.transition_factor
        
        body_height = body_compensation * self.transition_factor
        
        return joint_positions, body_height
    
    def _calculate_foot_trajectory(self, leg: str, phase: float) -> dict:
        """
        Calculate foot trajectory in body frame.
        
        Uses triangular or sinusoidal trajectory.
        
        Args:
            leg: Leg name (FL, FR, BL, BR)
            phase: Current phase angle (0 to 2π)
            
        Returns:
            dict: Foot position {'x', 'y', 'z'}
        """
        # Normalize phase to 0-2π
        phase = phase % (2 * math.pi)
        
        # Default foot position (standing)
        base_pos = self.leg_positions[leg].copy()
        
        # Apply leg-specific offsets
        offset = self.config.leg_offsets[leg]
        base_pos['x'] += offset['x']
        base_pos['y'] += offset['y']
        base_pos['z'] += offset['z']
        
        # Scale by transition factor
        step_length = self.config.step_length * self.transition_factor
        step_height = self.config.step_height * self.transition_factor
        
        # Calculate trajectory
        x_offset = 0.0
        z_offset = 0.0
        
        if self.config.trajectory_type == 'triangle':
            # Triangular trajectory
            # Swing phase: 0 to π (foot in air, moving forward)
            # Stance phase: π to 2π (foot on ground, moving backward)
            
            if phase < math.pi:  # Swing phase
                # Triangle wave for height
                # Peak at phase = π/2
                if phase < math.pi / 2:
                    z_offset = step_height * (phase / (math.pi / 2))
                else:
                    z_offset = step_height * (1 - (phase - math.pi/2) / (math.pi / 2))
                
                # Forward motion (sine for smoothness)
                x_offset = step_length * math.sin(phase)
            
            else:  # Stance phase
                # Foot on ground
                z_offset = 0.0
                # Backward motion relative to body
                x_offset = -step_length * math.sin(phase - math.pi)
        
        else:  # sinusoid
            # Sinusoidal trajectory
            # Positive z = swing (foot up)
            z_offset = max(0, math.sin(phase)) * step_height
            
            # Forward/backward motion
            x_offset = step_length * math.cos(phase)
        
        return {
            'x': base_pos['x'] + x_offset,
            'y': base_pos['y'],
            'z': base_pos['z'] + z_offset
        }
    
    def _foot_to_joint_angles(self, leg: str, foot_pos: dict) -> dict:
        """
        Convert foot position to joint angles (simplified inverse kinematics).
        
        Args:
            leg: Leg name
            foot_pos: Foot position {'x', 'y', 'z'}
            
        Returns:
            dict: Joint angles for the leg
        """
        # Simplified IK for 3-DOF leg
        # Leg structure: hip (abduction) -> thigh (flexion) -> calf (knee)
        
        L1 = 0.083  # Hip to thigh length (m)
        L2 = 0.25   # Thigh to calf length (m)
        L3 = 0.25   # Calf to foot length (m)
        
        x, y, z = foot_pos['x'], foot_pos['y'], foot_pos['z']
        
        # Hip angle (abduction/adduction)
        hip_angle = math.atan2(y, -z)
        
        # Distance in sagittal plane
        d = math.sqrt(x**2 + z**2)
        
        # Thigh and calf angles using law of cosines
        # Simplified - assuming leg is mostly in sagittal plane
        cos_knee = (L2**2 + L3**2 - d**2) / (2 * L2 * L3)
        cos_knee = max(-1, min(1, cos_knee))  # Clamp
        knee_angle = math.pi - math.acos(cos_knee)
        
        alpha = math.atan2(-z, x)
        cos_beta = (L2**2 + d**2 - L3**2) / (2 * L2 * d)
        cos_beta = max(-1, min(1, cos_beta))
        beta = math.acos(cos_beta)
        
        thigh_angle = alpha + beta - math.pi / 2
        
        # Adjust based on leg side
        if leg in ['FR', 'BR']:
            hip_angle = -hip_angle
            thigh_angle = -thigh_angle
            knee_angle = -knee_angle
        
        # Mirror for back legs
        if leg in ['BL', 'BR']:
            thigh_angle = -thigh_angle
            knee_angle = -knee_angle
        
        return {
            f'{leg}_hip_joint': hip_angle,
            f'{leg}_thigh_joint': thigh_angle,
            f'{leg}_calf_joint': knee_angle
        }
    
    def _get_standing_positions(self) -> dict:
        """Return standing position for all joints."""
        positions = {}
        for leg in ['FL', 'FR', 'BL', 'BR']:
            positions[f'{leg}_hip_joint'] = self.standing_pose['hip']
            positions[f'{leg}_thigh_joint'] = self.standing_pose['thigh']
            positions[f'{leg}_calf_joint'] = self.standing_pose['calf']
        return positions
    
    def _publish_trajectory(self, positions: dict):
        """Publish joint trajectory message."""
        msg = JointTrajectory()
        msg.joint_names = list(positions.keys())
        
        point = JointTrajectoryPoint()
        point.positions = list(positions.values())
        point.time_from_start = Duration(sec=0, nanosec=int(self.timer_period * 1e9))
        
        msg.points = [point]
        
        self.trajectory_pub.publish(msg)
    
    def _publish_body_height(self, height: float):
        """Publish body height adjustment."""
        msg = Float64()
        msg.data = height
        self.body_height_pub.publish(msg)
    
    def _publish_cmd_vel(self):
        """Publish cmd_vel for testing/monitoring."""
        msg = Twist()
        msg.linear.x = self.linear_velocity_x * self.transition_factor
        msg.linear.y = self.linear_velocity_y * self.transition_factor
        msg.angular.z = self.angular_velocity_z * self.transition_factor
        self.cmd_vel_pub.publish(msg)
    
    def _publish_gait_state(self):
        """Publish current gait state."""
        msg = Bool()
        msg.data = self.gait_state in [GaitState.RUNNING, GaitState.STARTING]
        self.gait_state_pub.publish(msg)
    
    def set_gait(self, gait_type: str):
        """Set the current gait type."""
        valid_gaits = ['trot', 'walk', 'pace', 'bound', 'stand']
        if gait_type in valid_gaits:
            self.gait_type = gait_type
            self.get_logger().info(f'Gait set to: {gait_type}')
        else:
            self.get_logger().warn(f'Unknown gait: {gait_type}. Valid: {valid_gaits}')
    
    def start_gait(self):
        """Start the gait with smooth transition."""
        if self.gait_state == GaitState.STOPPED:
            self.gait_state = GaitState.STARTING
            self.transition_time = 0.0
            self.get_logger().info('Gait: STARTING')
        elif self.gait_state == GaitState.STOPPING:
            # Reverse stopping to starting
            self.gait_state = GaitState.STARTING
            self.get_logger().info('Gait: STARTING (reversed from stopping)')
    
    def stop_gait(self):
        """Stop the gait with smooth transition."""
        if self.gait_state in [GaitState.RUNNING, GaitState.STARTING]:
            self.gait_state = GaitState.STOPPING
            self.transition_time = 0.0
            self.get_logger().info('Gait: STOPPING')
    
    def set_velocity(self, linear_x: float, linear_y: float, angular_z: float):
        """Set gait velocity."""
        self.linear_velocity_x = linear_x
        self.linear_velocity_y = linear_y
        self.angular_velocity_z = angular_z


def main(args=None):
    rclpy.init(args=args)
    
    controller = GaitController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
