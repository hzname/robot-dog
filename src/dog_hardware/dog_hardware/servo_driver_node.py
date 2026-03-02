"""
Servo Driver Node for PCA9685 PWM controller.
Controls 12 servos for quadruped robot on Banana Pi.
"""

import math
import yaml
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Try to import smbus2, fallback to mock for testing
try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False
    SMBus = None


class PCA9685:
    """PCA9685 PWM controller driver via I2C."""
    
    # PCA9685 registers
    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    
    # Bits
    RESTART = 0x80
    SLEEP = 0x10
    ALLCALL = 0x01
    OUTDRV = 0x04

    def __init__(self, bus_number: int = 0, address: int = 0x40, frequency: int = 50):
        """
        Initialize PCA9685.
        
        Args:
            bus_number: I2C bus number (0 for Banana Pi I2C0)
            address: I2C device address (default 0x40)
            frequency: PWM frequency in Hz (50Hz for servos)
        """
        self.address = address
        self.frequency = frequency
        self._mock_mode = not HAS_SMBUS
        
        if self._mock_mode:
            # Mock mode for testing without hardware
            self.bus = None
        else:
            self.bus = SMBus(bus_number)
            self._init_chip()
    
    def _init_chip(self):
        """Initialize PCA9685 chip."""
        if self._mock_mode:
            return
            
        # Reset
        self._write_byte(self.MODE1, self.RESTART)
        import time
        time.sleep(0.005)
        
        # Set sleep mode to change prescale
        self._write_byte(self.MODE1, self.SLEEP)
        
        # Calculate prescale for desired frequency
        # prescale = round(25MHz / (4096 * frequency)) - 1
        prescale = int(round(25000000.0 / (4096.0 * self.frequency) - 1.0))
        self._write_byte(self.PRESCALE, prescale)
        
        # Wake up
        self._write_byte(self.MODE1, 0x00)
        import time
        time.sleep(0.005)
        
        # Set MODE2 (totem pole drive, non-inverted)
        self._write_byte(self.MODE2, self.OUTDRV)
        
        # Set MODE1 (auto-increment, normal mode)
        self._write_byte(self.MODE1, self.ALLCALL)
        import time
        time.sleep(0.005)
    
    def _write_byte(self, register: int, value: int):
        """Write byte to register."""
        if self._mock_mode:
            return
        self.bus.write_byte_data(self.address, register, value)
    
    def set_pwm(self, channel: int, on: int, off: int):
        """
        Set PWM for a channel.
        
        Args:
            channel: Channel number (0-15)
            on: ON tick (0-4095)
            off: OFF tick (0-4095)
        """
        if self._mock_mode:
            return
            
        base = self.LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.address, base, on & 0xFF)
        self.bus.write_byte_data(self.address, base + 1, on >> 8)
        self.bus.write_byte_data(self.address, base + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, base + 3, off >> 8)
    
    def set_servo_pulse(self, channel: int, pulse_us: float):
        """
        Set servo pulse width in microseconds.
        
        Args:
            channel: Channel number (0-15)
            pulse_us: Pulse width in microseconds (typically 500-2500)
        """
        # For 50Hz, period is 20000us
        # 4096 ticks per period
        # tick_time = 20000 / 4096 = 4.88us per tick
        tick = int(pulse_us / 20000.0 * 4096.0)
        tick = max(0, min(4095, tick))  # Clamp to valid range
        self.set_pwm(channel, 0, tick)
    
    def set_all_off(self):
        """Turn off all PWM outputs."""
        for channel in range(16):
            self.set_pwm(channel, 0, 0)


class ServoConfig:
    """Configuration for a single servo."""
    
    def __init__(self, 
                 name: str,
                 channel: int,
                 min_angle: float = -90.0,
                 max_angle: float = 90.0,
                 min_pulse: float = 500.0,
                 max_pulse: float = 2500.0,
                 inverted: bool = False,
                 offset: float = 0.0):
        """
        Initialize servo configuration.
        
        Args:
            name: Joint name (e.g., 'FL_hip')
            channel: PCA9685 channel (0-15)
            min_angle: Minimum angle in degrees
            max_angle: Maximum angle in degrees
            min_pulse: Minimum pulse width in microseconds
            max_pulse: Maximum pulse width in microseconds
            inverted: Whether to invert angle
            offset: Angle offset in degrees
        """
        self.name = name
        self.channel = channel
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.inverted = inverted
        self.offset = offset
        self.current_angle = 0.0
    
    def angle_to_pulse(self, angle: float) -> float:
        """
        Convert angle to pulse width.
        
        Args:
            angle: Angle in degrees
            
        Returns:
            Pulse width in microseconds
        """
        # Apply inversion
        if self.inverted:
            angle = -angle
        
        # Apply offset
        angle = angle + self.offset
        
        # Clamp to limits
        angle = max(self.min_angle, min(self.max_angle, angle))
        
        # Linear interpolation
        pulse = self.min_pulse + (angle - self.min_angle) / \
                (self.max_angle - self.min_angle) * (self.max_pulse - self.min_pulse)
        
        return pulse
    
    def pulse_to_angle(self, pulse: float) -> float:
        """
        Convert pulse width to angle.
        
        Args:
            pulse: Pulse width in microseconds
            
        Returns:
            Angle in degrees
        """
        # Linear interpolation
        angle = self.min_angle + (pulse - self.min_pulse) / \
                (self.max_pulse - self.min_pulse) * (self.max_angle - self.min_angle)
        
        # Remove offset
        angle = angle - self.offset
        
        # Apply inversion
        if self.inverted:
            angle = -angle
        
        return angle


class ServoDriverNode(Node):
    """
    ROS2 node for controlling servos via PCA9685.
    
    Subscribes to /joint_commands and publishes /joint_states.
    """
    
    def __init__(self):
        super().__init__('servo_driver_node')
        
        # Parameters
        self.declare_parameter('i2c_bus', 0)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('pwm_frequency', 50)
        self.declare_parameter('config_file', '')
        self.declare_parameter('use_mock', not HAS_SMBUS)
        
        i2c_bus = self.get_parameter('i2c_bus').value
        i2c_address = self.get_parameter('i2c_address').value
        pwm_frequency = self.get_parameter('pwm_frequency').value
        config_file = self.get_parameter('config_file').value
        use_mock = self.get_parameter('use_mock').value
        
        self.get_logger().info(f'Initializing PCA9685 on bus {i2c_bus}, address 0x{i2c_address:02X}')
        
        # Initialize PCA9685
        try:
            if use_mock:
                self.get_logger().warn('Running in MOCK mode (no hardware)')
                self.pca = None
            else:
                self.pca = PCA9685(i2c_bus, i2c_address, pwm_frequency)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PCA9685: {e}')
            self.get_logger().warn('Falling back to MOCK mode')
            self.pca = None
        
        # Load servo configurations
        self.servos: Dict[str, ServoConfig] = {}
        self._load_config(config_file)
        
        # ROS interfaces
        self.command_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self._command_callback,
            10
        )
        
        self.state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Timer for publishing joint states
        self.state_timer = self.create_timer(0.02, self._publish_states)  # 50Hz
        
        self.get_logger().info(f'Servo driver initialized with {len(self.servos)} servos')
    
    def _load_config(self, config_file: str):
        """Load servo configuration from file or use defaults."""
        
        if config_file:
            try:
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                
                for servo_cfg in config.get('servos', []):
                    servo = ServoConfig(
                        name=servo_cfg['name'],
                        channel=servo_cfg['channel'],
                        min_angle=servo_cfg.get('min_angle', -90.0),
                        max_angle=servo_cfg.get('max_angle', 90.0),
                        min_pulse=servo_cfg.get('min_pulse', 500.0),
                        max_pulse=servo_cfg.get('max_pulse', 2500.0),
                        inverted=servo_cfg.get('inverted', False),
                        offset=servo_cfg.get('offset', 0.0)
                    )
                    self.servos[servo.name] = servo
                
                self.get_logger().info(f'Loaded configuration from {config_file}')
                return
            except Exception as e:
                self.get_logger().error(f'Failed to load config: {e}, using defaults')
        
        # Default configuration for quadruped robot
        # Leg structure: Front-Left (FL), Front-Right (FR), Back-Left (BL), Back-Right (BR)
        # Each leg: hip, thigh, calf
        
        default_servos = [
            # Front Left (no inversion)
            {'name': 'FL_hip', 'channel': 0, 'inverted': False},
            {'name': 'FL_thigh', 'channel': 1, 'inverted': False},
            {'name': 'FL_calf', 'channel': 2, 'inverted': False},
            
            # Front Right (inverted)
            {'name': 'FR_hip', 'channel': 3, 'inverted': True},
            {'name': 'FR_thigh', 'channel': 4, 'inverted': True},
            {'name': 'FR_calf', 'channel': 5, 'inverted': True},
            
            # Back Left (no inversion)
            {'name': 'BL_hip', 'channel': 6, 'inverted': False},
            {'name': 'BL_thigh', 'channel': 7, 'inverted': False},
            {'name': 'BL_calf', 'channel': 8, 'inverted': False},
            
            # Back Right (inverted)
            {'name': 'BR_hip', 'channel': 9, 'inverted': True},
            {'name': 'BR_thigh', 'channel': 10, 'inverted': True},
            {'name': 'BR_calf', 'channel': 11, 'inverted': True},
        ]
        
        for servo_cfg in default_servos:
            servo = ServoConfig(
                name=servo_cfg['name'],
                channel=servo_cfg['channel'],
                inverted=servo_cfg['inverted']
            )
            self.servos[servo.name] = servo
        
        self.get_logger().info('Using default servo configuration')
    
    def _command_callback(self, msg: JointState):
        """Handle incoming joint commands."""
        for name, position in zip(msg.name, msg.position):
            if name in self.servos:
                servo = self.servos[name]
                
                # Convert position (radians) to degrees
                angle_deg = math.degrees(position)
                
                # Update current angle
                servo.current_angle = angle_deg
                
                # Convert to pulse and send to servo
                pulse = servo.angle_to_pulse(angle_deg)
                
                if self.pca:
                    self.pca.set_servo_pulse(servo.channel, pulse)
                
                self.get_logger().debug(f'{name}: {angle_deg:.2f}° -> {pulse:.1f}us')
            else:
                self.get_logger().warn(f'Unknown joint: {name}')
    
    def _publish_states(self):
        """Publish current joint states."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        for name, servo in self.servos.items():
            msg.name.append(name)
            msg.position.append(math.radians(servo.current_angle))
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
        
        self.state_pub.publish(msg)
    
    def on_shutdown(self):
        """Cleanup on shutdown."""
        self.get_logger().info('Shutting down servo driver')
        if self.pca:
            self.pca.set_all_off()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
