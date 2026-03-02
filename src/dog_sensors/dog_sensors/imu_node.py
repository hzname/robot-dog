#!/usr/bin/env python3
"""
IMU Node for MPU6050 sensor.
Reads data via I2C and publishes sensor_msgs/Imu and body tilt angles.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import math


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [
        cy * cp * cr + sy * sp * sr,  # w
        cy * cp * sr - sy * sp * cr,  # x
        sy * cp * sr + cy * sp * cr,  # y
        sy * cp * cr - cy * sp * sr,  # z
    ]
    return q


class MPU6050Driver:
    """Driver for MPU6050 IMU sensor via I2C."""
    
    # MPU6050 registers
    MPU6050_ADDR = 0x68
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    
    # Scale factors
    ACCEL_SCALE = 16384.0  # ±2g
    GYRO_SCALE = 131.0     # ±250°/s
    
    def __init__(self, bus=1):
        self.bus_num = bus
        self.bus = None
        self._try_import()
    
    def _try_import(self):
        """Try to import smbus, handle gracefully if not available."""
        try:
            import smbus2
            self.smbus = smbus2
            self._init_bus()
        except ImportError:
            self.smbus = None
            print("Warning: smbus2 not available, using mock data")
        except Exception as e:
            self.smbus = None
            print(f"Warning: Failed to initialize I2C: {e}, using mock data")
    
    def _init_bus(self):
        """Initialize I2C bus."""
        try:
            self.bus = self.smbus.SMBus(self.bus_num)
            # Wake up MPU6050
            self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0)
        except Exception as e:
            print(f"Failed to initialize MPU6050: {e}")
            self.bus = None
    
    def _read_word(self, reg):
        """Read 16-bit word from sensor."""
        if self.bus is None:
            return 0
        try:
            high = self.bus.read_byte_data(self.MPU6050_ADDR, reg)
            low = self.bus.read_byte_data(self.MPU6050_ADDR, reg + 1)
            value = (high << 8) | low
            if value > 32767:
                value -= 65536
            return value
        except Exception as e:
            print(f"I2C read error: {e}")
            return 0
    
    def read_accel(self):
        """Read accelerometer data (g)."""
        if self.smbus is None or self.bus is None:
            # Mock data for simulation
            return (0.0, 0.0, 1.0)
        
        x = self._read_word(self.ACCEL_XOUT_H) / self.ACCEL_SCALE
        y = self._read_word(self.ACCEL_XOUT_H + 2) / self.ACCEL_SCALE
        z = self._read_word(self.ACCEL_XOUT_H + 4) / self.ACCEL_SCALE
        return (x, y, z)
    
    def read_gyro(self):
        """Read gyroscope data (deg/s)."""
        if self.smbus is None or self.bus is None:
            # Mock data for simulation
            return (0.0, 0.0, 0.0)
        
        x = self._read_word(self.GYRO_XOUT_H) / self.GYRO_SCALE
        y = self._read_word(self.GYRO_XOUT_H + 2) / self.GYRO_SCALE
        z = self._read_word(self.GYRO_XOUT_H + 4) / self.GYRO_SCALE
        return (x, y, z)


class IMUNode(Node):
    """ROS2 node for MPU6050 IMU sensor."""
    
    def __init__(self):
        super().__init__('imu_node')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('use_mock', False)
        
        i2c_bus = self.get_parameter('i2c_bus').value
        publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        use_mock = self.get_parameter('use_mock').value
        
        # Initialize sensor
        if use_mock:
            self.driver = None
        else:
            self.driver = MPU6050Driver(bus=i2c_bus)
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.tilt_pub = self.create_publisher(Float32MultiArray, '/imu/tilt', 10)
        
        # Timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Complementary filter state
        self.roll = 0.0   # Forward/backward tilt
        self.pitch = 0.0  # Left/right tilt
        self.yaw = 0.0
        
        self.alpha = 0.98  # Complementary filter coefficient
        self.prev_time = self.get_clock().now()
        
        self.get_logger().info('IMU node initialized')
    
    def timer_callback(self):
        """Read sensor and publish data."""
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time
        
        # Read sensor data
        if self.driver:
            accel = self.driver.read_accel()
            gyro = self.driver.read_gyro()
        else:
            # Mock data for testing
            accel = (0.0, 0.0, 1.0)
            gyro = (0.0, 0.0, 0.0)
        
        ax, ay, az = accel
        gx, gy, gz = gyro
        
        # Convert gyro to rad/s
        gx_rad = math.radians(gx)
        gy_rad = math.radians(gy)
        gz_rad = math.radians(gz)
        
        # Calculate angles from accelerometer
        accel_roll = math.atan2(ay, math.sqrt(ax**2 + az**2))
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        
        # Complementary filter
        self.roll = self.alpha * (self.roll + gx_rad * dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy_rad * dt) + (1 - self.alpha) * accel_pitch
        self.yaw += gz_rad * dt
        
        # Create and publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = self.frame_id
        
        # Orientation (quaternion from Euler)
        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        imu_msg.orientation.w = q[0]
        imu_msg.orientation.x = q[1]
        imu_msg.orientation.y = q[2]
        imu_msg.orientation.z = q[3]
        
        # Orientation covariance (unknown)
        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.orientation_covariance[0] = 0.1
        imu_msg.orientation_covariance[4] = 0.1
        imu_msg.orientation_covariance[8] = 0.1
        
        # Angular velocity
        imu_msg.angular_velocity.x = gx_rad
        imu_msg.angular_velocity.y = gy_rad
        imu_msg.angular_velocity.z = gz_rad
        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[4] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.01
        
        # Linear acceleration
        imu_msg.linear_acceleration.x = ax * 9.80665
        imu_msg.linear_acceleration.y = ay * 9.80665
        imu_msg.linear_acceleration.z = az * 9.80665
        imu_msg.linear_acceleration_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance[0] = 0.1
        imu_msg.linear_acceleration_covariance[4] = 0.1
        imu_msg.linear_acceleration_covariance[8] = 0.1
        
        self.imu_pub.publish(imu_msg)
        
        # Publish tilt angles [roll, pitch] in degrees
        tilt_msg = Float32MultiArray()
        tilt_msg.data = [
            math.degrees(self.roll),
            math.degrees(self.pitch)
        ]
        self.tilt_pub.publish(tilt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
