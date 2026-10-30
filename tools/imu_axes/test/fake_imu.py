#!/usr/bin/env python3
"""A scripted IMU for checking the check: level, nose down, left up, counter-clockwise.

The interactive tool is only as good as its plumbing - subscription, the quaternion and
accelerometer cross-check, the CSV it leaves behind. This publisher replays the four
positions on a timeline so that flow can be exercised without a robot:

    python3 tools/imu_axes/test/fake_imu.py &
    printf '\\n\\n\\n\\n' | python3 tools/imu_axes/run_ros.py --seconds 2 --out /tmp/imu

It also publishes imu/calibrated, so the tool reports a calibrated IMU the way imu_node
would after its bias window.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

G = 9.81
# seconds from start -> (roll_deg, pitch_deg, yaw_rate_dps)
TIMELINE = [(0.0, 0.0, 0.0, 0.0), (4.0, 0.0, 15.0, 0.0), (6.0, 15.0, 0.0, 0.0),
            (8.0, 0.0, 0.0, 40.0)]


def rpy_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class FakeImu(Node):
    def __init__(self):
        super().__init__('fake_imu')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Imu, '/dog/imu/data', 50)
        self.cal = self.create_publisher(Bool, '/dog/imu/calibrated', latched)
        self.start = self.get_clock().now()
        self.create_timer(0.01, self.tick)
        self.announced = False

    def tick(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        roll_deg = pitch_deg = yaw_rate_dps = 0.0
        for at, r, p, w in TIMELINE:
            if elapsed >= at:
                roll_deg, pitch_deg, yaw_rate_dps = r, p, w
        if not self.announced:
            self.cal.publish(Bool(data=True))
            self.announced = True

        r, p = math.radians(roll_deg), math.radians(pitch_deg)
        qx, qy, qz, qw = rpy_to_quaternion(r, p, 0.0)
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = qx, qy, qz, qw
        m.linear_acceleration.x = -G * math.sin(p)
        m.linear_acceleration.y = G * math.sin(r) * math.cos(p)
        m.linear_acceleration.z = G * math.cos(r) * math.cos(p)
        m.angular_velocity.z = math.radians(yaw_rate_dps)
        self.pub.publish(m)


def main():
    rclpy.init()
    node = FakeImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
