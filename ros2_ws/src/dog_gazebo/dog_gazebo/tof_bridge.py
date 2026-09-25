"""Turns the simulated VL53L1X cones (gpu_lidar, 5 x 5 rays) into
sensor_msgs/Range messages, like the real sensor delivers them.

The VL53L1X reports one distance: the peak of its photon histogram over the
whole 27 deg cone. For a surface it is close to the median of the rays; a
small object inside the cone pulls it closer only when it fills a good part
of the cone. The median of the rays reproduces both. No target in range ->
+inf (the real driver reports "out of range" the same way).
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


class TofBridge(Node):
    def __init__(self):
        super().__init__('tof_bridge')
        names = self.declare_parameter('names', ['fl', 'fr', 'fc', 'rc']).value
        self.fov = math.radians(self.declare_parameter('fov_deg', 27.0).value)
        self.pubs = {}
        for n in names:
            self.pubs[n] = self.create_publisher(Range, f'tof/{n}', qos_profile_sensor_data)
            self.create_subscription(LaserScan, f'sim/tof_{n}/scan',
                                     lambda m, n=n: self.on_scan(n, m), qos_profile_sensor_data)

    def on_scan(self, name, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=float)
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        out = Range()
        out.header = msg.header
        out.header.frame_id = f'tof_{name}'
        out.radiation_type = Range.INFRARED
        out.field_of_view = self.fov
        out.min_range, out.max_range = 0.04, 1.3
        # most rays must see a target, else the sensor reports no target
        out.range = float(np.median(r[ok])) if ok.sum() >= r.size // 2 else math.inf
        self.pubs[name].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TofBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
