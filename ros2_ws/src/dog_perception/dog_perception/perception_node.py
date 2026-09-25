"""perception_node: terrain perception from the crossed (X) lidars and the
VL53L1X ToF sensors.

Subscribes (relative, /dog namespace):
  lidar_left/scan, lidar_right/scan  sensor_msgs/LaserScan
  tof/<name>                         sensor_msgs/Range
  joint_states, imu/data, state, odom
Publishes:
  perception/ground_lidar  geometry_msgs/Vector3Stamped  x roll, y pitch [rad],
                           z body height [m] relative to the floor plane seen by the lidars
  perception/ground_feet   same, from the leg kinematics (four-leg support + IMU)
  perception/hazards       std_msgs/String  JSON list of hazards ahead of the feet
  perception/tof           std_msgs/Float32MultiArray  every ToF reading:
                           [sensor index, measured, expected, residual] (NaN = none)
  perception/map           std_msgs/Float32MultiArray  elevation map (1 Hz):
                           [origin_x, origin_y, resolution, n, n*n mean heights (NaN = unknown)]
  perception/stats         std_msgs/Float64MultiArray  [process CPU s, lidar cycles,
                           lidar points, lidar ms total, ToF messages, ToF ms total,
                           messages received (all topics), their callback ms total,
                           then count / callback ms for joint_states, imu, odom]

Every reading is compared with the ground plane at the reading's own time
stamp: the body pitches by several degrees at ~2 Hz in the trot, and 50 ms
of skew between a sensor and the IMU is already 1 deg.

The map is kept in the odom frame. In simulation odom is the ground truth; on
the robot it will come from an estimate (IMU + commanded motion) and drift.
"""

import bisect
import collections
import json
import math
import os
import time

# One BLAS thread: the SVDs are tiny, and multi-threaded BLAS busy-waits on
# every core (measured: 49 s CPU in 30 s) - on the robot it would starve the gait.
for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')

import numpy as np  # noqa: E402
import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan, Range
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

from . import core

SENSOR_DEFAULTS = {
    'x_lidar': True, 'x_lidar_x': 0.10, 'x_lidar_y': 0.04, 'x_lidar_z': 0.05,
    'x_lidar_tilt_deg': 30.0, 'x_lidar_yaw_deg': 40.0,
    'tof': True, 'tof_names': ['fl', 'fr', 'fc', 'rc'], 'tof_x': [0.115, 0.115, 0.115, -0.115],
    'tof_y': [0.045, -0.045, 0.0, 0.0], 'tof_z': [-0.012, -0.012, 0.0, -0.012],
    'tof_pitch_deg': [40.0, 40.0, 20.0, 40.0], 'tof_yaw_deg': [23.0, -23.0, 0.0, 180.0],
}
GEOMETRY_DEFAULTS = {'hip_offset': 0.055, 'thigh': 0.105, 'calf': 0.105, 'hip_x': 0.09, 'hip_y': 0.06}


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')
        s = {k: self.declare_parameter(f'sensors.{k}', v).value for k, v in SENSOR_DEFAULTS.items()}
        self.geometry = {k: self.declare_parameter(f'geometry.{k}', v).value
                         for k, v in GEOMETRY_DEFAULTS.items()}
        self.thr = self.declare_parameter('perception.threshold', 0.015).value
        # FC looks far and flat (20 deg): 1 deg of pitch error is 23 mm there
        tof_thr = self.declare_parameter('perception.tof_threshold', [0.015, 0.015, 0.05, 0.015]).value
        self.tof_confirm = self.declare_parameter('perception.tof_confirm', 3).value
        # per-sensor range offsets [m] measured once on a known flat floor and
        # kept in the config. Auto-calibration at every start (tof_autocal)
        # is off: whatever the sensors see at the start (a ramp, a threshold)
        # would become a permanent offset - found in the slope simulation.
        tof_offsets = self.declare_parameter('perception.tof_offsets', [0.0, 0.0, 0.0, 0.0]).value
        self.tof_autocal = self.declare_parameter('perception.tof_autocal', False).value
        # reference ground for the detectors: 'auto' = the lidar plane when it
        # agrees with the legs (0.05 deg in simulation), else the legs (~1 deg);
        # 'feet' = legs only (a robot with ToF sensors but no lidars)
        self.reference = self.declare_parameter('perception.reference', 'auto').value
        self.lidar_plane = None  # (plane, R at the scan, t)
        self.map_size = self.declare_parameter('perception.map_size', 3.0).value
        self.map_res = self.declare_parameter('perception.map_resolution', 0.02).value
        self.mounts = core.mounts_from_params(s)
        self.feet = core.FeetPlane()
        self.R_imu = None
        self.imu_hist = collections.deque(maxlen=400)  # (t [s], R), ~4 s at 100 Hz
        self.state = None
        self.odom = None
        self.scans = {}
        self.map = core.ElevationMap(self.map_size, self.map_res)
        self.stats = np.zeros(14)
        self.tof = {n: core.TofDetector(self.mounts[f'tof_{n}'], tof_thr[k], confirm=self.tof_confirm,
                                        offset=tof_offsets[k], baseline_n=50 if self.tof_autocal else 0)
                    for k, n in enumerate(s['tof_names'] if s['tof'] else [])}
        self.tof_index = {n: k for k, n in enumerate(self.tof)}

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'state', lambda m: setattr(self, 'state', m.data), latched)
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        if s['x_lidar']:
            for name in ('lidar_left', 'lidar_right'):
                self.create_subscription(LaserScan, f'{name}/scan',
                                         lambda m, n=name: self.on_scan(n, m), qos_profile_sensor_data)
        for n in self.tof:
            self.create_subscription(Range, f'tof/{n}', lambda m, n=n: self.on_tof(n, m),
                                     qos_profile_sensor_data)
        self.pub_ground_lidar = self.create_publisher(Vector3Stamped, 'perception/ground_lidar', 10)
        self.pub_ground_feet = self.create_publisher(Vector3Stamped, 'perception/ground_feet', 10)
        self.pub_hazards = self.create_publisher(String, 'perception/hazards', 10)
        self.pub_tof = self.create_publisher(Float32MultiArray, 'perception/tof', 10)
        self.pub_map = self.create_publisher(Float32MultiArray, 'perception/map', 1)
        self.pub_stats = self.create_publisher(Float64MultiArray, 'perception/stats', 1)
        self.create_timer(1.0, self.publish_slow)
        self.get_logger().info(f'perception: {", ".join(self.mounts)}')

    # ------------------------------------------------------------ inputs
    def on_imu(self, msg):
        t0 = time.perf_counter()
        q = msg.orientation
        self.R_imu = core.quat_to_rot(q.x, q.y, q.z, q.w)
        self.imu_hist.append((_t(msg.header.stamp), self.R_imu))
        self._count(t0, 10)

    def on_odom(self, msg):
        t0 = time.perf_counter()
        self.odom = msg
        self._count(t0, 12)

    def R_at(self, stamp):
        """IMU attitude at a time stamp (nearest sample)."""
        if not self.imu_hist:
            return None
        t = _t(stamp)
        ts = [h[0] for h in self.imu_hist]
        i = bisect.bisect_left(ts, t)
        if i == 0:
            return self.imu_hist[0][1]
        if i >= len(ts):
            return self.imu_hist[-1][1]
        return self.imu_hist[i][1] if ts[i] - t < t - ts[i - 1] else self.imu_hist[i - 1][1]

    def on_joints(self, msg):
        t0 = time.perf_counter()
        feet = core.feet_body(self.geometry, dict(zip(msg.name, msg.position)))
        self.feet.update_feet(feet, self.R_at(msg.header.stamp))
        self._count(t0, 8)

    def ground(self, stamp=None):
        R = self.R_at(stamp) if stamp is not None else self.R_imu
        feet = self.feet.current(R)
        lp = self.lidar_plane
        if self.reference == 'auto' and lp is not None and stamp is not None and abs(_t(stamp) - lp[2]) < 0.3:
            (n, c), R0, _ = lp
            if R is not None and R0 is not None:
                n = (R0.T @ R).T @ n
            if feet is None or abs(c - feet[1]) < 0.015:
                return n, c
        return feet

    def _count(self, t0, slot=None):
        ms = 1e3 * (time.perf_counter() - t0)
        self.stats[6:8] += (1, ms)
        if slot is not None:
            self.stats[slot:slot + 2] += (1, ms)

    def upright(self):
        return self.state in ('stand', 'walk')

    # ------------------------------------------------------------ lidars
    def on_scan(self, name, msg):
        t0 = time.perf_counter()
        pts = core.scan_to_body(self.mounts[name], msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max)
        # own legs and body: nothing inside the robot's footprint
        pts = pts[~((np.abs(pts[:, 0]) < 0.22) & (np.abs(pts[:, 1]) < 0.17))]
        stamp = msg.header.stamp
        now, R_now = _t(stamp), self.R_at(stamp)
        self.scans[name] = (now, pts, R_now)
        other = self.scans.get('lidar_right' if name == 'lidar_left' else 'lidar_left')
        if other is None or now - other[0] > 0.15 or not self.upright():
            self.stats[1:4] += (0, len(pts), 1e3 * (time.perf_counter() - t0))
            self._count(t0)
            return
        # the other scan is up to 0.1 s old: turn it by the body rotation since
        # (the trot pitches the body ~10 deg/s: 1 deg of plane error otherwise)
        o_pts = other[1]
        if R_now is not None and other[2] is not None:
            o_pts = o_pts @ (R_now.T @ other[2]).T
        both = np.vstack([pts, o_pts])
        # 1. floor plane seen by the lidars (near band, below the body)
        near = both[(np.hypot(both[:, 0], both[:, 1]) < 1.2) & (both[:, 2] < -0.05)]
        fit = core.robust_plane(near)
        if fit is not None:
            (n, c), inl, rms = fit
            roll, pitch = core.roll_pitch_of_normal(n)
            self.pub_ground_lidar.publish(_v3(stamp, roll, pitch, -c))
            if rms < 0.008 and inl.mean() > 0.6:
                self.lidar_plane = ((n, c), R_now, now)
        # 2. hazards in the foot corridors relative to the ground under the feet
        feet = self.feet.current(R_now)
        if feet is not None:
            rf, pf = core.roll_pitch_of_normal(feet[0])
            self.pub_ground_feet.publish(_v3(stamp, rf, pf, -feet[1]))
        ref = self.ground(stamp) or (fit[0] if fit is not None else None)
        if ref is not None:
            hz = core.lidar_hazards(both, ref, thr=self.thr)
            if hz:
                self.publish_hazards([{'source': 'lidar', 'corridor': c, 'kind': k, 'x': round(x, 3),
                                       'h': round(h, 3)} for c, k, x, h in hz])
        # 3. elevation map in the odom frame
        if self.odom is not None:
            p = self.odom.pose.pose
            R = core.quat_to_rot(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            world = both @ R.T + np.array([p.position.x, p.position.y, p.position.z])
            self.map.recenter([p.position.x, p.position.y])
            self.map.insert(world)
        self.stats[1:4] += (1, len(pts), 1e3 * (time.perf_counter() - t0))
        self._count(t0)

    # ------------------------------------------------------------ ToF
    def on_tof(self, name, msg):
        t0 = time.perf_counter()
        plane = self.ground(msg.header.stamp)
        det = self.tof[name]
        if plane is not None and self.upright():
            if len(det.offsets) < det.baseline_n and self.state == 'stand':
                det.calibrate(msg.range, plane)
            elif len(det.offsets) >= det.baseline_n:
                verdict, exp, res = det.check(msg.range, plane)
                self.pub_tof.publish(Float32MultiArray(data=[
                    float(self.tof_index[name]), _f(msg.range), _f(exp), _f(res)]))
                if verdict:
                    self.publish_hazards([{'source': f'tof_{name}', 'kind': verdict,
                                           'range': _num(msg.range), 'expected': _num(exp),
                                           'residual': _num(res)}])
        self.stats[4:6] += (1, 1e3 * (time.perf_counter() - t0))
        self._count(t0)

    # ------------------------------------------------------------ outputs
    def publish_hazards(self, items):
        t = self.get_clock().now().nanoseconds * 1e-9
        for it in items:
            it['t'] = round(t, 3)
        self.pub_hazards.publish(String(data=json.dumps(items)))

    def publish_slow(self):
        self.stats[0] = time.process_time()
        self.pub_stats.publish(Float64MultiArray(data=[float(v) for v in self.stats]))
        m = self.map.mean()
        self.pub_map.publish(Float32MultiArray(
            data=[float(self.map.origin[0]), float(self.map.origin[1]), float(self.map.res),
                  float(self.map.n)] + m.astype(np.float32).ravel().tolist()))


def _v3(stamp, x, y, z):
    m = Vector3Stamped()
    m.header.stamp = stamp
    m.header.frame_id = 'base_link'
    m.vector.x, m.vector.y, m.vector.z = float(x), float(y), float(z)
    return m


def _t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _f(v):
    return float(v) if math.isfinite(v) else math.nan


def _num(v):
    return round(float(v), 4) if math.isfinite(v) else None


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
