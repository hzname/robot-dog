"""Terrain perception without ROS: geometry of the sensors, ground planes,
elevation map and the two detectors (X lidars, VL53L1X).

Frames: body = base_link (x forward, y left, z up, origin at the body centre).
A plane is (n, c): points p on it satisfy n . p = c, n is the unit normal
pointing up (away from the ground), so c = -(height of the origin above it).
"""

import math

import numpy as np

LEGS = (('lf', 1, 1), ('rf', 1, -1), ('lr', -1, 1), ('rr', -1, -1))


# --------------------------------------------------------------- rotations
def rot_rpy(roll, pitch, yaw):
    """URDF convention: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_to_rot(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def roll_pitch_of_normal(n):
    """Body roll / pitch [rad] relative to a plane with normal n (body frame):
    zero when the body is parallel to it; REP-103 signs (pitch + = nose down)."""
    roll = math.atan2(n[1], n[2])
    pitch = math.atan2(-n[0], math.hypot(n[1], n[2]))
    return roll, pitch


# --------------------------------------------------------------- sensors
class SensorMount:
    """A sensor fixed to the body: origin p and rotation R (sensor x = beam)."""

    def __init__(self, name, xyz, rpy):
        self.name = name
        self.p = np.asarray(xyz, float)
        self.R = rot_rpy(*rpy)

    @property
    def beam(self):
        return self.R[:, 0]


def mounts_from_params(s):
    """SensorMounts from the 'sensors' section of robot.yaml (same rules as
    dog_description.urdf.sensor_frames)."""
    out = {}
    if s.get('x_lidar'):
        tilt, yaw = math.radians(s['x_lidar_tilt_deg']), math.radians(s['x_lidar_yaw_deg'])
        for name, side in (('lidar_left', 1), ('lidar_right', -1)):
            out[name] = SensorMount(name, (s['x_lidar_x'], side * s['x_lidar_y'], s['x_lidar_z']),
                                    (0.0, tilt, -side * yaw))
    if s.get('tof'):
        for k, n in enumerate(s['tof_names']):
            out[f'tof_{n}'] = SensorMount(
                f'tof_{n}', (s['tof_x'][k], s['tof_y'][k], s['tof_z'][k]),
                (0.0, math.radians(s['tof_pitch_deg'][k]), math.radians(s['tof_yaw_deg'][k])))
    return out


def scan_to_body(mount, ranges, angle_min, angle_inc, rmin=0.03, rmax=12.0):
    """LaserScan ranges -> Nx3 points in the body frame (invalid ones dropped)."""
    r = np.asarray(ranges, float)
    a = angle_min + angle_inc * np.arange(r.size)
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    r, a = r[ok], a[ok]
    local = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)
    return local @ mount.R.T + mount.p


def ray_to_plane(p, u, plane):
    """Distance along the unit ray p + t u to the plane, inf if it never hits."""
    n, c = plane
    den = float(n @ u)
    if den >= -1e-6:
        return math.inf
    t = (c - float(n @ p)) / den
    return t if t > 0 else math.inf


# --------------------------------------------------------------- planes
def fit_plane(points):
    """Least-squares plane through Nx3 points, normal pointing up (+z)."""
    ctr = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - ctr, full_matrices=False)
    n = vt[2]
    if n[2] < 0:
        n = -n
    return n, float(n @ ctr)


def robust_plane(points, iterations=3, keep=0.02):
    """Plane fit with outlier rejection (residual > max(keep, 2.5 sigma)).
    Returns (plane, inlier mask, rms) or None with too few points."""
    if len(points) < 10:
        return None
    mask = np.ones(len(points), bool)
    plane = None
    for _ in range(iterations):
        if mask.sum() < 10:
            return None
        plane = fit_plane(points[mask])
        res = points @ plane[0] - plane[1]
        sig = float(np.std(res[mask]))
        mask = np.abs(res) < max(keep, 2.5 * sig)
    res = points[mask] @ plane[0] - plane[1]
    return plane, mask, float(np.sqrt(np.mean(res ** 2)))


# --------------------------------------------------------------- legs
def feet_body(geometry, q, foot_radius=0.012):
    """Contact points of the four feet in the body frame [(4, 3)] from the joint
    angles q = {joint: rad} (same kinematics as dog_control)."""
    L1, L2, L3 = geometry['hip_offset'], geometry['thigh'], geometry['calf']
    out = []
    for leg, fr, sd in LEGS:
        q0, q1, q2 = (q.get(f'{leg}_{j}_joint', 0.0) for j in ('hip', 'thigh', 'calf'))
        hip = np.array([fr * geometry['hip_x'], sd * geometry['hip_y'], 0.0])
        A = rot_rpy(q0, 0, 0)
        th = hip + A @ np.array([0.0, sd * L1, 0.0])
        B = A @ rot_rpy(0, q1, 0)
        kn = th + B @ np.array([0.0, 0.0, -L2])
        ft = kn + B @ rot_rpy(0, q2, 0) @ np.array([0.0, 0.0, -L3])
        out.append(ft - np.array([0.0, 0.0, foot_radius]))
    return np.array(out)


class FeetPlane:
    """Ground under the robot from the leg kinematics.

    The plane is taken when all four feet lie on one plane (the four-leg
    support phases of the trot, and standing); in between it is carried along
    with the body rotation measured by the IMU."""

    def __init__(self, tolerance=0.0015):
        self.tol = tolerance
        self.plane = None      # in the body frame at the time it was taken
        self.R_at = None       # IMU attitude then
        self.age = math.inf

    def update_feet(self, feet, R_imu=None, t=0.0):
        plane = fit_plane(feet)
        if np.max(np.abs(feet @ plane[0] - plane[1])) < self.tol:
            self.plane, self.R_at, self.t = plane, R_imu, t
            return True
        return False

    def current(self, R_imu=None):
        """Plane in the current body frame."""
        if self.plane is None:
            return None
        n, c = self.plane
        if R_imu is not None and self.R_at is not None:
            # body rotated by dR since then: a fixed world plane moves by dR^T
            dR = self.R_at.T @ R_imu
            n = dR.T @ n
        return n, c


# --------------------------------------------------------------- elevation map
class ElevationMap:
    """Rolling grid in the world (odom) frame: mean and max height per cell."""

    def __init__(self, size=3.0, res=0.02):
        self.res, self.n = res, int(round(size / res))
        self.origin = np.zeros(2)          # world xy of cell (0, 0)
        self.sum = np.zeros((self.n, self.n))
        self.cnt = np.zeros((self.n, self.n), np.int32)
        self.max = np.full((self.n, self.n), -np.inf)

    def recenter(self, xy):
        """Keep the robot in the middle; cells that leave the window are dropped."""
        new = np.floor((np.asarray(xy) - self.n * self.res / 2) / self.res) * self.res
        shift = np.round((new - self.origin) / self.res).astype(int)
        if not shift.any():
            return
        for arr, fill in ((self.sum, 0.0), (self.cnt, 0), (self.max, -np.inf)):
            arr[:] = _shifted(arr, shift, fill)
        self.origin = new

    def insert(self, pts):
        ij = np.floor((pts[:, :2] - self.origin) / self.res).astype(int)
        ok = (ij >= 0).all(axis=1) & (ij < self.n).all(axis=1)
        ij, z = ij[ok], pts[ok, 2]
        np.add.at(self.sum, (ij[:, 0], ij[:, 1]), z)
        np.add.at(self.cnt, (ij[:, 0], ij[:, 1]), 1)
        np.maximum.at(self.max, (ij[:, 0], ij[:, 1]), z)

    def mean(self):
        with np.errstate(invalid='ignore', divide='ignore'):
            return np.where(self.cnt > 0, self.sum / np.maximum(self.cnt, 1), np.nan)

    def centers(self):
        c = self.origin[:, None] + (np.arange(self.n) + 0.5) * self.res
        return np.meshgrid(c[0], c[1], indexing='ij')


def _shifted(a, shift, fill):
    out = np.full_like(a, fill)
    dx, dy = shift
    n = a.shape[0]
    xs, xd = (slice(dx, n), slice(0, n - dx)) if dx >= 0 else (slice(0, n + dx), slice(-dx, n))
    ys, yd = (slice(dy, n), slice(0, n - dy)) if dy >= 0 else (slice(0, n + dy), slice(-dy, n))
    if abs(dx) < n and abs(dy) < n:
        out[xd, yd] = a[xs, ys]
    return out


# --------------------------------------------------------------- detectors
CORRIDORS = {'left': (0.06, 0.18), 'centre': (-0.06, 0.06), 'right': (-0.18, -0.06)}


def lidar_hazards(points, plane, x_range=(0.25, 1.0), thr=0.015, min_points=3):
    """Hazards in the foot corridors ahead from lidar points (body frame) and
    the ground plane under the feet: [(corridor, kind, x_nearest, height)].
    kind 'up' = above the plane (stone, step up, ramp start), 'down' = below
    (hole, step down)."""
    out = []
    n, c = plane
    res = points @ n - c
    ahead = (points[:, 0] > x_range[0]) & (points[:, 0] < x_range[1])
    for name, (y0, y1) in CORRIDORS.items():
        m = ahead & (points[:, 1] > y0) & (points[:, 1] < y1)
        for kind, sel in (('up', res > thr), ('down', res < -thr)):
            k = m & sel
            if k.sum() >= min_points:
                i = np.argmin(points[k, 0])
                out.append((name, kind, float(points[k][i, 0]), float(np.median(res[k]))))
    return out


class TofDetector:
    """VL53L1X hazard check: measured range vs. the expected distance to the
    ground plane under the feet, minus a per-sensor offset. The offset comes
    from the config (measured once on a flat floor) or, with baseline_n > 0,
    from the first readings while standing. `confirm` readings in a row must agree."""

    def __init__(self, mount, thr=0.02, max_expected=1.0, baseline_n=50, confirm=2, offset=0.0):
        self.m, self.thr, self.max_expected = mount, thr, max_expected
        self.baseline_n, self.confirm = baseline_n, confirm
        self.offsets = []
        self.offset = offset
        self.last, self.run = None, 0

    def check(self, measured, plane):
        exp = ray_to_plane(self.m.p, self.m.beam, plane)
        if not math.isfinite(exp) or exp > self.max_expected:
            return None, exp, math.nan
        if not math.isfinite(measured):
            kind, res = 'down', math.inf
        else:
            res = measured - exp - self.offset
            kind = 'up' if res < -self.thr else ('down' if res > self.thr else None)
        self.run = self.run + 1 if kind == self.last else 1
        self.last = kind
        verdict = kind if (kind is not None and self.run >= self.confirm) else None
        return verdict, exp, res

    def calibrate(self, measured, plane):
        exp = ray_to_plane(self.m.p, self.m.beam, plane)
        if math.isfinite(measured) and math.isfinite(exp) and len(self.offsets) < self.baseline_n:
            self.offsets.append(measured - exp)
            self.offset = float(np.median(self.offsets))
        return len(self.offsets) >= self.baseline_n
