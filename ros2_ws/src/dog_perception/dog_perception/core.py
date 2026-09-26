"""Terrain perception without ROS: geometry of the sensors, ground planes,
elevation map and the two detectors (X lidars, VL53L1X).

Frames: body = base_link (x forward, y left, z up, origin at the body centre).
A plane is (n, c): points p on it satisfy n . p = c, n is the unit normal
pointing up (away from the ground), so c = -(height of the origin above it).
"""

import math
import os

import numpy as np

LEGS = (('lf', 1, 1), ('rf', 1, -1), ('lr', -1, 1), ('rr', -1, -1))


# --------------------------------------------------------------- config
def robot_config(path=None):
    """(sensors, geometry, stance) sections of robot.yaml: the given file, the
    installed dog_bringup one, or the one in this source tree. Offline tools
    and the simulation checks must use the same sensor mounts as the robot."""
    import yaml
    if path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('dog_bringup'), 'config', 'robot.yaml')
        except Exception:  # no ROS: the source tree
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'dog_bringup', 'config',
                                'robot.yaml')
    with open(path) as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    return p.get('sensors', {}), p['geometry'], p.get('stance', {})


def gs2_line_x(sensors, stand_height):
    """Body x where the GS2 centre ray meets a flat floor in the stand pose."""
    m = mounts_from_params(sensors).get('gs2')
    if m is None:
        return math.nan
    t = ray_to_plane(m.p, m.beam, (np.array([0.0, 0.0, 1.0]), -stand_height))
    return float(m.p[0] + t * m.beam[0]) if math.isfinite(t) else math.nan


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
    if s.get('gs2'):
        out['gs2'] = SensorMount('gs2', (s['gs2_x'], s['gs2_y'], s['gs2_z']),
                                 (0.0, math.radians(s['gs2_pitch_deg']), 0.0))
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
def feet_body(geometry, q, foot_radius=0.0):
    """Contact points of the four feet in the body frame [(4, 3)] from the joint
    angles q = {joint: rad} (same kinematics as dog_control). calf runs to the
    contact point (robot.yaml, URDF), so there is nothing to subtract."""
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


def lidar_hazards(points, plane, x_range=(0.25, 1.0), thr=0.015, min_points=3, bin_=0.03, bin_points=3):
    """Hazards in the foot corridors ahead from lidar points (body frame) and
    the ground plane under the feet: [(corridor, kind, x_nearest, height, jump)].
    kind 'up' = above the plane (stone, step up, ramp start), 'down' = below
    (hole, step down). height is the median over the corridor; jump the
    largest rise ('up', > 0) or drop ('down', < 0) of the profile within two
    bins along x (3-6 cm): a wall or a step shows its full height there, a
    ramp only its slope (10 deg: ~10 mm). It does not depend on the plane.
    Bins with fewer than bin_points points are skipped: with ~20 points per
    corridor and 10 mm lidar noise, single-point bins made 15-30 mm 'jumps'
    on a flat floor (simulation, 600 scans).
    """
    out = []
    n, c = plane
    res = points @ n - c
    ahead = (points[:, 0] > x_range[0]) & (points[:, 0] < x_range[1])
    for name, (y0, y1) in CORRIDORS.items():
        m = ahead & (points[:, 1] > y0) & (points[:, 1] < y1)
        up_j = down_j = None
        for kind, sel in (('up', res > thr), ('down', res < -thr)):
            k = m & sel
            if k.sum() >= min_points:
                if up_j is None:
                    up_j, down_j = _profile_jumps(points[m, 0], res[m], bin_, bin_points)
                i = np.argmin(points[k, 0])
                out.append((name, kind, float(points[k][i, 0]), float(np.median(res[k])),
                            up_j if kind == 'up' else -down_j))
    return out


def _profile_jumps(x, h, bin_, bin_points=1):
    """Largest rise and largest drop of the binned median profile h(x) over
    one or two bins."""
    b = np.floor(x / bin_).astype(int)
    keys, cnt = np.unique(b, return_counts=True)
    keys = keys[cnt >= bin_points]
    med = np.array([np.median(h[b == k]) for k in keys])
    up = down = 0.0
    for gap in (1, 2):
        if len(keys) > gap:
            ok = keys[gap:] - keys[:-gap] <= 2
            d = (med[gap:] - med[:-gap])[ok]
            if len(d):
                up, down = max(up, float(d.max())), max(down, float(-d.min()))
    return up, down


def gs2_hazards(points, plane, expected_centre, thr_local=0.012, thr_abs=0.015, min_points=3):
    """Hazards on the GS2 line (body-frame points of one scan):
    [(corridor, kind, x, y, height, how)].

    how 'line'  - a stone / pit shorter than the line: the points leave a
                  straight-line fit of the profile itself. Needs no ground
                  reference, so body pitch errors do not matter.
    how 'plane' - the whole corridor is above / below the reference plane
                  (a step across the path; the line fit would follow it).
    how 'gap'   - the centre of the fan sees no floor although it should
                  (hole or step down deeper than the 0.3 m range allows).
    """
    out = []
    y0c, y1c = CORRIDORS['centre']
    if expected_centre and np.count_nonzero((points[:, 1] > y0c) & (points[:, 1] < y1c)) < min_points:
        out.append(('centre', 'down', math.nan, 0.0, math.nan, 'gap'))
    if len(points) < 10:
        return out
    y, z = points[:, 1], points[:, 2]
    # robust line z(y): start from the median level (most of the line is floor)
    r = z - np.median(z)
    for _ in range(4):
        keep = np.abs(r) < max(0.006, 3.0 * 1.4826 * float(np.median(np.abs(r))))
        if keep.sum() < 8:
            break
        k, b = np.polyfit(y[keep], z[keep], 1)
        r = z - (k * y + b)
    res_plane = points @ plane[0] - plane[1] if plane is not None else None
    for name, (y0, y1) in CORRIDORS.items():
        m = (y > y0) & (y < y1)
        if m.sum() < min_points:
            continue
        for kind, sel in (('up', r > thr_local), ('down', r < -thr_local)):
            s = m & sel
            if s.sum() >= min_points:
                i = np.argmin(points[s, 0])
                out.append((name, kind, float(points[s][i, 0]), float(points[s][i, 1]), float(np.median(r[s])), 'line'))
        if res_plane is not None:
            med = float(np.median(res_plane[m]))
            if abs(med) > thr_abs:
                out.append((name, 'up' if med > 0 else 'down', float(np.median(points[m, 0])), (y0 + y1) / 2, med,
                            'plane'))
    return out


class HazardGuard:
    """Turns hazard reports into what the gait should do: a forward speed
    limit and a swing height per leg.

    Reports are kept on a 5 cm grid in the odom frame. A cell counts how
    often it was reported as
      'stop' - an edge too tall even for the crawl (> climb_max) or a drop
               deeper than descend_max (lidars: they see both from above, well ahead);
      'crawl'- an edge above trot_climb (a drop below trot_descend) up to those:
               steps, stairs, high bars - the slow three-legged crawl
               (dog_control/crawl.hpp) takes it with its own swing;
      'step' - can be walked over. Also 'no floor' from GS2 / ToF unless
               deep_stop: both lose the floor whenever the body pitches nose
               up (15-20 deg on a stone or a ramp start in simulation), and
               stopping on that froze the robot in 2 of 6 runs;
    and the highest lift (how high the feet must clear it; 0 = unknown or a
    drop). A cell is confirmed with `confirm` reports in it and its 8
    neighbours ('stop': stop_confirm stop reports) - one stray report does
    nothing. A confirmed stop stays a stop while the cell is still reported
    at all: right in front of a wall the lidars no longer see its edge and
    call it 'step'; a list of reports let those push the stop out of memory
    and the robot walked into the wall (simulation). A cell is forgotten
    `memory` s after its last report or when it is well behind the robot.

    Each tick the cells are seen from the current pose; only those in the
    path (|lateral| < half_width) count, `d` is how far ahead of the body
    centre they are:
      stop cell, d < stop_dist      -> max_vx 0 (front feet ~0.2 m before it)
      crawl cell, -crawl_pass < d < crawl_dist -> 'crawl' (the gait switches
                                       on the spot); once crawling, any cell
                                       in that window keeps it (the next
                                       riser, seen from a tread, may read low)
      a leg has a cell with lift    -> near_vx, that leg swings lift + margin
      any other cell, d > -pass     -> slow_vx
    A leg lifts higher only while a cell with lift lies on its own foot line
    (|lateral - foot y| < leg_width), from leg_behind behind its foot to
    leg_ahead in front of it: front legs first, rear legs when they get there,
    and a stone on the left line never lifts the right legs. The swing stays
    low (max_step 30 mm): a higher swing rocks this trot on its diagonal pair
    (35-50 mm tipped it over on waves, stones and a ramp in simulation).
    Unknown heights (ToF, GS2 'plane': a step and a ramp look the same) only
    slow down. Turning away takes the cells out of the path, so the operator
    can steer around a stop. Only forward motion is limited.
    """

    CELL = 0.05

    def __init__(self, slow_vx=0.08, near_vx=0.05, stop_dist=0.30, pass_dist=0.20, half_width=0.20,
                 trot_climb=0.025, trot_descend=0.035, climb_max=0.07, descend_max=0.07,
                 step_margin=0.01, max_step=0.03, memory=15.0, crawl=True, crawl_dist=0.45, crawl_pass=0.30,
                 feet=((0.09, 0.115), (0.09, -0.115), (-0.09, 0.115), (-0.09, -0.115)),
                 leg_width=0.08, leg_ahead=0.15, leg_behind=0.06, confirm=2, stop_confirm=3, deep_stop=False):
        self.slow_vx, self.near_vx = slow_vx, near_vx
        self.stop_dist, self.pass_dist = stop_dist, pass_dist
        self.half_width = half_width
        self.trot_climb, self.trot_descend = trot_climb, trot_descend
        self.climb_max, self.descend_max = climb_max, descend_max
        self.crawl, self.crawl_dist, self.crawl_pass = crawl, crawl_dist, crawl_pass
        self.crawling = False
        self.step_margin, self.max_step = step_margin, max_step
        self.memory = memory
        self.feet, self.leg_width, self.leg_ahead, self.leg_behind = feet, leg_width, leg_ahead, leg_behind
        self.confirm, self.stop_confirm = confirm, stop_confirm
        self.deep_stop = deep_stop
        self.cells = {}  # (i, j) -> [sum x, sum y, n, n_stop, t_last, lift, n_crawl]

    def verdict(self, kind, edge, deep=False):
        """'stop', 'crawl' or 'step' for a hazard of this kind and edge height
        [m] (NaN = unknown). edge must be a jump over a few cm, not a height
        over the reference plane: a ramp is high above the plane but has no edge."""
        if deep:  # no floor seen: GS2 gap, ToF without a return
            return 'stop' if self.deep_stop else 'step'
        if not math.isfinite(edge):
            return 'step'
        h = edge if kind == 'up' else -edge
        trot, crawl = (self.trot_climb, self.climb_max) if kind == 'up' else (self.trot_descend, self.descend_max)
        if h <= trot:
            return 'step'
        return 'crawl' if self.crawl and h <= crawl else 'stop'

    def add(self, t, xy, verdict, lift=0.0):
        """lift: height the feet must clear [m]; 0 or NaN = only slow down."""
        key = (math.floor(xy[0] / self.CELL), math.floor(xy[1] / self.CELL))
        c = self.cells.setdefault(key, [0.0, 0.0, 0, 0, t, 0.0, 0])
        c[0] += xy[0]
        c[1] += xy[1]
        c[2] += 1
        c[3] += verdict == 'stop'
        c[6] += verdict == 'crawl'
        c[4] = t
        if math.isfinite(lift) and lift > c[5]:
            c[5] = lift

    def command(self, t, xy, yaw):
        """(max_vx or inf, [swing height per leg LF, RF, LR, RR or NaN], state,
        nearest d) for the pose (x, y, yaw). state 'crawl': switch to the crawl."""
        none = [math.nan] * len(self.feet)
        cs, sn = math.cos(yaw), math.sin(yaw)
        live = {}
        behind = max(self.pass_dist, self.crawl_pass) + 0.2
        for key, c in self.cells.items():
            if t - c[4] >= self.memory:
                continue
            dx, dy = c[0] / c[2] - xy[0], c[1] / c[2] - xy[1]
            d, lat = cs * dx + sn * dy, -sn * dx + cs * dy
            if d < -behind and abs(lat) < 0.6:  # well behind
                continue
            live[key] = (c, d, lat)
        self.cells = {k: v[0] for k, v in live.items()}
        dmin, dstop, dcrawl, steps = math.inf, math.inf, math.inf, list(none)
        crawl = False
        for (i, j), (c, d, lat) in live.items():
            if not abs(lat) < self.half_width:
                continue
            n = n_stop = n_crawl = 0
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = live.get((i + di, j + dj))
                    if nb:
                        n += nb[0][2]
                        n_stop += nb[0][3]
                        n_crawl += nb[0][6]
            if n < self.confirm:
                continue
            if -self.crawl_pass < d < self.crawl_dist and (
                    (n_crawl >= self.stop_confirm and c[6] > 0) or self.crawling):
                crawl = True
                dcrawl = min(dcrawl, d)
            if d <= -self.pass_dist:
                continue
            dmin = min(dmin, d)
            if n_stop >= self.stop_confirm and c[3] > 0 and d > -0.1:
                dstop = min(dstop, d)
            if c[5] > 0:
                for k, (fx, fy) in enumerate(self.feet):
                    if abs(lat - fy) < self.leg_width and fx - self.leg_behind < d < fx + self.leg_ahead:
                        h = min(self.max_step, c[5] + self.step_margin)
                        steps[k] = h if math.isnan(steps[k]) else max(steps[k], h)
        self.crawling = crawl
        if dstop < self.stop_dist:
            return 0.0, none, 'stop', dstop
        if crawl:  # slow by itself, clears the terrain with its own swing
            return math.inf, none, 'crawl', dcrawl
        if not math.isfinite(dmin):
            return math.inf, none, 'clear', math.nan
        if any(math.isfinite(h) for h in steps):
            return self.near_vx, steps, 'step_over', dmin
        return self.slow_vx, steps, 'caution', dmin


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
