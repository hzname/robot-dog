"""Virtual robot + camera for testing the calibration without hardware.

FakeRobot behaves like the robot's calibration channel, but its servos follow
a hidden "true" calibration while it reports the "believed" one. The virtual
camera renders ArUco markers where the true joint angles put them, so the
whole pipeline (detection -> geometry -> fit) is exercised end to end.
"""

import copy
import math

import numpy as np

from . import geometry as geo
from .servo_model import ServoCal
from .vision import Intrinsics, cv2, marker_image

JOINTS = [f'{l}_{j}_joint' for l in geo.LEGS for j in ('hip', 'thigh', 'calf')]
GEOMETRY = {'hip_offset': 0.055, 'thigh': 0.105, 'calf': 0.105, 'hip_x': 0.09, 'hip_y': 0.06,
            'knee_direction': -1}


def default_cal(joint):
    """servos.yaml defaults."""
    i = JOINTS.index(joint)
    right = joint[0] == 'r'
    kind = joint.split('_')[1]
    lim = {'hip': (0.0, -40.0, 40.0), 'thigh': (45.0, -45.0, 135.0), 'calf': (-90.0, -165.0, -15.0)}[kind]
    return ServoCal(channel=i, direction=-1 if right else 1, offset_deg=lim[0],
                    min_deg=lim[1], max_deg=lim[2])


def cal_to_params(cal: ServoCal):
    d = {k: getattr(cal, k) for k in ('channel', 'direction', 'offset_deg', 'pulse_min_us',
                                      'pulse_max_us', 'range_deg', 'min_deg', 'max_deg',
                                      'coupled_to', 'coupling')}
    d.update({k: getattr(cal.linkage, k) for k in ('servo_arm_mm', 'joint_arm_mm', 'rod_mm',
                                                   'axis_distance_mm')})
    return d


class FakeRobot:
    def __init__(self, true_cal: dict, believed_cal: dict = None, geometry=None, stops=None):
        self.true = true_cal
        self.believed = believed_cal or {j: default_cal(j) for j in JOINTS}
        self.geometry = dict(geometry or GEOMETRY)
        self.stops = stops or {}          # joint -> (min_deg, max_deg) mechanical hard stops
        self.pulses = {j: 0.0 for j in JOINTS}
        self.cmd = {j: 0.0 for j in JOINTS}
        self.stalled = set()

    # --- calibration channel API (same as client.RobotClient)
    def info(self):
        return {'joints': list(JOINTS), 'geometry': dict(self.geometry),
                'calibration': {j: cal_to_params(c) for j, c in self.believed.items()}}

    def pose(self, joints_deg: dict):
        self.cmd.update(joints_deg)
        for j in JOINTS:
            c = self.believed[j]
            parent = self.cmd.get(c.coupled_to, 0.0) if c.coupled_to else 0.0
            self.pulses[j] = c.joint_to_pulse(self.cmd[j], parent)

    def status(self):
        current = 0.4 + 2.0 * len(self.stalled)
        return {'pulses': dict(self.pulses), 'power': {'voltage': 6.0, 'current': current}}

    def set_params(self, params: dict):
        for key, value in params.items():
            j, field = key.rsplit('.', 1)
            c = self.believed[j]
            if field in ('servo_arm_mm', 'joint_arm_mm', 'rod_mm', 'axis_distance_mm'):
                setattr(c.linkage, field, float(value))
            elif field == 'coupled_to':
                c.coupled_to = str(value)
            elif field in ('channel', 'direction'):
                setattr(c, field, int(value))
            else:
                setattr(c, field, float(value))
        self.pose({})
        return True, ''

    def settle(self):
        pass

    # --- ground truth
    def true_angles(self):
        """Actual joint angles produced by the pulses, honouring hard stops."""
        out, self.stalled = {}, set()
        for leg in geo.LEGS:
            for kind in ('hip', 'thigh', 'calf'):
                j = f'{leg}_{kind}_joint'
                c = self.true[j]
                parent = out.get(c.coupled_to, 0.0) if c.coupled_to else 0.0
                q = c.pulse_to_joint(self.pulses[j], parent)
                if j in self.stops:
                    lo, hi = self.stops[j]
                    if q < lo or q > hi:
                        self.stalled.add(j)
                    q = min(max(q, lo), hi)
                out[j] = q
        return out


# Camera placement per view, body frame: rows = camera x, y, z axes; position [m].
_CAMERAS = {
    'left': (np.array([[-1, 0, 0], [0, 0, -1], [0, -1, 0]], float), np.array([0.0, 0.55, -0.07])),
    'right': (np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float), np.array([0.0, -0.55, -0.07])),
    'front': (np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]], float), np.array([0.55, 0.0, -0.07])),
    'rear': (np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float), np.array([-0.55, 0.0, -0.07])),
}


def _tilt(R, deg):
    """Small camera roll + pitch, for robustness tests."""
    a, b = math.radians(deg), math.radians(deg * 0.6)
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, math.cos(b), -math.sin(b)], [0, math.sin(b), math.cos(b)]])
    return Rz @ Rx @ R


def marker_points_body(robot: FakeRobot, view):
    """Body-frame centres [m] of the markers used by `view`."""
    q = robot.true_angles()
    g = robot.geometry
    pts = {}
    for leg, ids in geo.VIEWS[view]['legs'].items():
        hip, thigh, knee, foot = geo.leg_points(g, leg, q[f'{leg}_hip_joint'],
                                                q[f'{leg}_thigh_joint'], q[f'{leg}_calf_joint'])
        if geo.VIEWS[view]['kind'] == 'side':
            pts[ids[0]], pts[ids[1]], pts[ids[2]] = thigh, knee, foot
        else:
            f = geo.FRONT[leg]
            pts[ids[0]] = hip + np.array([f * 0.02, 0.0, 0.0])   # on the body's end face
            pts[ids[1]] = foot + np.array([f * 0.015, 0.0, 0.0])  # tab in front of the foot
    return pts


class VirtualCamera:
    def __init__(self, robot: FakeRobot, view, width=1280, height=720, marker_mm=22.0,
                 tilt_deg=3.0, noise=2.0, seed=0):
        self.robot, self.view = robot, view
        self.w, self.h = width, height
        self.marker_m = marker_mm / 1000.0
        R, t = _CAMERAS[view]
        self.R, self.t = _tilt(R, tilt_deg), t
        self.intr = Intrinsics.approximate(width, height, 65.0)
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self._tiles = {}

    def _project(self, p_cam):
        K = self.intr.K
        return np.array([K[0, 0] * p_cam[0] / p_cam[2] + K[0, 2], K[1, 1] * p_cam[1] / p_cam[2] + K[1, 2]])

    def render(self):
        img = np.full((self.h, self.w), 170, np.uint8)
        pts = marker_points_body(self.robot, self.view)
        # markers face the camera: square in the camera's x-y plane
        order = sorted(pts, key=lambda i: -(self.R @ (pts[i] - self.t))[2])  # far first
        for i in order:
            c = self.R @ (pts[i] - self.t)
            h = self.marker_m / 2.0
            corners = np.array([self._project(c + np.array(d)) for d in
                                ((-h, -h, 0), (h, -h, 0), (h, h, 0), (-h, h, 0))], np.float32)
            if i not in self._tiles:
                m = marker_image(i, 120)
                self._tiles[i] = cv2.copyMakeBorder(m, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)
            tile = self._tiles[i]
            s = tile.shape[0]
            # the tile has a white border: its marker square spans 20..140 of 160
            src = np.array([[20, 20], [140, 20], [140, 140], [20, 140]], np.float32)
            Hm = cv2.getPerspectiveTransform(src, corners)
            warped = cv2.warpPerspective(tile, Hm, (self.w, self.h), flags=cv2.INTER_LINEAR, borderValue=0)
            mask = cv2.warpPerspective(np.full((s, s), 255, np.uint8), Hm, (self.w, self.h))
            img[mask > 0] = warped[mask > 0]
        if self.noise:
            img = np.clip(img + self.rng.normal(0, self.noise, img.shape), 0, 255).astype(np.uint8)
        return img

    def frames(self, n):
        return [self.render() for _ in range(n)]

    def size(self):
        return self.w, self.h


def hidden_errors(seed=1):
    """A believable mis-calibration: offsets off by several degrees, one servo
    mounted reversed, servos with slightly different pulse scales."""
    rng = np.random.default_rng(seed)
    true = {}
    for j in JOINTS:
        c = copy.deepcopy(default_cal(j))
        c.offset_deg += float(rng.uniform(-9, 9))
        c.range_deg = float(rng.uniform(170, 192))
        true[j] = c
    true['rr_thigh_joint'].direction *= -1
    return true
