"""Joint angles from marker positions.

Markers are glued with their centre ON a joint axis (or on the foot), so only
their positions matter - how a marker is rotated on the part does not.

  side views (left / right): thigh axis, knee axis, foot of the front and the
  rear leg -> thigh and calf angles of both legs
  front / rear views: hip (abduction) axis on the body face and the foot of
  the left and right leg -> hip angles

Frames: body x forward, y left, z up (REP-103); camera x right, y down,
z forward (OpenCV). Angles use the robot's URDF convention (degrees).
"""

import math

import numpy as np

LEGS = ('lf', 'rf', 'lr', 'rr')
SIDE = {'lf': 1, 'rf': -1, 'lr': 1, 'rr': -1}
FRONT = {'lf': 1, 'rf': 1, 'lr': -1, 'rr': -1}

# Marker ids per view. Side views: (thigh axis, knee, foot) per leg.
# Front / rear views: (hip axis on the body face, foot) per leg.
VIEWS = {
    'left': {'kind': 'side', 'legs': {'lf': (0, 1, 2), 'lr': (3, 4, 5)}},
    'right': {'kind': 'side', 'legs': {'rf': (10, 11, 12), 'rr': (13, 14, 15)}},
    'front': {'kind': 'end', 'legs': {'lf': (20, 21), 'rf': (22, 23)}},
    'rear': {'kind': 'end', 'legs': {'lr': (24, 25), 'rr': (26, 27)}},
}
MARKER_NAMES = {
    0: 'LF ось бедра', 1: 'LF колено', 2: 'LF стопа', 3: 'LR ось бедра', 4: 'LR колено', 5: 'LR стопа',
    10: 'RF ось бедра', 11: 'RF колено', 12: 'RF стопа', 13: 'RR ось бедра', 14: 'RR колено', 15: 'RR стопа',
    20: 'LF ось hip (перед)', 21: 'LF стопа (перед)', 22: 'RF ось hip (перед)', 23: 'RF стопа (перед)',
    24: 'LR ось hip (зад)', 25: 'LR стопа (зад)', 26: 'RR ось hip (зад)', 27: 'RR стопа (зад)',
}
CAMERA_Z = np.array([0.0, 0.0, 1.0])


def _unit(v):
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError('markers coincide')
    return v / n


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _leg_plane_normal(v, pos):
    """Common normal of the legs' marker planes (None if degenerate)."""
    rows = []
    for ids in v['legs'].values():
        p = [np.asarray(pos[i], dtype=float) for i in ids if i in pos]
        if len(p) < 3:
            return None
        p = np.array(p)
        rows.append(p - p.mean(axis=0))
    _, sv, vt = np.linalg.svd(np.vstack(rows))
    if sv[1] < 1e-6 or sv[2] / sv[1] > 0.2:  # legs straight or not planar
        return None
    return vt[2]


def body_frame(view, pos):
    """Rows = body x, y, z axes expressed in camera coordinates."""
    v = VIEWS[view]
    legs = list(v['legs'])
    if v['kind'] == 'side':
        front, rear = (l for l in legs if FRONT[l] > 0), (l for l in legs if FRONT[l] < 0)
        f, r = next(front), next(rear)
        x = _unit(pos[v['legs'][f][0]] - pos[v['legs'][r][0]])
        y_ref = -CAMERA_Z if view == 'left' else CAMERA_Z  # body +y towards / away from camera
        # Each leg's markers lie in a plane normal to body y (hips at zero):
        # take y from the markers so a tilted camera does not bias the angles.
        pts = [np.asarray(pos[i]) for ids in v['legs'].values() for i in ids if i in pos]
        n = _leg_plane_normal(v, pos)
        y_dir = n if n is not None and len(pts) >= 6 else y_ref
        if np.dot(y_dir, y_ref) < 0:
            y_dir = -y_dir
        y = _unit(y_dir - np.dot(y_dir, x) * x)
        z = np.cross(x, y)
    else:
        left = next(l for l in legs if SIDE[l] > 0)
        right = next(l for l in legs if SIDE[l] < 0)
        y = _unit(pos[v['legs'][left][0]] - pos[v['legs'][right][0]])
        x_ref = -CAMERA_Z if view == 'front' else CAMERA_Z  # robot faces / turns away
        x = _unit(x_ref - np.dot(x_ref, y) * y)
        z = np.cross(x, y)
    return np.vstack([x, y, z])


def sagittal_angles(R, thigh_axis, knee, foot):
    """Thigh and calf angles [deg] from three marker positions (camera frame)."""
    v = R @ (knee - thigh_axis)
    w = R @ (foot - knee)
    q1 = math.degrees(math.atan2(-v[0], -v[2]))
    a = math.degrees(math.atan2(-w[0], -w[2]))
    return q1, _wrap(a - q1)


def leg_zs(geometry, thigh_deg, calf_deg):
    """Foot height below the thigh axis in the leg plane [m]."""
    L2, L3 = geometry['thigh'], geometry['calf']
    q1, q2 = math.radians(thigh_deg), math.radians(calf_deg)
    return -L2 * math.cos(q1) - L3 * math.cos(q1 + q2)


def hip_angle(R, hip_axis, foot, side, geometry, thigh_deg, calf_deg):
    """Hip abduction angle [deg] from the hip-axis and foot markers.

    Needs the leg's thigh / calf angles (calibrate the side views first) and
    the hip link length, because the foot sits off the hip axis by hip_offset.
    """
    v = R @ (foot - hip_axis)
    alpha = math.atan2(v[2], v[1])
    alpha0 = math.atan2(leg_zs(geometry, thigh_deg, calf_deg), side * geometry['hip_offset'])
    return _wrap(math.degrees(alpha - alpha0))


def measure(view, pos, geometry=None, leg_angles=None):
    """All joint angles visible in `view` from marker positions {id: xyz}.

    Returns {joint_name: degrees}; joints whose markers are missing are left out.
    For end views `leg_angles` gives {leg: (thigh_deg, calf_deg)} used for the
    foot offset (commanded values are fine once the side views are done).
    """
    v = VIEWS[view]
    needed = [ids[0] for ids in v['legs'].values()]
    if any(i not in pos for i in needed):
        return {}
    R = body_frame(view, pos)
    out = {}
    for leg, ids in v['legs'].items():
        if any(i not in pos for i in ids):
            continue
        if v['kind'] == 'side':
            q1, q2 = sagittal_angles(R, *(pos[i] for i in ids))
            out[f'{leg}_thigh_joint'], out[f'{leg}_calf_joint'] = q1, q2
        else:
            th, ca = (leg_angles or {}).get(leg, (0.0, -90.0))
            out[f'{leg}_hip_joint'] = hip_angle(R, pos[ids[0]], pos[ids[1]], SIDE[leg], geometry, th, ca)
    return out


# --------------------------------------------------------------- forward model

def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def leg_points(geometry, leg, hip_deg, thigh_deg, calf_deg):
    """Body-frame points of one leg: hip axis, thigh axis, knee, foot [m]."""
    s, f = SIDE[leg], FRONT[leg]
    L1, L2, L3 = geometry['hip_offset'], geometry['thigh'], geometry['calf']
    hip = np.array([f * geometry['hip_x'], s * geometry['hip_y'], 0.0])
    A = _rx(math.radians(hip_deg))
    thigh = hip + A @ np.array([0.0, s * L1, 0.0])
    B = A @ _ry(math.radians(thigh_deg))
    knee = thigh + B @ np.array([0.0, 0.0, -L2])
    C = B @ _ry(math.radians(calf_deg))
    foot = knee + C @ np.array([0.0, 0.0, -L3])
    return hip, thigh, knee, foot
