"""The generated URDF must agree with the kinematics used by locomotion."""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np

from dog_description.urdf import build_urdf, joint_names, load_config

GEOM = {'hip_offset': 0.055, 'thigh': 0.105, 'calf': 0.105, 'hip_x': 0.09, 'hip_y': 0.06}


def _rot(axis, q):
    c, s = math.cos(q), math.sin(q)
    if axis == (1, 0, 0):
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == (0, 1, 0):
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    raise ValueError(axis)


def _fk(root, leg, q):
    """Foot position in trunk frame by walking the URDF joint chain."""
    joints = {j.get('name'): j for j in root.findall('joint')}
    T_R, T_p = np.eye(3), np.zeros(3)
    for jn, angle in zip(('hip', 'thigh', 'calf', 'foot'), list(q) + [0.0]):
        j = joints[f'{leg}_{jn}_joint']
        xyz = np.array([float(v) for v in j.find('origin').get('xyz').split()])
        T_p = T_p + T_R @ xyz
        if j.get('type') == 'revolute':
            axis = tuple(int(float(v)) for v in j.find('axis').get('xyz').split())
            T_R = T_R @ _rot(axis, angle)
    return T_p


def _analytic(leg_side, q):
    """Same formulas as dog_control/kinematics.cpp forwardKinematics()."""
    L1, L2, L3 = GEOM['hip_offset'], GEOM['thigh'], GEOM['calf']
    xs = -L2 * math.sin(q[1]) - L3 * math.sin(q[1] + q[2])
    zs = -L2 * math.cos(q[1]) - L3 * math.cos(q[1] + q[2])
    y0 = leg_side * L1
    c, s = math.cos(q[0]), math.sin(q[0])
    return np.array([xs, y0 * c - zs * s, y0 * s + zs * c])


def test_urdf_matches_controller_kinematics():
    root = ET.fromstring(build_urdf(GEOM))
    rng = np.random.default_rng(0)
    for leg, front, side in (('lf', 1, 1), ('rf', 1, -1), ('lr', -1, 1), ('rr', -1, -1)):
        hip = np.array([front * GEOM['hip_x'], side * GEOM['hip_y'], 0.0])
        for _ in range(50):
            q = rng.uniform([-0.6, -0.8, -2.5], [0.6, 2.0, -0.2])
            np.testing.assert_allclose(_fk(root, leg, q), hip + _analytic(side, q), atol=1e-9)


def test_joint_names_and_limits():
    root = ET.fromstring(build_urdf(GEOM))
    revolute = [j.get('name') for j in root.findall('joint') if j.get('type') == 'revolute']
    assert revolute == joint_names()
    assert len(revolute) == 12
    for j in root.findall('joint'):
        lim = j.find('limit')
        if lim is not None:
            assert float(lim.get('lower')) < float(lim.get('upper'))


def test_gazebo_extras_and_shipped_config():
    here = os.path.dirname(__file__)
    cfg = os.path.join(here, '..', '..', 'dog_bringup', 'config', 'robot.yaml')
    geometry, description = load_config(cfg)
    urdf = build_urdf(geometry, description, gazebo=True)
    root = ET.fromstring(urdf)
    plugins = root.findall('gazebo/plugin')
    assert sum('JointPositionController' in p.get('name') for p in plugins) == 12
    masses = [float(m.get('value')) for m in root.iter('mass')]
    assert 1.0 < sum(masses) < 2.5  # MG996R dog: ~1.5 kg
