"""The generated URDF must agree with the kinematics used by locomotion."""

import hashlib
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from dog_description.urdf import build_urdf, joint_names, load_config

GEOM = {'hip_offset': 0.055, 'thigh': 0.105, 'calf': 0.105, 'hip_x': 0.09, 'hip_y': 0.06}

# The description the golden hashes were taken from: the literal DEFAULT_DESCRIPTION
# of the tree before servo_model was added, without body_com_x (plan 01-10). The
# guard must not follow DEFAULT_DESCRIPTION, which plan 01-15 may sync to a
# measurement; it pins the ideal model of the shipped defaults (D-15).
PINNED_DESC = {
    'body_length': 0.23, 'body_width': 0.10, 'body_height': 0.05,
    'body_mass': 0.80, 'hip_mass': 0.06, 'thigh_mass': 0.08, 'calf_mass': 0.03,
    'foot_radius': 0.012, 'servo_effort': 1.1, 'servo_velocity': 6.0, 'sim_p_gain': 25.0,
    'hip_limits_deg': [-40.0, 40.0], 'thigh_limits_deg': [-45.0, 135.0],
    'calf_limits_deg': [-165.0, -15.0],
}

# SHA-256 of _ideal_cases() taken on the untouched urdf.py before plan 01-10:
# the ideal model output must stay byte for byte the same (D-15).
GOLDEN_SHA256 = {
    'plain': 'b73643bda3555b22c94396ef7ae9eea6a4ee09d7836e83220643f8a5ad57bd36',
    'gazebo': '54c0eef47c67ca32aa714e4ade91260051e5553dc25bbc54a525e790423faaf9',
}


def _ideal_cases():
    """The two pinned ideal-model URDF strings: plain and the Gazebo variant."""
    return {
        'plain': build_urdf(GEOM, PINNED_DESC),
        'gazebo': build_urdf(GEOM, PINNED_DESC, gazebo=True, initial=(0.0, 0.7, -1.4)),
    }


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


def _analytic(leg_side, q, calf=None):
    """Same formulas as dog_control/kinematics.cpp forwardKinematics()."""
    L1, L2, L3 = GEOM['hip_offset'], GEOM['thigh'], GEOM['calf'] if calf is None else calf
    xs = -L2 * math.sin(q[1]) - L3 * math.sin(q[1] + q[2])
    zs = -L2 * math.cos(q[1]) - L3 * math.cos(q[1] + q[2])
    y0 = leg_side * L1
    c, s = math.cos(q[0]), math.sin(q[0])
    return np.array([xs, y0 * c - zs * s, y0 * s + zs * c])


def test_urdf_matches_controller_kinematics():
    """calf runs to the foot CONTACT point: the foot sphere (radius r) sits on
    the calf axis with its centre r short of the calf end, its bottom at the end."""
    root = ET.fromstring(build_urdf(GEOM))
    r = 0.012  # DEFAULT_DESCRIPTION foot_radius
    rng = np.random.default_rng(0)
    for leg, front, side in (('lf', 1, 1), ('rf', 1, -1), ('lr', -1, 1), ('rr', -1, -1)):
        hip = np.array([front * GEOM['hip_x'], side * GEOM['hip_y'], 0.0])
        for _ in range(50):
            q = rng.uniform([-0.6, -0.8, -2.5], [0.6, 2.0, -0.2])
            np.testing.assert_allclose(_fk(root, leg, q), hip + _analytic(side, q, GEOM['calf'] - r), atol=1e-9)
    # standing straight on the floor: sphere bottom = calf end = what dog_control puts on the ground
    np.testing.assert_allclose(_fk(root, 'lf', (0, 0, 0))[2] - r, -(GEOM['thigh'] + GEOM['calf']), atol=1e-9)


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


def test_perception_sensors_in_urdf():
    """Sensor frames match the angles in robot.yaml; Gazebo gets gpu_lidar sensors."""
    from dog_description.urdf import sensor_frames
    cfg = os.path.join(os.path.dirname(__file__), '..', '..', 'dog_bringup', 'config', 'robot.yaml')
    geometry, description = load_config(cfg)
    s = description['sensors']
    frames = {f[0]: f for f in sensor_frames(s)}
    assert {'lidar_left', 'lidar_right', 'tof_fl', 'tof_fr', 'tof_fc', 'tof_rc', 'gs2'} <= set(frames)
    assert abs(frames['gs2'][3][1] - math.radians(40)) < 1e-9
    # crossed: the left lidar dips towards the right and vice versa
    assert frames['lidar_left'][3][2] < 0 < frames['lidar_right'][3][2]
    assert abs(frames['tof_fl'][3][1] - math.radians(40)) < 1e-9
    root = ET.fromstring(build_urdf(geometry, description, gazebo=True))
    assert len(root.findall(".//sensor[@type='gpu_lidar']")) == 7
    plain = ET.fromstring(build_urdf(geometry, description))
    assert not plain.findall('.//sensor') and plain.find("link[@name='lidar_left']") is not None


def test_ideal_urdf_unchanged():
    """D-15: the default ideal model keeps the pinned bytes; the servo_sim and
    servo keys and an explicit servo_model='ideal' change nothing, <dynamics> never appears."""
    for name, urdf in _ideal_cases().items():
        assert hashlib.sha256(urdf.encode()).hexdigest() == GOLDEN_SHA256[name]
    extra = dict(PINNED_DESC, servo_sim={'backlash_deg': 1.5}, servo={'knee_ratio': 1.0})
    assert build_urdf(GEOM, extra, servo_model='ideal') == _ideal_cases()['plain']
    assert (build_urdf(GEOM, extra, gazebo=True, initial=(0.0, 0.7, -1.4))
            == _ideal_cases()['gazebo'])
    assert '<dynamics' not in _ideal_cases()['plain']


def test_unknown_servo_model_is_rejected():
    with pytest.raises(ValueError, match='servo_model'):
        build_urdf(GEOM, PINNED_DESC, servo_model='servo')


def test_cli_servo_model_flag(monkeypatch, capsys):
    """main() --servo-model real emits the 12 friction joints; the default emits none."""
    from dog_description import urdf as urdf_module
    cfg = os.path.join(os.path.dirname(__file__), '..', '..', 'dog_bringup', 'config', 'robot.yaml')
    monkeypatch.setattr(sys, 'argv', ['generate_urdf', cfg, '--gazebo', '--servo-model', 'real'])
    urdf_module.main()
    assert capsys.readouterr().out.count('<dynamics') == 12
    monkeypatch.setattr(sys, 'argv', ['generate_urdf', cfg, '--gazebo'])
    urdf_module.main()
    assert '<dynamics' not in capsys.readouterr().out


def test_load_config_carries_servo_blocks():
    """load_config() passes servo_sim and servo through to the description."""
    cfg = os.path.join(os.path.dirname(__file__), '..', '..', 'dog_bringup', 'config', 'robot.yaml')
    _, description = load_config(cfg)
    assert set(description['servo_sim']) >= {'backlash_deg', 'delay_ms', 'friction_nm',
                                             'bus_voltage', 'bus_voltage_ref'}
    assert set(description['servo']) >= {'max_speed', 'margin', 'knee_ratio'}


def test_body_com_x_moves_trunk_inertial():
    """D-21/CAL-16: description.body_com_x shifts the trunk inertial origin along
    x; a missing key and an explicit 0.0 keep the previous bytes."""
    zero = build_urdf(GEOM, PINNED_DESC)
    assert build_urdf(GEOM, {**PINNED_DESC, 'body_com_x': 0.0}) == zero
    plus = build_urdf(GEOM, {**PINNED_DESC, 'body_com_x': 0.012})
    trunk = next(l for l in ET.fromstring(plus).findall('link') if l.get('name') == 'trunk')
    assert trunk.find('inertial/origin').get('xyz') == '0.0120 0.0000 0.0000'
    assert trunk.find('visual/origin') is None and trunk.find('collision/origin') is None
    # swapping the shifted fragment for the zero one reproduces the plain string
    assert plus.replace('0.0120 0.0000 0.0000', '0.0000 0.0000 0.0000') == zero
    minus = build_urdf(GEOM, {**PINNED_DESC, 'body_com_x': -0.02})
    trunk = next(l for l in ET.fromstring(minus).findall('link') if l.get('name') == 'trunk')
    assert trunk.find('inertial/origin').get('xyz') == '-0.0200 0.0000 0.0000'
    for bad in (float('nan'), float('inf'), True):
        with pytest.raises(ValueError, match='body_com_x'):
            build_urdf(GEOM, {**PINNED_DESC, 'body_com_x': bad})
        with pytest.raises(ValueError, match='body_com_x'):
            build_urdf(GEOM, {**PINNED_DESC, 'body_com_x': bad}, servo_model='real')
