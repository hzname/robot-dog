"""The servo profile of servo_model=real: bus-voltage factors, the URDF side of
the profile and the bridge selection, checked against the shipped robot.yaml."""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from dog_description.servo_profile import (BacklashPlay, CommandShaper, DelayLine, ServoProfile,
                                           bridge_settings, parse_override, speed_factor, torque_factor)
from dog_description.urdf import build_urdf, load_config

GEOM = {'hip_offset': 0.055, 'thigh': 0.105, 'calf': 0.105, 'hip_x': 0.09, 'hip_y': 0.06}

# The literal description of the tree before plan 01-10 (same pin as test_urdf.py):
# the tests must not follow DEFAULT_DESCRIPTION, which plan 01-15 may sync.
PINNED_DESC = {
    'body_length': 0.23, 'body_width': 0.10, 'body_height': 0.05,
    'body_mass': 0.80, 'hip_mass': 0.06, 'thigh_mass': 0.08, 'calf_mass': 0.03,
    'foot_radius': 0.012, 'servo_effort': 1.1, 'servo_velocity': 6.0, 'sim_p_gain': 25.0,
    'hip_limits_deg': [-40.0, 40.0], 'thigh_limits_deg': [-45.0, 135.0],
    'calf_limits_deg': [-165.0, -15.0],
}


def _robot_yaml():
    return os.path.join(os.path.dirname(__file__), '..', '..', 'dog_bringup', 'config', 'robot.yaml')


def test_speed_and_torque_factors():
    for v, factor in ((4.8, 0.82348), (5.2, 0.88232), (6.0, 1.0), (6.6, 1.08826)):
        assert speed_factor(v) == pytest.approx(factor, abs=1e-5)
    for v, factor in ((4.8, 0.85456), (5.2, 0.90304), (6.0, 1.0), (6.6, 1.07272)):
        assert torque_factor(v) == pytest.approx(factor, abs=1e-5)
    assert speed_factor(5.7, 5.7) == 1.0
    assert torque_factor(5.7, 5.7) == 1.0


def test_voltage_clamped():
    assert speed_factor(3.0) == speed_factor(4.8)
    assert speed_factor(9.0) == speed_factor(6.6)
    assert torque_factor(3.0) == torque_factor(4.8)
    assert torque_factor(9.0) == torque_factor(6.6)
    for bad in (float('nan'), float('inf'), True, '6.0'):
        with pytest.raises(ValueError, match='"v"'):
            speed_factor(bad)
        with pytest.raises(ValueError, match='"v"'):
            torque_factor(bad)
    with pytest.raises(ValueError, match='v_ref'):
        speed_factor(6.0, 3.0)
    with pytest.raises(ValueError, match='v_ref'):
        torque_factor(6.0, 7.0)


def test_profile_from_params():
    default = ServoProfile()
    assert ServoProfile.from_params(None) == default
    assert ServoProfile.from_params({}) == default
    p = ServoProfile.from_params({'backlash_deg': 2.0, 'delay_ms': 30.0, 'friction_nm': 0.1,
                                  'bus_voltage': 5.2, 'bus_voltage_ref': 6.0, 'extra': 1})
    assert (p.backlash_deg, p.delay_ms, p.friction_nm) == (2.0, 30.0, 0.1)
    assert (p.bus_voltage, p.bus_voltage_ref) == (5.2, 6.0)
    for key, value in (('backlash_deg', -1.0), ('delay_ms', float('nan')),
                       ('friction_nm', True), ('bus_voltage', 0.0), ('bus_voltage_ref', -6.0)):
        with pytest.raises(ValueError, match=key):
            ServoProfile.from_params({key: value})


def test_profile_defaults_match_robot_yaml():
    _, description = load_config(_robot_yaml())
    block = description['servo_sim']
    default = ServoProfile()
    assert block['backlash_deg'] == default.backlash_deg
    assert block['delay_ms'] == default.delay_ms
    assert block['friction_nm'] == default.friction_nm
    assert block['bus_voltage'] == default.bus_voltage
    assert block['bus_voltage_ref'] == default.bus_voltage_ref


def test_real_urdf_has_12_friction_joints():
    """The shipped robot.yaml with servo_model='real': every revolute joint gets
    dynamics damping 0 / friction friction_nm, and its limits are the profile formulas."""
    geometry, description = load_config(_robot_yaml())
    p = ServoProfile.from_params(description.get('servo_sim'))
    sf = speed_factor(p.bus_voltage, p.bus_voltage_ref)
    tf = torque_factor(p.bus_voltage, p.bus_voltage_ref)
    eff = round(description['servo_effort'] * tf, 4)
    vel = round(description['servo_velocity'] * sf, 4)
    for gazebo in (False, True):
        root = ET.fromstring(build_urdf(geometry, description, gazebo=gazebo, servo_model='real'))
        joints = root.findall("joint[@type='revolute']")
        assert len(joints) == 12
        for joint in joints:
            dyn = joint.find('dynamics')
            assert dyn is not None
            assert float(dyn.get('damping')) == 0.0
            assert float(dyn.get('friction')) == pytest.approx(description['servo_sim']['friction_nm'])
            limit = joint.find('limit')
            assert float(limit.get('effort')) == pytest.approx(eff)
            assert float(limit.get('velocity')) == pytest.approx(vel)
        if gazebo:
            caps = [(p_.find('joint_name').text, float(p_.find('cmd_max').text),
                     float(p_.find('cmd_min').text))
                    for p_ in root.findall('gazebo/plugin') if p_.find('joint_name') is not None]
            assert len(caps) == 12
            for _, cmd_max, cmd_min in caps:
                assert cmd_max == pytest.approx(vel)
                assert cmd_min == pytest.approx(-vel)


def test_real_urdf_scales_by_voltage_and_knee_ratio():
    """Every number comes from the formulas on the same description; the knee rod
    drive divides its speed and multiplies its torque by knee_ratio (D-20)."""
    desc = dict(PINNED_DESC, servo_sim={'bus_voltage': 5.2, 'bus_voltage_ref': 6.0},
                servo={'knee_ratio': 1.5})
    plain = ET.fromstring(build_urdf(GEOM, desc, servo_model='real'))
    gazebo = ET.fromstring(build_urdf(GEOM, desc, gazebo=True, servo_model='real'))
    joints = {j.get('name'): j for j in plain.findall("joint[@type='revolute']")}
    for leg in ('lf', 'rf', 'lr', 'rr'):
        for joint, velocity, effort in (('hip', 5.2939, 0.9933), ('thigh', 5.2939, 0.9933),
                                        ('calf', 3.5293, 1.49)):
            limit = joints[f'{leg}_{joint}_joint'].find('limit')
            assert float(limit.get('velocity')) == pytest.approx(velocity, abs=1e-4)
            assert float(limit.get('effort')) == pytest.approx(effort, abs=1e-4)
    caps = {p_.find('joint_name').text: (p_.find('cmd_max').text, p_.find('cmd_min').text)
            for p_ in gazebo.findall('gazebo/plugin') if p_.find('joint_name') is not None}
    assert caps['lf_hip_joint'] == ('5.2939', '-5.2939')
    assert caps['lf_calf_joint'] == ('3.5293', '-3.5293')


def test_backlash_dead_zone_and_reversal():
    play = BacklashPlay(0.02)
    commands = [0.50, 0.505, 0.52, 0.515, 0.49, 0.495, 0.505]
    assert [play(x) for x in commands] == pytest.approx([0.50, 0.50, 0.51, 0.51, 0.50, 0.50, 0.50])
    half = 0.01
    # a random walk within +-0.02: the output never leaves the dead zone, holds
    # still while the command is inside it and moves with the command upwards
    play = BacklashPlay(0.02)
    rng = np.random.default_rng(0)
    prev = None
    for x in rng.uniform(-0.02, 0.02, 1000):
        y = play(x)
        if prev is not None and abs(x - prev) <= half:
            assert y == prev
        assert abs(x - y) <= half + 1e-12
        prev = y
    ys = [play(x) for x in np.linspace(-0.05, 0.05, 200)]
    assert all(b >= a for a, b in zip(ys, ys[1:]))
    # zero width is exact passthrough
    exact = BacklashPlay(0.0)
    for x in (0.1, 0.2, 0.1, -0.3, 0.1):
        assert exact(x) == x
    # a non-finite command comes back as it is and leaves the state intact
    a, b = BacklashPlay(0.02), BacklashPlay(0.02)
    seq = [0.50, 0.52, 0.49, 0.505]
    assert [a(x) for x in seq[:2]] == [b(x) for x in seq[:2]]
    assert math.isnan(b(float('nan')))
    assert b(float('inf')) == float('inf')
    assert [a(x) for x in seq[2:]] == [b(x) for x in seq[2:]]


def test_delay_line_orders_by_sim_time():
    line = DelayLine(0.04)
    line.push(1.00, 'a')
    line.push(1.01, 'b')
    line.push(1.03, 'c')
    assert line.pop_ready(1.039) == []
    assert line.pop_ready(1.04) == ['a']
    assert line.pop_ready(1.05) == ['b']
    assert line.pop_ready(2.0) == ['c']
    assert line.pop_ready(2.0) == []
    assert len(line) == 0
    # same-instant entries keep their push order; a clock going back loses nothing
    line = DelayLine(0.04)
    for item in ('x', 'y', 'z'):
        line.push(2.0, item)
    assert line.pop_ready(1.0) == []
    assert len(line) == 3
    assert line.pop_ready(2.04) == ['x', 'y', 'z']
    # zero delay releases at once; negative or NaN delays are rejected
    zero = DelayLine(0.0)
    zero.push(5.0, 'v')
    assert zero.pop_ready(5.0) == ['v']
    for bad in (-0.01, float('nan')):
        with pytest.raises(ValueError):
            DelayLine(bad)


def test_shaper_zero_is_passthrough():
    shaper = CommandShaper(0.0, 0.0)
    rng = np.random.default_rng(0)
    names = ['lf_hip_joint', 'rf_thigh_joint', 'lr_calf_joint']
    for i in range(100):
        t = 0.002 * i
        positions = list(rng.uniform(-1.0, 1.0, len(names)))
        shaper.push(t, names, positions)
        assert shaper.pop_ready(t) == list(zip(names, positions))
    assert len(shaper.delay) == 0


def test_shaper_applies_backlash_then_delay():
    shaper = CommandShaper(0.02, 0.04)
    shaper.push(1.0, ['a', 'b'], [0.50, 0.50])
    assert shaper.pop_ready(1.039) == []
    assert shaper.pop_ready(1.04) == [('a', 0.50), ('b', 0.50)]
    shaper.push(1.05, ['a', 'b'], [0.52, 0.50])
    assert shaper.pop_ready(1.089) == []
    moved, held = shaper.pop_ready(1.09)
    assert moved[0] == 'a' and moved[1] == pytest.approx(0.51)
    assert held == ('b', 0.50)  # b has its own play: the command sits inside its dead zone


def test_bridge_settings():
    assert bridge_settings('ideal', {'backlash_deg': 2.0, 'delay_ms': 30.0}, 25.0) == (0.0, 0.0)
    assert bridge_settings('ideal', {}, None) == (0.0, 0.0)
    assert bridge_settings('real', {}, None) == (1.5, 0.04)
    assert bridge_settings('real', {'backlash_deg': 2.0, 'delay_ms': 30.0}, None) == (2.0, 0.03)
    assert bridge_settings('real', {}, 25.0) == (1.5, 0.025)
    with pytest.raises(ValueError, match='servo_model'):
        bridge_settings('servo', {}, None)


def test_parse_override():
    assert parse_override('servo_speed', None) is None
    assert parse_override('servo_speed', '') is None
    assert parse_override('servo_speed', '   ') is None
    assert parse_override('servo_speed', '5.5') == 5.5
    for text in ('abc', 'nan', 'inf', '-1'):
        with pytest.raises(ValueError, match='servo_speed'):
            parse_override('servo_speed', text)
    assert parse_override('servo_delay_ms', '0') == 0.0
    with pytest.raises(ValueError, match='servo_delay_ms'):
        parse_override('servo_delay_ms', '0', positive=True)
