"""Contract of the Phase 1 config keys in robot.yaml and servos.yaml."""

import ast
import math
import os
import re

import pytest
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'ros2_ws', 'src', 'dog_bringup', 'config')

NUMBERS = ['gait.min_period', 'servo.max_speed', 'servo.margin', 'servo.knee_ratio',
           'servo_sim.backlash_deg', 'servo_sim.delay_ms', 'servo_sim.friction_nm',
           'servo_sim.bus_voltage', 'servo_sim.bus_voltage_ref', 'description.body_com_x']


def _robot():
    with open(os.path.join(CONFIG, 'robot.yaml')) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def _servos():
    with open(os.path.join(CONFIG, 'servos.yaml')) as f:
        return yaml.safe_load(f)['/**/servo_driver']['ros__parameters']


def _dig(root, path):
    for key in path.split('.'):
        root = root[key]
    return root


def _block_lines(name):
    """Lines of the `name:` block of robot.yaml, keys and comments, no header."""
    with open(os.path.join(CONFIG, 'robot.yaml')) as f:
        lines = f.readlines()
    header = re.compile(r'^\s+%s:\s*(#.*)?$' % re.escape(name))
    i = next((k for k, ln in enumerate(lines) if header.match(ln.rstrip('\n'))), None)
    assert i is not None, 'block %r not found in robot.yaml' % name
    base = len(lines[i]) - len(lines[i].lstrip())
    out = []
    for ln in lines[i + 1:]:
        if ln.strip() and not ln.lstrip().startswith('#') and len(ln) - len(ln.lstrip()) <= base:
            break
        out.append(ln.rstrip('\n'))
    return out


@pytest.mark.parametrize('path', ['gait.auto_period'] + NUMBERS)
def test_contract_keys_exist_with_the_right_types(path):
    value = _dig(_robot(), path)
    if path == 'gait.auto_period':
        assert isinstance(value, bool), path
        return
    assert isinstance(value, (int, float)) and not isinstance(value, bool), path
    assert math.isfinite(value), path


def test_three_servo_speeds_are_one_number():
    velocity = _dig(_robot(), 'description.servo_velocity')
    max_speed = _dig(_robot(), 'servo.max_speed')
    joint_speed = _servos()['max_joint_speed']
    assert velocity == max_speed == joint_speed
    assert velocity > 0


def test_margin_is_a_fraction_and_knee_ratio_is_at_least_one():
    servo = _robot()['servo']
    assert 0 < servo['margin'] <= 1
    assert servo['knee_ratio'] >= 1.0


def test_min_period_is_in_seconds():
    value = _robot()['gait']['min_period']
    assert 0 < value <= 1.5


def test_servo_sim_numbers_are_physical():
    sim = _robot()['servo_sim']
    assert sim['backlash_deg'] >= 0 and sim['delay_ms'] >= 0 and sim['friction_nm'] >= 0
    assert sim['bus_voltage'] > 0 and sim['bus_voltage_ref'] > 0


def test_servo_blocks_carry_a_comment_with_a_unit():
    for name in ('servo', 'servo_sim'):
        for ln in _block_lines(name):
            if not ln.strip() or ln.lstrip().startswith('#'):
                continue
            assert re.match(r'^\s+\w+:', ln), (name, ln)
            assert '#' in ln, (name, ln)
            comment = ln.split('#', 1)[1].strip()
            assert len(comment) >= 10, (name, ln)
            assert re.search(r'\[[^\]]+\]', comment), (name, ln)


# ---------------------------------------------------------------- coupled knee
# Numbers of the thigh x knee collision map: CAD (Fusion 360, Full_body_case, stand-in
# servo SPT5435LV), 2026-10-13, docs/CALIBRATION.md, section "Колено связано с бедром".
# The knee servo sets the absolute shin angle s = thigh + calf, so the free region is a
# rectangle in (q2, s). Not measured on the robot.
CAD_Q2_REGION = (-145.0, -35.0)         # [deg] knee against the thigh, by the map
CAD_S_HARD_STOPS = (-87.0, 18.0)        # [deg] hard stops of s from the fine sweeps
GAIT_S_ENVELOPE = (-68.6, -16.2)        # [deg] stance/walk/lie, `stance_envelope` of the map
# [deg] thigh 45 + knee -90, all servos centred. An assembly assumption, not a measurement: after
# s0 is measured at 1370 us (docs/CALIBRATION.md) put s0 here, the test then holds for both recipes.
MOUNT_S_DEG = -45.0
CENTRE_PULSE_US = 1370.0                # [us] v1 centre pulse, the mount pose
LEGS = ('lf', 'rf', 'lr', 'rr')
# [deg] knee trims at the mount pose, measured on the bench with tools/servo_pose (2026-10-26):
# the mount pose of a knee is held at 1370 + direction * trim * 1700/180 us, so its pulse window
# is shifted by that much (the s window of the CAD map stays the same).
KNEE_MOUNT_TRIM_DEG = {'lf': -12.0, 'rf': -11.0, 'lr': -11.0, 'rr': -30.0}


def _calf(leg):
    return _servos()['%s_calf_joint' % leg]


@pytest.mark.parametrize('leg', LEGS)
def test_calf_is_coupled_one_to_one_to_the_thigh_of_its_leg(leg):
    calf, thigh = _calf(leg), _servos()['%s_thigh_joint' % leg]
    assert calf['coupled_to'] == '%s_thigh_joint' % leg
    assert calf['coupling'] == 1.0
    assert calf['servo_arm_mm'] == 0.0
    assert calf['direction'] == (1 if leg[0] == 'l' else -1)
    assert calf['direction'] == thigh['direction']


@pytest.mark.parametrize('leg', LEGS)
def test_calf_limits_lie_in_the_cad_region_and_equal_robot_yaml(leg):
    lo, hi = _robot()['description']['calf_limits_deg']
    calf = _calf(leg)
    assert CAD_Q2_REGION[0] <= calf['min_deg'] < calf['max_deg'] <= CAD_Q2_REGION[1]
    assert (calf['min_deg'], calf['max_deg']) == (lo, hi)


@pytest.mark.parametrize('leg', LEGS)
def test_calf_window_stays_inside_the_cad_stops_and_covers_the_gait(leg):
    c = _calf(leg)
    low, high = c['offset_deg'] - c['range_deg'] / 2, c['offset_deg'] + c['range_deg'] / 2
    assert CAD_S_HARD_STOPS[0] <= low and high <= CAD_S_HARD_STOPS[1], (low, high)
    assert low <= GAIT_S_ENVELOPE[0] and GAIT_S_ENVELOPE[1] <= high, (low, high)
    us_per_deg = (c['pulse_max_us'] - c['pulse_min_us']) / c['range_deg']
    assert abs(us_per_deg / (1700.0 / 180.0) - 1.0) < 0.01, us_per_deg
    assert 300.0 <= c['pulse_min_us'] < c['pulse_max_us'] <= 2800.0


@pytest.mark.parametrize('leg', LEGS)
def test_the_mount_pose_is_the_centre_pulse_of_the_knee(leg):
    """Physical consistency: the mount pose (thigh 45, knee -90, s = -45) is the centre pulse
    1370 us plus the knee's measured mount trim."""
    c = _calf(leg)
    centre = (c['pulse_min_us'] + c['pulse_max_us']) / 2
    us_per_deg = (c['pulse_max_us'] - c['pulse_min_us']) / c['range_deg']
    pulse = centre + c['direction'] * (MOUNT_S_DEG - c['offset_deg']) * us_per_deg
    expected = CENTRE_PULSE_US + c['direction'] * KNEE_MOUNT_TRIM_DEG[leg] * 1700.0 / 180.0
    assert abs(pulse - expected) <= 1.0, (pulse, expected, 'knee window does not match the measured mount trim')


# Rule of the project (outline #7, note 01-10): the defaults of the code are the
# YAML values. These two guards fail when a YAML number changes without the C++
# default, so a later phase cannot silently leave the nodes on the old numbers.
ROS = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'ros2_ws', 'src')


def _read(*parts):
    with open(os.path.join(ROS, *parts)) as f:
        return f.read()


def _cpp_struct(text, name):
    """Body of `struct <name> { ... }`, line comments removed."""
    text = re.sub(r'//.*', '', text)
    start = re.search(r'\bstruct\s+%s\s*\{' % re.escape(name), text)
    assert start, 'struct %s not found' % name
    body = text[start.end() - 1:]
    depth = 0
    for i, ch in enumerate(body):
        if ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0:
                return body[1:i]
    raise AssertionError('unbalanced struct %s' % name)


def _cpp_field(body, name):
    """Default of `double|float|int|bool <name>{...}` inside a struct body."""
    m = re.search(r'\b(?:double|float|int|bool)\s+%s\s*\{\s*([^}]+?)\s*\}' % re.escape(name), body)
    assert m, 'default of %s not found' % name
    raw = m.group(1)
    if raw == 'true':
        return True
    if raw == 'false':
        return False
    return float(raw)


def _default_description():
    tree = ast.parse(_read('dog_description', 'dog_description', 'urdf.py'))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', None) == 'DEFAULT_DESCRIPTION'
                                                for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError('DEFAULT_DESCRIPTION not found in urdf.py')


def test_cpp_defaults_match_geometry_in_robot_yaml():
    robot = _robot()
    leg = _cpp_struct(_read('dog_control', 'include', 'dog_control', 'kinematics.hpp'),
                      'LegGeometry')
    for field, path in (('hip', 'geometry.hip_offset'), ('thigh', 'geometry.thigh'),
                        ('calf', 'geometry.calf')):
        assert _cpp_field(leg, field) == pytest.approx(_dig(robot, path)), (field, path)
    params = _cpp_struct(_read('dog_control', 'include', 'dog_control', 'locomotion.hpp'),
                         'LocomotionParams')
    for field, path in (('hip_x', 'geometry.hip_x'), ('hip_y', 'geometry.hip_y'),
                        ('stand_height', 'stance.stand_height'), ('lie_height', 'stance.lie_height'),
                        ('min_height', 'stance.min_height'), ('max_height', 'stance.max_height'),
                        ('foot_offset_x', 'stance.foot_offset_x'),
                        ('foot_offset_y', 'stance.foot_offset_y')):
        assert _cpp_field(params, field) == pytest.approx(_dig(robot, path)), (field, path)
    default = _default_description()
    assert set(default) == set(robot['description'])
    for key, value in default.items():
        assert robot['description'][key] == value, key


def test_cpp_defaults_match_servo_model_and_auto_period():
    robot = _robot()
    model = _cpp_struct(_read('dog_control', 'include', 'dog_control', 'servo_limits.hpp'),
                        'ServoSpeedModel')
    for field in ('max_speed', 'margin', 'knee_ratio'):
        assert _cpp_field(model, field) == pytest.approx(robot['servo'][field]), field
    params = _cpp_struct(_read('dog_control', 'include', 'dog_control', 'locomotion.hpp'),
                         'LocomotionParams')
    assert _cpp_field(params, 'auto_period') is robot['gait']['auto_period']
