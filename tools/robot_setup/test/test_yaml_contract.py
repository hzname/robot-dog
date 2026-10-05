"""Contract of the Phase 1 config keys in robot.yaml and servos.yaml."""

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
