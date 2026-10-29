"""Preflight must refuse to start the robot when the hardware is not there.

These tests cannot make a chip ACK, so they pin the two things that decide the
outcome: which devices are looked for, and what happens when one is missing. A full
pass is proven on the robot and inside the arm64 image, where the job asserts that
the same script *refuses*.
"""

import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import preflight  # noqa: E402


def _fake_root(tmp_path, bus=True, js=True):
    root = tmp_path
    (root / 'dev' / 'input').mkdir(parents=True)
    if bus:
        (root / 'dev' / 'i2c-0').write_bytes(b'')     # a file, not an i2c adapter
    if js:
        (root / 'dev' / 'input' / 'js0').write_bytes(b'')
    return str(root)


def test_no_bus_blocks_the_launch(tmp_path):
    code = preflight.main(['--root', str(tmp_path), '--args', 'backend:=pca9685'])
    assert code == 2


def test_a_regular_file_is_not_an_i2c_bus(tmp_path):
    root = _fake_root(tmp_path)
    items = preflight.checks(root, '/dev/i2c-0', preflight.ADDRESSES, gamepad=True)
    texts = ' | '.join(t for _, t, _ in items)
    assert any(ok and t.startswith('/dev/i2c-0') for ok, t, _ in items), texts
    # ioctl on a plain file fails: every chip must come back as not answering
    assert not [t for ok, t, req in items if req and ok and t.startswith('0x')], texts
    assert preflight.main(['--root', root, '--args', 'backend:=pca9685 gamepad:=false']) == 2


def test_the_mock_backend_needs_no_hardware(tmp_path):
    assert preflight.main(['--root', str(tmp_path), '--args', 'backend:=mock']) == 0
    assert preflight.main(['--root', str(tmp_path), '--args', '', '--mock']) == 0


def test_the_required_set_is_the_three_chips_the_robot_walks_on():
    required = [a for a, _, req in preflight.ADDRESSES if req]
    assert (0x40, 0x68, 0x41) == tuple(required)
    assert 0x29 not in required, 'the ToF sensor is optional for walking'


def test_a_failed_optional_chip_does_not_block():
    items = [(True, '/dev/i2c-0: есть', True),
             (False, '0x29 VL53L1X (ToF): не отвечает', False),
             (True, '0x40 PCA9685: отвечает', True)]
    assert preflight.blocked(items) == []


def test_a_failed_required_chip_blocks():
    items = [(True, '/dev/i2c-0: есть', True),
             (False, '0x68 MPU6050 (IMU): не отвечает', True)]
    assert preflight.blocked(items) == ['0x68 MPU6050 (IMU): не отвечает']


def test_gamepad_discovery(tmp_path):
    root = _fake_root(tmp_path, bus=False, js=True)
    assert preflight.gamepad_devices(root) == ['js0']
    (tmp_path / 'dev' / 'input' / 'js0').unlink()
    assert preflight.gamepad_devices(root) == []


def test_gamepad_false_skips_the_joystick_check(tmp_path):
    root = _fake_root(tmp_path, bus=False, js=False)
    items = preflight.checks(root, '/dev/i2c-0', preflight.ADDRESSES, gamepad=False)
    assert not [t for _, t, _ in items if 'js0' in t]


@pytest.mark.parametrize('args_,expected', [
    ('backend:=pca9685', 'true'),
    ('backend:=pca9685 gamepad:=true', 'true'),
    ('backend:=pca9685 gamepad:=false', 'false'),
])
def test_gamepad_defaults_follow_the_launch_arguments(args_, expected):
    # the same rule main() uses, without touching the bus
    gamepad = 'false' if 'gamepad:=false' in args_ else 'true'
    assert gamepad == expected


# CAL-01/CAL-03 at the door: the profile that walks in the open asks for a calibration of
# this robot first. Everything below runs without a robot, which is the point.

def fake_run(code, text=''):
    calls = []

    def run(config_dir):
        calls.append(config_dir)
        return code, text
    run.calls = calls
    return run


def test_the_floor_profile_without_a_local_calibration_is_blocked(tmp_path):
    items = preflight.calibration_gate(str(tmp_path), 'floor')
    assert items and not items[0][0]
    assert 'servos.local.yaml' in items[0][1]
    assert items[0][2] is True          # blocking, not advice


def test_a_calibrated_robot_passes(tmp_path):
    (tmp_path / 'servos.local.yaml').write_text('/**/servo_driver: {ros__parameters: {}}\n')
    run = fake_run(0, 'robot_setup: калибровка есть у всех суставов (servos.local.yaml)')
    items = preflight.calibration_gate(str(tmp_path), 'floor', run)
    assert [ok for ok, _t, _r in items] == [True]
    assert 'калибровка есть' in items[0][1]
    assert run.calls == [str(tmp_path)]


def test_a_half_calibrated_robot_is_blocked_with_the_reason(tmp_path):
    (tmp_path / 'servos.local.yaml').write_text('/**/servo_driver: {ros__parameters: {}}\n')
    run = fake_run(3, 'robot_setup: калибровка неполная — в servos.local.yaml нет 10 суставов: lf_calf_joint')
    items = preflight.calibration_gate(str(tmp_path), 'floor', run)
    assert not items[0][0]
    assert 'калибровка неполная' in items[0][1]
    assert 'lf_calf_joint' in items[0][1]


def test_a_tool_that_cannot_run_blocks_instead_of_passing(tmp_path):
    (tmp_path / 'servos.local.yaml').write_text('/**/servo_driver: {ros__parameters: {}}\n')
    items = preflight.calibration_gate(str(tmp_path), 'floor', fake_run(1, 'не запустить robot_setup'))
    assert not items[0][0]
    assert 'проверка не запустилась' in items[0][1]


def test_the_bench_profile_asks_nothing_about_calibration(tmp_path):
    assert preflight.calibration_gate(str(tmp_path), 'bench') == []
    assert preflight.calibration_gate(str(tmp_path), '') == []


def test_the_profile_comes_from_the_launch_arguments():
    assert preflight.launch_profile('backend:=pca9685 profile:=floor gamepad:=true') == 'floor'
    assert preflight.launch_profile('backend:=pca9685') == ''
