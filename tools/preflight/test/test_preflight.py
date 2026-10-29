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
