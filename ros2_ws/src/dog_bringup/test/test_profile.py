"""The floor profile must be stricter than the bench profile on every knob it touches.

No ROS here: the launch file reads `profile.py` and the test reads it too, so a
number typed by hand cannot drift away from what the test expects.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'launch'))
import profile  # noqa: E402


def test_an_unknown_profile_is_refused():
    with pytest.raises(ValueError, match='unknown profile'):
        profile.params('floorr', 'locomotion')
    with pytest.raises(ValueError):
        profile.check('')


def test_floor_walks_at_five_centimetres_per_second():
    limits = profile.params(profile.FLOOR, 'locomotion')
    assert limits['limits.max_vx'] == pytest.approx(0.05)
    # The teleop limit times the stick scale must not exceed what locomotion clamps.
    teleop = profile.params(profile.FLOOR, 'joy_teleop')
    assert teleop['max_vx'] * teleop['normal_scale'] <= limits['limits.max_vx'] + 1e-9


def test_floor_turbo_gives_nothing_extra():
    teleop = profile.params(profile.FLOOR, 'joy_teleop')
    assert teleop['turbo_scale'] <= teleop['normal_scale']


def test_floor_requires_the_imu_and_drops_the_assists():
    limits = profile.params(profile.FLOOR, 'locomotion')
    assert limits['safety.profile'] == 'floor'
    assert limits['safety.require_imu'] is True
    # Slope compensation and heading hold would fight an operator who walks beside
    # the robot on a first outing: both off.
    assert limits['slope.compensation'] is False
    assert limits['heading.hold'] is False


def test_floor_web_is_view_only():
    assert profile.web_drive(profile.FLOOR) is False
    assert profile.params(profile.FLOOR, 'web_teleop')['drive_enabled'] is False


def test_calibration_never_opens_on_the_floor():
    assert profile.allow_calibration(profile.FLOOR, calibrate=True) is False
    assert profile.allow_calibration(profile.FLOOR, calibrate=False) is False


def test_calibration_opens_on_the_bench_only_when_asked():
    assert profile.allow_calibration(profile.BENCH, calibrate=False) is False
    assert profile.allow_calibration(profile.BENCH, calibrate=True) is True


def test_bench_changes_nothing():
    for node in ('locomotion', 'joy_teleop', 'web_teleop'):
        assert profile.params(profile.BENCH, node) == {}
    assert profile.web_drive(profile.BENCH) is True


def test_the_summary_names_what_changed():
    assert 'limits.max_vx=0.05' in profile.summary(profile.FLOOR)
    assert profile.summary(profile.BENCH) == 'profile bench: YAML defaults'
