import numpy as np
import pytest

from robotdog_autocal.fit import find_limit, fit_joint
from robotdog_autocal.servo_model import Linkage, ServoCal


def test_direct_fit_recovers_direction_offset_and_scale():
    true = ServoCal(direction=-1, offset_deg=48.3, range_deg=184.0)
    believed = ServoCal()
    q = np.linspace(20, 60, 9)
    pulses = [true.joint_to_pulse(a) for a in q]
    noisy = q + np.random.default_rng(0).normal(0, 0.3, len(q))
    r = fit_joint('j', believed, pulses, noisy)
    assert r.direction == -1
    assert r.offset_deg == pytest.approx(48.3, abs=0.4)
    assert r.range_deg == pytest.approx(184.0, abs=2.0)
    assert r.rms_deg < 0.5


def test_linkage_fit_uses_the_rod_model():
    link = Linkage(servo_arm_mm=15, joint_arm_mm=22, rod_mm=92, axis_distance_mm=90)
    true = ServoCal(direction=1, offset_deg=-83.0, min_deg=-170, max_deg=-10, linkage=link)
    believed = ServoCal(direction=-1, offset_deg=-90.0, min_deg=-170, max_deg=-10, linkage=link)
    q = np.linspace(-110, -60, 11)
    r = fit_joint('j', believed, [true.joint_to_pulse(a) for a in q], q)
    assert r.linkage and r.direction == 1
    assert r.offset_deg == pytest.approx(-83.0, abs=1e-3)


def test_joint_that_does_not_move_is_reported():
    with pytest.raises(ValueError):
        fit_joint('j', ServoCal(), [1300, 1400, 1500], [10.0, 10.0, 10.0])


def test_find_limit():
    cmd = [30, 33, 36, 39, 42, 45]
    meas = [30, 33, 36, 38.5, 38.6, 38.6]  # stop at ~38.5
    assert find_limit(cmd, meas, 1) == 3
