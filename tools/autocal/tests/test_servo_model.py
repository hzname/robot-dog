import math

import pytest

from robotdog_autocal.servo_model import Linkage, ServoCal

# Reference values printed by dog_hardware/src/servo_driver.cpp (Linkage::jointDelta)
# for servo arm 15 mm, joint arm 25 mm, axis distance 90 mm (rod = 90 mm).
CPP_REFERENCE = {-90: -30.836, -45: -24.221, -15: -8.842, 0: 0.0, 15: 8.982, 30: 17.708,
                 45: 25.764, 60: 32.693, 90: 41.271}


def test_linkage_matches_cpp():
    l = Linkage(servo_arm_mm=15, joint_arm_mm=25, axis_distance_mm=90)
    for s, g in CPP_REFERENCE.items():
        assert l.joint_delta(s) == pytest.approx(g, abs=2e-3), s


def test_usable_range_stops_before_dead_point():
    lo, hi = Linkage(servo_arm_mm=15, joint_arm_mm=25, axis_distance_mm=90).usable_servo_range(90)
    assert -80 < lo < -60 and hi == 90


def test_parallelogram_is_linear():
    l = Linkage(servo_arm_mm=20, axis_distance_mm=90)
    for s in range(-80, 81, 20):
        assert l.joint_delta(s) == pytest.approx(s, abs=1e-9)


@pytest.mark.parametrize('link', [Linkage(), Linkage(servo_arm_mm=15, joint_arm_mm=22, rod_mm=92, axis_distance_mm=90)])
def test_round_trip(link):
    c = ServoCal(direction=-1, offset_deg=-90, min_deg=-170, max_deg=-10, linkage=link)
    for deg in range(-130, -49, 5):
        assert c.pulse_to_joint(c.joint_to_pulse(deg)) == pytest.approx(deg, abs=1e-6)


def test_coupling_and_centre():
    c = ServoCal(offset_deg=-45, coupled_to='lf_thigh_joint', coupling=1.0, min_deg=-170)
    assert c.center_us == 1370.0
    assert c.joint_to_pulse(-90, parent_deg=45) == pytest.approx(1370.0)
    assert c.pulse_to_joint(c.joint_to_pulse(-80, 30), 30) == pytest.approx(-80)


def test_from_params():
    c = ServoCal.from_params({'channel': 3.0, 'direction': -1.0, 'offset_deg': 5, 'servo_arm_mm': 12,
                              'axis_distance_mm': 80, 'coupled_to': '', 'coupling': 0})
    assert c.channel == 3 and c.direction == -1 and not c.linkage.direct
    assert math.isclose(c.linkage.axis_distance_mm, 80)
