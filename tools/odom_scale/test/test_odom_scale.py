"""odom.scale: measured distance divided by what the robot reported, with sanity around it.

The number is one division; the work is refusing the divisions that would produce a
confident wrong answer - a walk that turned, a walk of nothing, a measurement that is off
by an order of magnitude.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import odom_scale as os_  # noqa: E402


def straight(metres, n=50):
    """A straight walk along +x with a little jitter, as dead reckoning produces."""
    return [(metres * i / (n - 1), 0.001 * (i % 3)) for i in range(n)]


def states(lines):
    return [s for s, _t in lines]


def test_a_straight_two_metres_reported_as_225_gives_the_floor_scale():
    lines = os_.judge(straight(2.25), measured_m=2.0)
    scales = [t for _s, t in lines if 'odom.scale' in t]
    assert scales and 'odom.scale = 0.889' in scales[0], lines


def test_perfect_dead_reckoning_gives_one():
    lines = os_.judge(straight(2.0), measured_m=2.0)
    assert any('odom.scale = 1.000' in t for _s, t in lines)


def test_a_walk_that_turned_is_refused():
    # Two legs at a right angle: the path is 2 m but the ends are 1.4 m apart, so a scale
    # computed from this would mix a turn into a distance.
    points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    lines = os_.judge(points, measured_m=2.0)
    assert 'плохо' in states(lines)
    assert any('не похож на прямой' in t for _s, t in lines)


def test_a_walk_that_came_back_to_the_start_is_refused():
    # Out and back: path 2 m, ends together. Dividing by zero or by the path both lie.
    points = [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)]
    lines = os_.judge(points, measured_m=2.0)
    assert 'плохо' in states(lines)
    assert any('вернулся в начало' in t for _s, t in lines)


def test_a_robot_that_did_not_move_has_no_scale():
    lines = os_.judge([(0.0, 0.0)] * 10, measured_m=2.0)
    assert any('не двигался' in t for _s, t in lines)


def test_a_measurement_off_by_an_order_of_magnitude_is_not_believed():
    lines = os_.judge(straight(0.2), measured_m=2.0)   # robot says 20 cm, tape says 2 m
    assert 'плохо' in states(lines)
    assert any('вне 0.7…1.3' in t for _s, t in lines)


def test_the_printed_line_is_pasteable():
    lines = os_.judge(straight(2.1), measured_m=2.0)
    text = '\n'.join(t for _s, t in lines)
    assert 'в robot.yaml: odom:' in text
    assert 'scale: 0.952' in text


def test_path_and_chord_are_different_things():
    points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert os_.path_length(points) == pytest.approx(2.0)
    assert os_.chord(points) == pytest.approx(2 ** 0.5)


def test_a_zero_reported_distance_does_not_divide_by_zero():
    assert os_.scale_from(2.0, 0.0) != os_.scale_from(2.0, 0.0)      # NaN
