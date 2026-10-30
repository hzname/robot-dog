"""The floor-run verdict: ten minutes, no latch, no bus errors, and nothing invented.

Everything here runs without ROS - the judgement is a pure function over samples, and the
message mapping is fed plain objects with the same fields the real messages have.
"""

import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import floor_run as fr  # noqa: E402


def quat(roll_deg=0.0, pitch_deg=0.0):
    """A level-ish orientation as (x, y, z, w), small angles: enough for the tilt read."""
    import math
    r, p = math.radians(roll_deg) / 2, math.radians(pitch_deg) / 2
    return SimpleNamespace(x=r, y=p, z=0.0, w=1.0)


def good_run():
    run = fr.Run()
    run.duration = 620.0
    run.voltage = [(0.0, 6.1), (300.0, 5.9), (620.0, 5.7)]
    run.current = [(float(i), 3.0 + i % 5) for i in range(200)]
    run.tilt = [(0.0, 1.0, 2.0), (300.0, -3.0, 6.0)]
    run.states = [(0.0, 'stand'), (5.0, 'walk')]
    run.estops = [(0.0, False)]
    run.diag = [(1.0, 'servo_driver', 'ok'), (2.0, 'power_monitor', 'calibrated')]
    run.odom = [(0.0, 0.0, 0.0), (300.0, 3.0, 4.0), (620.0, 6.0, 8.0)]
    return run


def states(lines):
    return {text.split(':')[0]: state for state, text in lines}


def test_a_clean_ten_minute_run_passes():
    lines = fr.judge(good_run(), temp_c=52.0)
    ok, summary = fr.verdict(lines)
    assert ok, summary
    assert summary.startswith('ГОДИТСЯ')


def test_a_short_run_does_not_count():
    run = good_run()
    run.duration = 120.0
    ok, summary = fr.verdict(fr.judge(run, temp_c=40.0))
    assert not ok
    assert 'длительность' in summary


def test_a_latched_estop_fails_the_run():
    run = good_run()
    run.estops = [(0.0, False), (10.0, True), (600.0, True)]
    lines = fr.judge(run, temp_c=45.0)
    assert states(lines)['E-STOP'] == fr.BAD


def test_a_press_that_was_released_is_only_counted():
    run = good_run()
    run.estops = [(0.0, False), (10.0, True), (12.0, False), (620.0, False)]
    lines = fr.judge(run, temp_c=45.0)
    text = [t for s, t in lines if t.startswith('E-STOP')][0]
    assert states(lines)['E-STOP'] == fr.OK
    assert 'нажатий за прогон: 1' in text


def test_no_estop_messages_on_a_clean_run_are_not_a_gap():
    """The driver announces a latch once; a clean run simply has no messages. The state
    machine is what keeps reporting, so the verdict rests on it."""
    run = good_run()
    run.estops = []
    run.states = [(0.0, 'stand'), (620.0, 'passive')]
    assert states(fr.judge(run, temp_c=45.0))['E-STOP'] == fr.OK
    run.states = [(0.0, 'stand'), (620.0, 'estop')]
    assert states(fr.judge(run, temp_c=45.0))['E-STOP'] == fr.BAD


def test_a_bus_error_fails_the_run():
    run = good_run()
    run.diag = [(1.0, 'servo_driver', 'bus write failed 3 frames in a row')]
    lines = fr.judge(run, temp_c=45.0)
    assert states(lines)['ошибки шины'] == fr.BAD


def test_temperature_nobody_measured_leaves_the_run_unfinished():
    ok, summary = fr.verdict(fr.judge(good_run(), temp_c=None))
    assert not ok                       # unproven is not "passed"
    assert 'НЕ ЗАВЕРШЕНО' in summary
    assert 'температура' in summary


def test_a_hot_servo_fails_the_run():
    lines = fr.judge(good_run(), temp_c=71.0)
    assert states(lines)['температура серв'] == fr.BAD


def test_a_sagging_rail_fails_the_run():
    run = good_run()
    run.voltage = [(0.0, 6.1), (100.0, 5.2), (620.0, 5.9)]
    assert states(fr.judge(run, temp_c=45.0))['шина'] == fr.BAD


def test_tilt_past_the_hard_limit_is_a_fall():
    run = good_run()
    run.tilt = [(0.0, 0.0, 1.0), (10.0, 2.0, 34.0)]
    assert states(fr.judge(run, temp_c=45.0))['наклон корпуса'] == fr.BAD


def test_tilt_between_the_limits_is_not_a_verdict():
    run = good_run()
    run.tilt = [(0.0, 0.0, 1.0), (10.0, 2.0, 25.0)]
    assert states(fr.judge(run, temp_c=45.0))['наклон корпуса'] == fr.UNKNOWN


def test_distance_comes_from_odometry():
    run = good_run()
    assert run.distance() == pytest.approx(10.0)   # two 3-4-5 legs


def test_p99_uses_nearest_rank_not_interpolation():
    """p99 of the numbers 1..100 is the 99th of them, exactly - not 99.01 between two
    samples. Interpolation would invent a value nothing drew."""
    run = good_run()
    run.current = [(float(i), float(i)) for i in range(1, 101)]
    text = [t for _s, t in fr.judge(run, temp_c=45.0) if t.startswith('ток')][0]
    assert 'p99 99.00' in text
    assert 'максимум 100.00' in text


def test_an_empty_run_names_what_is_missing_instead_of_passing():
    ok, summary = fr.verdict(fr.judge(fr.Run(), temp_c=None))
    assert not ok
    assert 'шина' in summary and 'E-STOP' in summary


def test_messages_are_mapped_by_their_real_fields():
    run = fr.Run()
    run.add('/dog/power', SimpleNamespace(voltage=5.9, current=-3.2), 1.0)
    run.add('/dog/imu/data', SimpleNamespace(orientation=quat(pitch_deg=10.0)), 1.0)
    run.add('/dog/state', SimpleNamespace(data='estop'), 1.0)
    run.add('/dog/estop', SimpleNamespace(data=True), 1.0)
    run.add('/dog/odom', SimpleNamespace(
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0)))), 1.0)
    run.add('/dog/diagnostics', SimpleNamespace(status=[SimpleNamespace(name='imu',
                                                                       message='calibrated')]), 1.0)
    assert run.current[0][1] == pytest.approx(3.2)      # drawn current, sign dropped
    assert run.tilt[0][2] == pytest.approx(10.0, abs=0.5)
    assert run.states[0][1] == 'estop'
    assert run.estops[0][1] is True
    assert run.odom[0][1:] == (1.0, 2.0)
    assert run.diag[0][1:] == ('imu', 'calibrated')


def test_the_clock_starts_at_zero():
    run = fr.Run()
    run.voltage = [(100.0, 6.0), (130.0, 6.0)]
    run.shift(-100.0)
    assert [t for t, _v in run.voltage] == [0.0, 30.0]
