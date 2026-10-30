"""Thresholds come from a recording, not from taste (SAF-07).

Standing rule: the overcurrent trip is 1.3-1.5 x p99 of a recorded walk, the rail trip
sits a margin below the recorded minimum. These tests pin the arithmetic and, just as
important, that the tool refuses to hand over a threshold the sensor cannot measure.
"""

import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import power_thresholds as pt  # noqa: E402


def test_percentile_is_nearest_rank():
    values = list(range(1, 101))          # 1..100
    assert pt.percentile(values, 0.99) == 99
    assert pt.percentile(values, 0.50) == 50
    assert pt.percentile([5.0], 0.99) == 5.0
    assert pt.percentile([], 0.99) == 0.0


def test_junk_lines_are_skipped(tmp_path):
    f = tmp_path / 'current.txt'
    f.write_text('---\n1.5\n\n2.5\nзначение\n3.0\n', encoding='utf-8')
    assert pt.numbers(str(f)) == [1.5, 2.5, 3.0]


def test_the_suggestion_is_1_3_to_1_5_times_p99():
    walk = [1.0] * 95 + [8.0] * 5          # p99 = 8 A, a stall-free walk with peaks
    params, stats, warnings = pt.thresholds(walk, [6.0] * len(walk))
    assert params['overcurrent_a'] == pytest.approx(11.2, abs=0.05)   # 1.4 x 8
    assert 1.3 * stats['current_p99'] <= params['overcurrent_a'] <= 1.5 * stats['current_p99']
    assert not warnings


def test_the_rail_trip_sits_below_the_recorded_minimum():
    volts = [6.0] * 90 + [5.4] * 10
    params, stats, _ = pt.thresholds([2.0] * len(volts), volts)
    assert stats['voltage_min'] == 5.4
    assert params['undervoltage_v'] == pytest.approx(5.2, abs=0.05)


def test_a_switched_off_rail_is_not_a_reading():
    # 0 V means the servo rail is off; it must not drag the minimum to zero.
    volts = [0.0, 0.0, 6.0, 5.9, 6.1]
    _, stats, _ = pt.thresholds([1.0] * len(volts), volts)
    assert stats['voltage_min'] == 5.9


def test_the_tool_refuses_a_threshold_the_sensor_cannot_reach():
    # Twelve MG996R on an INA219 with its stock 0.1 ohm shunt: 3.2 A at most.
    walk = [2.0] * 95 + [9.0] * 5      # p99 lands on the 9 A samples
    params, _, warnings = pt.thresholds(walk, [6.0] * 100, sensor='ina219-0.1')
    assert params['overcurrent_a'] > 3.2
    assert any('шунт' in w for w in warnings)


def test_an_empty_recording_is_an_error_not_a_zero(monkeypatch):
    with pytest.raises(ValueError):
        pt.thresholds([], [6.0])


def test_main_prints_a_config_block(tmp_path, capsys):
    current = tmp_path / 'i.txt'
    current.write_text('\n'.join(['4.0'] * 100), encoding='utf-8')
    voltage = tmp_path / 'v.txt'
    voltage.write_text('\n'.join(['5.8'] * 100), encoding='utf-8')
    code = pt.main(['--current', str(current), '--voltage', str(voltage)])
    out = capsys.readouterr().out
    assert code == 0
    assert '/**/power_monitor:' in out
    assert 'overcurrent_a: 5.6' in out
    assert 'undervoltage_v: 5.6' in out


def test_main_says_so_when_there_is_nothing_to_read(tmp_path, capsys):
    empty = tmp_path / 'i.txt'
    empty.write_text('---\n', encoding='utf-8')
    assert pt.main(['--current', str(empty)]) == 2
    assert 'ошибка' in capsys.readouterr().err
