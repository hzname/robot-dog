"""End-to-end checks of the servo speed analyzer on synthetic traces.

Covers the tracer path synth_run -> analyze -> v_sat, the find_saturation
core rule, the determinism of the generator and the row contracts of the
plain and ramp_timing synthetic runs (D-08).
"""

import functools
import itertools
import json
import math

import pytest

from analyze import analyze, find_saturation
from synth import DEFAULT_SPEEDS, EXTENDED_SPEEDS, synth_run

TICK = 0.001


@functools.lru_cache(maxsize=None)
def _run(v_max, **kw):
    """Cached synth_run: identical synthetic runs are generated only once."""
    return synth_run(v_max, **kw)


def test_tracer_synth_to_v_sat_within_one_grid_step():
    """analyze(*synth_run(6.0)) finds the kink within one grid step of v_max."""
    rows, meta = _run(6.0)
    result = analyze(rows, meta)
    assert result['v_sat_status'] == 'ok'
    grid = meta['speeds_rad_s']
    idx = grid.index(result['v_sat_rad_s'])
    first = next(i for i, v in enumerate(grid) if v >= 6.0 - 1e-9)
    assert first <= idx <= first + 1


@pytest.mark.parametrize(
    'speeds, distances, eps, expected',
    [
        ((1, 2, 3, 4, 5, 6), (10, 8, 0.1, 0.1, 0.1), 1, 3),
        ((1, 2, 3, 4, 5, 6), (10, 8, 0.1, 0.1, 5), 1, None),
        ((1, 2, 3, 4, 5, 6), (10, 8, 6, 0.1, 0.1), 1, None),
        ((1, 2, 3, 4, 5, 6, 7), (10, 0.1, 8, 0.1, 0.1, 0.1), 1, 4),
        ((1, 2, 3, 4, 5, 6), (0.1, 0.2, 0.5, 0.3, 0.9), 1, 1),
        ((1, 2, 3, 4, 5, 6), (10, 8, 1.0, 1.0, 1.0), 1, 3),
    ],
    ids=['kink-at-third', 'below-kink-islands', 'two-pairs-above', 'early-dip',
         'all-below-eps', 'distance-equal-eps'],
)
def test_find_saturation_cases(speeds, distances, eps, expected):
    assert find_saturation(speeds, distances, eps) == expected


def test_find_saturation_length_mismatch():
    with pytest.raises(ValueError):
        find_saturation((1, 2, 3), (1.0,), 1.0)


def test_synth_run_is_deterministic_by_seed():
    a = synth_run(6.0, seed=1)
    b = synth_run(6.0, seed=1)
    c = synth_run(6.0, seed=2)
    assert a == b
    assert a != c


def test_synth_rows_contract():
    """Rows: 6 fields, pre-roll with stroke_id -1 / direction 0, strokes from 0
    with alternating +1/-1 (hold rows included), bus only every 8th row, and
    the cmd_us span equals 2*amp_deg*us_per_deg."""
    rows, meta = _run(3.5)
    assert all(len(row) == 6 for row in rows)
    per = meta['strokes_per_speed']
    for _, stroke_id, direction, _, _, _ in rows:
        if stroke_id == -1:
            assert direction == 0
        else:
            j = stroke_id % per
            assert direction == (1 if j % 2 == 0 else -1)
    stroke_ids = sorted({row[1] for row in rows})
    assert stroke_ids[0] == -1
    assert stroke_ids[1:] == list(range(len(DEFAULT_SPEEDS) * per))
    cmds = [row[3] for row in rows]
    assert max(cmds) - min(cmds) == pytest.approx(2 * meta['amp_deg'] * meta['us_per_deg'])
    bus_rows = [i for i, row in enumerate(rows) if row[5] is not None]
    assert bus_rows == list(range(0, len(rows), 8))


def test_synth_ramp_timing_contract():
    """ramp_timing repeats the 01-09 ramp: APPROACH first, strokes back to back
    with a hold each, REST between groups (not after the last), ids without
    gaps, time on a 1 ms grid."""
    rows, meta = _run(3.5, ramp_timing=True)
    ts = [row[0] for row in rows]
    assert ts[0] == pytest.approx(0.0)
    assert all(b - a == pytest.approx(TICK, abs=1e-9) for a, b in zip(ts, ts[1:]))
    assert rows[0][1] == -1 and rows[0][2] == 0

    segments = [(sid, list(group)) for sid, group in
                itertools.groupby(rows, key=lambda r: r[1])]
    expected = [-1]
    for g in range(len(DEFAULT_SPEEDS)):
        expected += list(range(g * 10, (g + 1) * 10))
        if g < len(DEFAULT_SPEEDS) - 1:
            expected.append(-1)
    assert [sid for sid, _ in segments] == expected

    span = math.radians(2 * meta['amp_deg'])
    hold_n = int(round(meta['hold_s'] / TICK))
    for sid, group in segments:
        duration = (group[-1][0] - group[0][0]) + TICK
        if sid >= 0:
            v = meta['speeds_rad_s'][sid // meta['strokes_per_speed']]
            assert duration == pytest.approx(span / v + meta['hold_s'], abs=0.003)
            hold = group[-hold_n:]
            assert len({row[3] for row in hold}) == 1  # command constant in the hold
            assert all(row[1] == sid and row[2] == group[0][2] for row in hold)
        else:
            assert {row[2] for row in group} == {0}
    neg = [group for sid, group in segments if sid == -1]
    assert len(neg) == len(DEFAULT_SPEEDS)  # APPROACH + one REST per gap
    approach = neg[0]
    assert approach[0][3] == pytest.approx(meta['center_us'])  # hold at the center
    app_dur = (approach[-1][0] - approach[0][0]) + TICK
    v0 = meta['speeds_rad_s'][0]
    assert app_dur == pytest.approx(meta['hold_s'] + math.radians(meta['amp_deg']) / v0,
                                    abs=0.003)
    rest_cmd = (meta['center_us']
                - math.radians(meta['amp_deg']) * meta['us_per_deg'] * 180.0 / math.pi)
    for group in neg[1:]:  # REST: constant at -amp
        assert len(group) == pytest.approx(meta['rest_s'] / TICK, abs=2)
        assert all(row[3] == pytest.approx(rest_cmd, abs=1e-6) for row in group)
