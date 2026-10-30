"""End-to-end checks of the servo speed analyzer on synthetic traces.

Covers the tracer path synth_run -> analyze -> v_sat, the find_saturation
core rule, the duration cross-check, per-direction output, flags and the
invariance of the result to the shunt scale, start desync and dropped rows,
the CSV/metadata contract and the CLI exit codes, plus the determinism and
row contracts of the generator (D-08, D-09).
"""

import ast
import functools
import itertools
import json
import math
import os
import sys

import pytest

import analyze as analyze_module
from analyze import (analyze, cross_check, find_saturation, load_csv, load_meta,
                     main as analyze_main, validate_meta)
from synth import (DEFAULT_SPEEDS, EXTENDED_SPEEDS, main as synth_main, synth_run,
                   write_run)

TICK = 0.001


@functools.lru_cache(maxsize=None)
def _run(v_max, **kw):
    """Cached synth_run: identical synthetic runs are generated only once."""
    return synth_run(v_max, **kw)


def _expected_index(speeds, v_max):
    """Index of the first grid speed >= v_max."""
    return next(i for i, v in enumerate(speeds) if v >= v_max - 1e-9)


def test_tracer_synth_to_v_sat_within_one_grid_step():
    """analyze(*synth_run(6.0)) finds the kink within one grid step of v_max."""
    rows, meta = _run(6.0)
    result = analyze(rows, meta)
    assert result['v_sat_status'] == 'ok'
    grid = meta['speeds_rad_s']
    idx = grid.index(result['v_sat_rad_s'])
    first = _expected_index(grid, 6.0)
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


# ---------- duration cross-check, flags, per-direction output, invariance ----------

@pytest.mark.parametrize(
    'args, expected',
    [
        ((6.0, 5.8), (True, 6.0, None)),
        ((6.5, 5.0), (False, 5.0, 'v_sat_v_dur_disagree')),
        ((5.0, 6.5), (False, 5.0, 'v_sat_v_dur_disagree')),
        ((6.0, None), (False, None, 'not_saturated')),
        ((None, 7.0), (False, None, 'not_saturated')),
        ((10.0, 8.6), (True, 10.0, None)),
        ((10.0, 8.4), (False, 8.4, 'v_sat_v_dur_disagree')),
    ],
    ids=['agree-3pct', 'disagree-down', 'disagree-up', 'no-dur', 'no-kink',
         'agree-at-15pct', 'disagree-at-16pct'],
)
def test_cross_check_cases(args, expected):
    assert cross_check(*args) == expected


@pytest.mark.parametrize('v_max, speeds', [(3.5, DEFAULT_SPEEDS), (4.5, DEFAULT_SPEEDS),
                                           (6.0, DEFAULT_SPEEDS), (7.5, EXTENDED_SPEEDS)])
@pytest.mark.parametrize('noise', [0.008, 0.015, 0.03])
def test_kink_within_one_grid_step(v_max, speeds, noise):
    r = analyze(*_run(v_max, noise_a=noise, speeds=speeds))
    assert r['v_sat_status'] == 'ok' and r['v_sat_rad_s'] is not None
    first = _expected_index(speeds, v_max)
    assert first <= list(speeds).index(r['v_sat_rad_s']) <= first + 1
    assert r['v_dur_plateau_rad_s'] == pytest.approx(v_max, rel=0.10)
    assert r['agreement'] is True
    assert r['servo_max_speed_rad_s'] == r['v_sat_rad_s']
    assert r['flag'] is None


@pytest.mark.parametrize('v_max, speeds', [(4.0, DEFAULT_SPEEDS), (6.0, DEFAULT_SPEEDS),
                                           (8.0, EXTENDED_SPEEDS)])
@pytest.mark.parametrize('noise', [0.008, 0.03])
def test_ramp_timing_kink(v_max, speeds, noise):
    r = analyze(*_run(v_max, ramp_timing=True, noise_a=noise, speeds=speeds))
    assert r['v_sat_status'] == 'ok' and r['v_sat_rad_s'] is not None
    first = _expected_index(speeds, v_max)
    assert first <= list(speeds).index(r['v_sat_rad_s']) <= first + 1
    assert r['v_dur_plateau_rad_s'] == pytest.approx(v_max, rel=0.10)
    assert r['agreement'] is True
    assert r['servo_max_speed_rad_s'] == r['v_sat_rad_s']
    assert r['flag'] is None
    assert r['strokes']['rejected'] <= 3  # stroke 0 after APPROACH may be dropped


def test_soft_kink_flags_disagreement():
    r = analyze(*_run(6.0, kp=100.0, a_max=3000.0))
    assert r['agreement'] is False
    assert r['servo_max_speed_rad_s'] == r['v_dur_plateau_rad_s']
    assert r['servo_max_speed_rad_s'] <= r['v_sat_rad_s']
    assert 'v_sat_v_dur_disagree' in r['flag']


@pytest.mark.parametrize('v_max', [12.0, 7.5])
def test_grid_without_saturation(v_max):
    r = analyze(*_run(v_max))
    assert r['v_sat_status'] == 'not_saturated'
    assert r['v_sat_rad_s'] is None
    assert r['servo_max_speed_rad_s'] is None
    assert 'not_saturated' in r['flag']
    if v_max == 7.5:
        # above ~7 rad/s the default grid has only two pairs left: v_dur is a lower bound
        assert r['v_dur_plateau_rad_s'] == pytest.approx(7.5, rel=0.10)


@pytest.mark.parametrize('scale', [0.1, 10.0])
def test_shunt_scale_invariance(scale):
    base = analyze(*_run(6.0))
    r = analyze(*_run(6.0, shunt_scale=scale))
    assert r['v_sat_rad_s'] == base['v_sat_rad_s']
    assert r['v_dur_plateau_rad_s'] == pytest.approx(base['v_dur_plateau_rad_s'], rel=0.005)
    assert r['noise_floor'] == pytest.approx(base['noise_floor'], rel=0.10)
    for a, b in zip(r['distance_rel'], base['distance_rel']):
        assert a == pytest.approx(b, rel=0.10)
    assert r['plateau_current_a'] == pytest.approx(base['plateau_current_a'] * scale, rel=0.02)


def test_jitter_does_not_move_the_kink():
    r0 = analyze(*_run(6.0, jitter_max_s=0.0))
    r = analyze(*_run(6.0, jitter_max_s=0.02))
    assert r['v_sat_rad_s'] == r0['v_sat_rad_s']
    assert r['v_dur_plateau_rad_s'] == pytest.approx(r0['v_dur_plateau_rad_s'], rel=0.03)


def test_drop_fraction_shifts_index_at_most_one_step():
    base = analyze(*_run(6.0))
    r = analyze(*_run(6.0, drop_fraction=0.1))
    grid = list(DEFAULT_SPEEDS)
    assert abs(grid.index(r['v_sat_rad_s']) - grid.index(base['v_sat_rad_s'])) <= 1


def test_per_direction_and_bus_conditions():
    r = analyze(*_run(6.0))
    up = r['per_direction']['up']
    down = r['per_direction']['down']
    for side in (up, down):
        assert {'v_sat_rad_s', 'v_dur_plateau_rad_s', 'plateau_current_a'} <= set(side)
    assert up['v_sat_rad_s'] == r['v_sat_rad_s']
    assert down['v_sat_rad_s'] == r['v_sat_rad_s']
    assert up['plateau_current_a'] >= down['plateau_current_a'] + 0.04  # leg weight helps one way
    assert 5.90 <= r['bus_v_mean'] <= 5.98


def test_incomplete_run_gets_flag():
    r = analyze(*_run(6.0, stop_reason='overcurrent'))
    assert 'run_incomplete:overcurrent' in r['flag']


def test_plateau_current_level_and_key_contract():
    r = analyze(*_run(6.0))
    assert r['plateau_current_a'] == pytest.approx(0.46, abs=0.03)
    for key in ('v_sat_rad_s', 'v_sat_status', 'v_dur_plateau_rad_s', 'v_dur_rad_s',
                'agreement', 'servo_max_speed_rad_s', 'flag', 'plateau_current_a',
                'noise_floor', 'bus_v_mean', 'per_direction', 'speeds_rad_s',
                'distance_rel', 'eps_rel', 'strokes'):
        assert key in r, key
    json.dumps(r)


# ---------- CSV and metadata I/O, CLI, plotting, static contract ----------

HEADER = 't_s,stroke_id,direction,cmd_us,shunt_raw,bus_raw'


def _csv_text(*lines):
    return HEADER + '\n' + ''.join(line + '\n' for line in lines)


def _same_result(a, b, path='result'):
    """Compare analyze results, allowing float rounding of the CSV round-trip.

    The CSV keeps t_s at 1 us and cmd_us at 0.1 us, so continuous values can
    move in the last digits; the decisive fields are compared exactly by the
    caller.
    """
    if isinstance(a, bool) or isinstance(b, bool):
        assert a == b, path
    elif isinstance(a, float) or isinstance(b, float):
        assert a == pytest.approx(b, rel=1e-6), path
    elif isinstance(a, dict):
        assert set(a) == set(b), path
        for key in a:
            _same_result(a[key], b[key], '%s.%s' % (path, key))
    elif isinstance(a, list):
        assert len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b)):
            _same_result(x, y, '%s[%d]' % (path, i))
    else:
        assert a == b, path


def test_write_run_load_roundtrip_matches_memory(tmp_path):
    rows, meta = _run(6.0, ramp_timing=True)
    p = str(tmp_path / 'run.csv')
    meta_path = write_run(p, rows, meta)
    assert meta_path == p + '.meta.json'
    loaded = load_csv(p)
    loaded_meta = load_meta(meta_path)
    assert len(loaded) == len(rows)
    assert all(len(row) == 6 for row in loaded)
    assert any(row[5] is None for row in loaded)
    assert any(row[5] is not None for row in loaded)
    in_memory = analyze(rows, meta)
    from_disk = analyze(loaded, loaded_meta)
    for key in ('v_sat_rad_s', 'v_sat_status', 'agreement', 'servo_max_speed_rad_s',
                'flag', 'strokes', 'speeds_rad_s'):
        assert from_disk[key] == in_memory[key], key
    _same_result(from_disk, in_memory)


def test_load_csv_rejects_bad_header(tmp_path):
    p = tmp_path / 'bad.csv'
    p.write_text('a,b,c\n', encoding='utf-8')
    with pytest.raises(ValueError) as ei:
        load_csv(str(p))
    assert 'line 1' in str(ei.value)


@pytest.mark.parametrize('line', [
    'x,0,1,1400.0,0,',
    '0.0,1.5,1,1400.0,0,',
    '0.0,-2,1,1400.0,0,',
    'nan,0,1,1400.0,0,',
    '0.0,0,1,inf,0,',
], ids=['t-not-number', 'stroke-not-int', 'stroke-below-minus-one',
        't-not-finite', 'cmd-not-finite'])
def test_load_csv_rejects_bad_fields(tmp_path, line):
    p = tmp_path / 'bad.csv'
    p.write_text(_csv_text(line), encoding='utf-8')
    with pytest.raises(ValueError) as ei:
        load_csv(str(p))
    assert 'line 2' in str(ei.value)


def test_load_csv_rejects_rows_beyond_max(tmp_path, monkeypatch):
    p = tmp_path / 'many.csv'
    p.write_text(_csv_text(*['0.0,0,1,1400.0,0,'] * 4), encoding='utf-8')
    monkeypatch.setattr(analyze_module, 'MAX_ROWS', 3)
    with pytest.raises(ValueError) as ei:
        load_csv(str(p))
    assert 'line 5' in str(ei.value)


def test_load_csv_accepts_direction_zero_outside_strokes(tmp_path):
    p = tmp_path / 'ok.csv'
    p.write_text(_csv_text('0.0,-1,0,1400.0,0,', '0.001,-1,-1,1400.0,0,',
                           '0.002,-1,1,1400.0,0,'), encoding='utf-8')
    rows = load_csv(str(p))
    assert [row[2] for row in rows] == [0, -1, 1]


def test_load_csv_rejects_direction_zero_inside_stroke(tmp_path):
    p = tmp_path / 'bad.csv'
    p.write_text(_csv_text('0.0,0,0,1400.0,0,'), encoding='utf-8')
    with pytest.raises(ValueError) as ei:
        load_csv(str(p))
    assert 'line 2' in str(ei.value) and 'direction' in str(ei.value)


def test_load_csv_rejects_direction_two_outside_stroke(tmp_path):
    p = tmp_path / 'bad.csv'
    p.write_text(_csv_text('0.0,-1,2,1400.0,0,'), encoding='utf-8')
    with pytest.raises(ValueError) as ei:
        load_csv(str(p))
    assert 'line 2' in str(ei.value)


def test_validate_meta_rejects_missing_fields():
    meta = synth_run(6.0)[1]
    for field in ('shunt_ohm', 'us_per_deg', 'speeds_rad_s', 'strokes_per_speed'):
        bad = dict(meta)
        del bad[field]
        with pytest.raises(ValueError):
            validate_meta(bad)


@pytest.mark.parametrize('field, value', [('shunt_ohm', True), ('us_per_deg', True),
                                          ('shunt_ohm', 0.0), ('us_per_deg', -1.0)])
def test_validate_meta_rejects_bool_and_nonpositive(field, value):
    meta = synth_run(6.0)[1]
    bad = dict(meta)
    bad[field] = value
    with pytest.raises(ValueError):
        validate_meta(bad)


@pytest.mark.parametrize('speeds', [[1.5, 1.5, 2.0, 2.5], [1.5, 2.0, 2.5],
                                    [3.0, 2.0, 1.5, 0.5]],
                         ids=['duplicate', 'too-short', 'descending'])
def test_validate_meta_rejects_bad_speed_grid(speeds):
    meta = synth_run(6.0)[1]
    bad = dict(meta)
    bad['speeds_rad_s'] = speeds
    with pytest.raises(ValueError):
        validate_meta(bad)


@pytest.mark.parametrize('value', [3, 5, 10.0])
def test_validate_meta_rejects_bad_strokes_per_speed(value):
    meta = synth_run(6.0)[1]
    bad = dict(meta)
    bad['strokes_per_speed'] = value
    with pytest.raises(ValueError):
        validate_meta(bad)


def test_main_writes_result_json_with_all_keys(tmp_path):
    rows, meta = _run(6.0)
    p = str(tmp_path / 'run.csv')
    write_run(p, rows, meta)
    out = str(tmp_path / 'result.json')
    assert analyze_main(['--csv', p, '--out', out]) == 0
    with open(out, encoding='utf-8') as fh:
        r = json.load(fh)
    for key in ('v_sat_rad_s', 'v_sat_status', 'v_dur_plateau_rad_s', 'v_dur_rad_s',
                'agreement', 'servo_max_speed_rad_s', 'flag', 'plateau_current_a',
                'noise_floor', 'bus_v_mean', 'per_direction', 'speeds_rad_s',
                'distance_rel', 'eps_rel', 'strokes'):
        assert key in r, key


def test_main_returns_1_when_not_saturated(tmp_path):
    rows, meta = _run(12.0)
    p = str(tmp_path / 'run.csv')
    write_run(p, rows, meta)
    assert analyze_main(['--csv', p]) == 1


def test_main_returns_2_on_input_errors(tmp_path, capsys):
    assert analyze_main(['--csv', str(tmp_path / 'nope.csv')]) == 2
    bad = tmp_path / 'bad.csv'
    bad.write_text('nope\n', encoding='utf-8')
    assert analyze_main(['--csv', str(bad)]) == 2
    lonely = tmp_path / 'lonely.csv'
    lonely.write_text(_csv_text('0.0,0,1,1400.0,0,'), encoding='utf-8')
    assert analyze_main(['--csv', str(lonely)]) == 2  # the meta file is missing
    rows, meta = _run(6.0)
    q = str(tmp_path / 'run.csv')
    write_run(q, rows, meta)
    assert analyze_main(['--csv', q, '--out', q]) == 2
    assert analyze_main(['--csv', q, '--plot', q + '.meta.json']) == 2
    assert capsys.readouterr().err != ''


def test_main_help_mentions_options(capsys):
    with pytest.raises(SystemExit) as ei:
        analyze_main(['--help'])
    assert ei.value.code == 0
    out = capsys.readouterr().out
    for opt in ('--csv', '--meta', '--out', '--plot'):
        assert opt in out


def test_plot_writes_png(tmp_path):
    pytest.importorskip('matplotlib')
    rows, meta = _run(6.0)
    p = str(tmp_path / 'run.csv')
    write_run(p, rows, meta)
    png = tmp_path / 'plot.png'
    assert analyze_main(['--csv', p, '--plot', str(png)]) == 0
    assert png.exists() and png.stat().st_size > 0


def test_plot_without_matplotlib_returns_2(tmp_path, monkeypatch, capsys):
    rows, meta = _run(6.0)
    p = str(tmp_path / 'run.csv')
    write_run(p, rows, meta)
    monkeypatch.setitem(sys.modules, 'matplotlib', None)
    monkeypatch.delitem(sys.modules, 'matplotlib.pyplot', raising=False)
    assert analyze_main(['--csv', p, '--plot', str(tmp_path / 'x.png')]) == 2
    assert 'matplotlib' in capsys.readouterr().err


def test_synth_main_writes_run(tmp_path, capsys):
    out = str(tmp_path / 'demo.csv')
    assert synth_main(['--v-max', '6.0', '--out', out]) == 0
    assert os.path.exists(out) and os.path.exists(out + '.meta.json')
    assert out in capsys.readouterr().out


ALLOWED_MODULE_IMPORTS = {'csv', 'json', 'math', 'os', 'sys', 'argparse', 'numpy',
                          '__future__'}


@pytest.mark.parametrize('module', ['analyze.py', 'synth.py'])
def test_ast_module_contract(module):
    src_path = os.path.join(os.path.dirname(__file__), '..', module)
    with open(src_path, encoding='utf-8') as fh:
        source = fh.read()
    tree = ast.parse(source, feature_version=(3, 8))
    names = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            names |= {a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                names.add(node.module.split('.')[0])
    assert names <= ALLOWED_MODULE_IMPORTS, names - ALLOWED_MODULE_IMPORTS
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            assert node.func.id not in ('eval', 'exec')
        if isinstance(node, ast.Subscript) and isinstance(node.value, ast.Name):
            assert node.value.id not in ('list', 'dict', 'tuple', 'set')
    if module == 'analyze.py':
        plot = next(n for n in tree.body
                    if isinstance(n, ast.FunctionDef) and n.name == 'plot_result')
        inside = {a.name.split('.')[0] for n in ast.walk(plot)
                  if isinstance(n, ast.Import) for a in n.names}
        assert 'matplotlib' in inside
