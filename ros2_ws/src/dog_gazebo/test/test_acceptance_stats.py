"""Pure-stdlib checks of dog_gazebo.acceptance_stats: repeat classification,
the min/median summary, the D-01/D-06/D-07 scoring rules, the D-05 push
threshold and the JSON schema 1 assembly - no ROS, no Gazebo."""

import ast
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

from dog_gazebo import acceptance_stats as a

PACKAGE_ROOT = Path(__file__).resolve().parents[1]  # ros2_ws/src/dog_gazebo


# ---------- helpers: walk_check --trace style dicts

def _maneuver(name, **values):
    rec = {'name': name, 'ok': True, 'detail': '%s ok' % name,
           'cmd': [0.0, -0.1, 0.0], 'seconds': 5.0, 'dx': 0.0, 'dy': 0.0,
           'dyaw_deg': -3.0, 'dyaw5_deg': -3.0, 'ratio': 0.6,
           'tilt_deg': 5.0, 'z': 0.15}
    rec.update(values)
    return rec


def _data(ratio=0.6, dyaw5=-3.0, tilt=5.0, z=0.15,
          names=('backward', 'left', 'right'), stand_ok=True, stand_state='stand'):
    """A walk_check --trace dict: the stand check plus the given manoeuvres."""
    results = [{'name': 'stand', 'ok': stand_ok, 'detail': 'state=%s z=0.150' % stand_state,
                'z': 0.15}]
    for name in names:
        results.append(_maneuver(name, ratio=ratio, dyaw5_deg=dyaw5, tilt_deg=tilt, z=z))
    return {'terrain': 'flat', 'level': 0.0, 'results': results}


def _rec(data, name):
    return next(r for r in data['results'] if r['name'] == name)


def _run(status='ok', ratio=0.6, tilt=5.0, z=0.15, wall=30.0, dyaw5=None):
    """One runs[] entry as run_record produces it."""
    if dyaw5 is None:
        dyaw5 = {'backward': -3.0, 'left': 2.0, 'right': -1.0}
    return {'status': status, 'ratio': ratio,
            'ratios': {} if ratio is None else {'backward': ratio},
            'dyaw5_deg': dict(dyaw5), 'tilt_deg': tilt, 'z': z, 'wall_s': wall}


# ---------- tracer: JSON -> runs[] -> summary -> threshold

def test_tracer_five_repeats_to_threshold():
    runs = [a.run_record(_data(ratio=r)) for r in (0.52, 0.6, 0.66, 0.7, 0.8)]
    s = a.summarize_cell('flat_A_bwd10', runs, 'jazzy')
    assert s['n'] == 5 and s['n_invalid'] == 0
    assert s['ratio_min'] == pytest.approx(0.52)
    assert s['ratio_median'] == pytest.approx(0.66)
    assert s['pass'] is True
    assert a.derive_push_threshold(s['ratio_min']) == 0.40


# ---------- classify_run

def test_classify_ok():
    assert a.classify_run(_data(0.6)) == 'ok'


def test_classify_fell_skipped_record():
    data = _data(0.6)
    _rec(data, 'left').update({'ok': False, 'detail': 'skipped: robot has fallen',
                               'fallen': True})
    assert a.classify_run(data) == 'fell'


def test_classify_fell_by_tilt():
    data = _data(0.6)
    _rec(data, 'right')['tilt_deg'] = 75.0  # a fall in the last manoeuvre, no flag
    assert a.classify_run(data) == 'fell'


def test_classify_no_stand_matches_never_stood():
    data = _data(0.6, stand_ok=False, stand_state='passive')
    assert a.classify_run(data) == 'no_stand'
    assert a.never_stood(data)


def test_classify_stand_failed_not_no_stand():
    data = _data(0.6, stand_ok=False, stand_state='lying')
    assert a.classify_run(data) != 'no_stand'


@pytest.mark.parametrize('bad', [
    {}, None, {'results': []}, {'error': 'x', 'results': []}, {'results': ['x']},
    {'results': [{'ok': True}]},
])
def test_classify_error_inputs(bad):
    assert a.classify_run(bad) == 'error'


# ---------- run_record

def test_run_record_ratio_and_ratios():
    rec = a.run_record(_data(0.66), wall_s=31.5)
    assert rec['status'] == 'ok'
    assert rec['ratio'] == pytest.approx(0.66)
    assert set(rec['ratios']) == {'backward', 'left', 'right'}
    assert all(v == pytest.approx(0.66) for v in rec['ratios'].values())
    assert rec['wall_s'] == pytest.approx(31.5)


def test_run_record_dyaw5_dict():
    data = _data(0.6)
    _rec(data, 'left')['dyaw5_deg'] = 7.5
    del _rec(data, 'right')['dyaw5_deg']
    rec = a.run_record(data)
    assert set(rec['dyaw5_deg']) == {'backward', 'left'}
    assert rec['dyaw5_deg']['backward'] == pytest.approx(-3.0)
    assert rec['dyaw5_deg']['left'] == pytest.approx(7.5)


def test_run_record_worst_tilt_and_min_z():
    data = _data(0.6)
    _rec(data, 'backward')['tilt_deg'] = 9.0
    _rec(data, 'left')['tilt_deg'] = 12.0
    _rec(data, 'right')['z'] = 0.141
    data['results'][0]['z'] = 0.199  # stand must not take part
    data['results'].append({'name': 'lie', 'ok': True, 'detail': 'state=lying z=0.02',
                            'z': 0.02})
    rec = a.run_record(data)
    assert rec['tilt_deg'] == pytest.approx(12.0)
    assert rec['z'] == pytest.approx(0.141)


def test_run_record_non_numeric_to_none():
    data = _data(0.6)
    _rec(data, 'backward')['ratio'] = '0.6'
    _rec(data, 'left')['dyaw5_deg'] = True
    _rec(data, 'right')['z'] = float('nan')
    _rec(data, 'right')['tilt_deg'] = None
    rec = a.run_record(data)
    assert rec['ratio'] is None
    assert rec['ratios']['backward'] is None
    assert rec['dyaw5_deg']['left'] is None
    assert rec['z'] == pytest.approx(0.15)
    assert rec['tilt_deg'] == pytest.approx(5.0)
    only_stand = a.run_record({'results': [{'name': 'stand', 'ok': True, 'detail': 'x',
                                            'z': 0.15}]})
    assert only_stand['ratio'] is None and only_stand['ratios'] == {}
    assert only_stand['tilt_deg'] is None and only_stand['z'] is None


# ---------- summary: n, min/median, falls, the D-01/D-07 rules

def test_min_and_median():
    runs = [_run(ratio=r) for r in (0.52, 0.6, 0.66, 0.7, 0.8)]
    s = a.summarize_cell('flat_A_bwd10', runs, 'jazzy')
    assert s['ratio_min'] == pytest.approx(0.52)
    assert s['ratio_median'] == pytest.approx(0.66)


def test_falls_no_stand_error_counted_separately():
    runs = ([_run() for _ in range(4)] + [_run(status='fell')]
            + [_run(status='no_stand', ratio=None) for _ in range(2)]
            + [_run(status='error', ratio=None)])
    s = a.summarize_cell('flat_A_bwd10', runs, 'jazzy')
    assert (s['n'], s['n_invalid'], s['falls']) == (5, 3, 1)
    assert s['pass'] is False
    assert 'falls=1' in s['reasons']


def test_fewer_than_five():
    good = [_run(ratio=r) for r in (0.5, 0.6, 0.7, 0.8)]
    s = a.summarize_cell('flat_A_bwd10', good, 'jazzy')
    assert s['pass'] is None
    assert s['reasons'][0].startswith('insufficient data')
    mixed = [_run() for _ in range(3)] + [_run(status='no_stand', ratio=None) for _ in range(2)]
    s2 = a.summarize_cell('flat_A_bwd10', mixed, 'jazzy')
    assert s2['n'] == 3 and s2['pass'] is None


def test_rule_a_jazzy_boundaries():
    edge = [_run(ratio=0.40, dyaw5={'backward': 10.0, 'left': 10.0, 'right': 10.0})
            for _ in range(5)]
    s = a.summarize_cell('flat_A_bwd10', edge, 'jazzy')
    assert s['pass'] is True and s['reasons'] == []
    low = [_run(ratio=0.3999) for _ in range(5)]
    assert a.summarize_cell('flat_A_bwd10', low, 'jazzy')['pass'] is False
    veer = ([_run(ratio=0.6) for _ in range(4)]
            + [_run(ratio=0.6, dyaw5={'backward': -3.0, 'left': 10.01, 'right': -3.0})])
    assert a.summarize_cell('flat_A_bwd10', veer, 'jazzy')['pass'] is False


def test_missing_data_fails():
    runs = [_run(ratio=0.6) for _ in range(5)]
    runs[2]['dyaw5_deg'] = {'backward': -3.0, 'left': 2.0}  # right has no dyaw5
    s = a.summarize_cell('flat_A_bwd10', runs, 'jazzy')
    assert s['pass'] is False
    assert 'dyaw5 missing in 1 case(s)' in s['reasons']
    runs = [_run(ratio=0.6) for _ in range(5)]
    for r in runs:
        r['ratio'] = None  # the backward manoeuvre ratio is missing
        r['ratios'] = {'left': 0.6, 'right': 0.6}
    s = a.summarize_cell('flat_A_bwd10', runs, 'jazzy')
    assert s['pass'] is False
    assert 'backward ratio missing in 5 run(s)' in s['reasons']


def test_rule_b():
    runs = [_run(ratio=0.6, dyaw5={'backward': 25.0, 'left': 25.0, 'right': -25.0})
            for _ in range(5)]
    s = a.summarize_cell('flat_B_bwd10', runs, 'jazzy')
    assert s['pass'] is True
    assert s['max_abs_dyaw5_deg'] == pytest.approx(25.0)
    low = [_run(ratio=0.39) for _ in range(5)]
    assert a.summarize_cell('flat_B_bwd10', low, 'jazzy')['pass'] is False


def test_report_cell():
    runs = [_run(ratio=r) for r in (0.1, 0.2, 0.3, 0.4, 0.5)]
    s = a.summarize_cell('flat_B_bwd05', runs, 'jazzy')
    assert s['pass'] is None
    assert s['reasons'] == ['report only: not scored']
    assert s['ratio_min'] == pytest.approx(0.1)
    assert s['ratio_median'] == pytest.approx(0.3)


def test_terrain_cells():
    for name in ('waves10_A_bwd10', 'rocks10_A_bwd10'):
        assert a.summarize_cell(name, [_run(ratio=0.10) for _ in range(5)], 'jazzy')['pass'] is True
        tilt = [_run(ratio=0.10) for _ in range(5)]
        tilt[3]['tilt_deg'] = 20.0  # must be strictly below
        assert a.summarize_cell(name, tilt, 'jazzy')['pass'] is False
        z = [_run(ratio=0.10) for _ in range(5)]
        z[1]['z'] = 0.108  # must be strictly above
        assert a.summarize_cell(name, z, 'jazzy')['pass'] is False
        fall = [_run(ratio=0.10) for _ in range(4)] + [_run(status='fell', ratio=0.10)]
        assert a.summarize_cell(name, fall, 'jazzy')['pass'] is False


def test_lyrical():
    runs = [_run(ratio=0.25, dyaw5={'backward': 25.0, 'left': 25.0, 'right': -25.0})
            for _ in range(5)]
    assert a.summarize_cell('flat_A_bwd10', runs, 'lyrical')['pass'] is True
    assert a.summarize_cell('flat_A_bwd10', runs, 'jazzy')['pass'] is False  # 25 % < 40 %
    low = [_run(ratio=0.19) for _ in range(5)]
    assert a.summarize_cell('flat_A_bwd10', low, 'lyrical')['pass'] is False
    floored = [_run(ratio=0.25) for _ in range(5)]
    assert a.summarize_cell('flat_A_bwd10', floored, 'lyrical', floor_ratio=0.3)['pass'] is False
    fall = [_run(ratio=0.5) for _ in range(4)] + [_run(status='fell', ratio=0.5)]
    assert a.summarize_cell('flat_A_bwd10', fall, 'lyrical')['pass'] is False
    terrain = [_run(ratio=0.1) for _ in range(5)]
    assert a.summarize_cell('waves10_A_bwd10', terrain, 'lyrical')['pass'] is True


def test_unknown_distro_cell_status_raise():
    runs = [_run() for _ in range(5)]
    with pytest.raises(ValueError):
        a.summarize_cell('flat_X', runs, 'jazzy')
    with pytest.raises(ValueError):
        a.summarize_cell('flat_A_bwd10', runs, 'foxy')
    with pytest.raises(ValueError):
        a.summarize_cell('flat_A_bwd10', [{'status': 'weird'}], 'jazzy')
    with pytest.raises(ValueError):
        a.summarize_cell('flat_A_bwd10', 'not a list', 'jazzy')
    with pytest.raises(ValueError):
        a.summarize_cell('flat_A_bwd10', runs, 'jazzy', floor_ratio='0.2')


def test_cells_follow_d03():
    assert list(a.CELLS) == ['flat_A_bwd10', 'flat_B_bwd10', 'flat_B_bwd05',
                             'waves10_A_bwd10', 'rocks10_A_bwd10']
    fields = {'terrain', 'level', 'mode', 'cmd_vx', 'maneuvers', 'rule'}
    assert all(set(a.CELLS[n]) == fields for n in a.CELLS)
    assert a.CELLS['flat_A_bwd10']['cmd_vx'] == -0.10
    assert a.CELLS['flat_B_bwd10']['cmd_vx'] == -0.10
    assert a.CELLS['flat_B_bwd05']['cmd_vx'] == -0.05
    assert a.CELLS['waves10_A_bwd10']['cmd_vx'] == -0.10
    assert a.CELLS['rocks10_A_bwd10']['cmd_vx'] == -0.10
    assert (a.CELLS['flat_A_bwd10']['mode'], a.CELLS['flat_B_bwd10']['mode'],
            a.CELLS['flat_B_bwd05']['mode']) == ('A', 'B', 'B')
    assert a.CELLS['flat_A_bwd10']['maneuvers'] == ('backward', 'left', 'right')
    assert a.CELLS['flat_B_bwd05']['maneuvers'] == ('backward',)
    assert (a.CELLS['waves10_A_bwd10']['terrain'], a.CELLS['rocks10_A_bwd10']['terrain']) == ('waves', 'rough')
    assert (a.CELLS['waves10_A_bwd10']['level'], a.CELLS['rocks10_A_bwd10']['level']) == (10, 10)
    assert (a.CELLS['flat_A_bwd10']['rule'], a.CELLS['flat_B_bwd10']['rule'],
            a.CELLS['flat_B_bwd05']['rule']) == ('score_a', 'score_b', 'report')
    assert (a.CELLS['waves10_A_bwd10']['rule'], a.CELLS['rocks10_A_bwd10']['rule']) == ('terrain', 'terrain')


# ---------- derive_push_threshold (D-05)

@pytest.mark.parametrize('value, expected', [
    (0.52, 0.40), (0.45, 0.35), (0.30, 0.20),
    (0.5, 0.40), (0.4375, 0.35), (0.499, 0.35),
    (0.35, 0.25), (0.25, 0.20), (0.2, 0.20), (0.0, 0.20), (-0.1, 0.20), (0.9, 0.40),
])
def test_derive_push_threshold_examples(value, expected):
    assert a.derive_push_threshold(value) == pytest.approx(expected)


def test_derive_push_threshold_custom_limits():
    assert a.derive_push_threshold(0.52, lo=0.1, hi=0.3) == pytest.approx(0.30)
    assert a.derive_push_threshold(0.52, margin=0.5) == pytest.approx(0.25)
    assert a.derive_push_threshold(0.45, step=0.1) == pytest.approx(0.30)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf'), '0.5', True, None])
def test_derive_push_threshold_rejects_non_finite(bad):
    with pytest.raises(ValueError):
        a.derive_push_threshold(bad)


# ---------- guard rails: constants and purity

def test_constants_match_walk_check():
    src = (PACKAGE_ROOT / 'dog_gazebo' / 'walk_check.py').read_text()
    m = re.search(r'^MIN_BODY_HEIGHT = ([\d.]+)', src, re.M)
    assert m and float(m.group(1)) == a.MIN_BODY_HEIGHT
    t = re.search(r'if worst_tilt > ([\d.]+):', src)
    assert t and float(t.group(1)) == a.FALL_TILT_DEG


def test_module_is_pure():
    src = (PACKAGE_ROOT / 'dog_gazebo' / 'acceptance_stats.py').read_text()
    tree = ast.parse(src)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
    assert not any(m == 'rclpy' or m.startswith('rclpy.') or m == 'numpy'
                   or m == 'walk_check' or m.endswith('.walk_check') for m in imported)
    assert 'dog_gazebo.terrain_sweep' in imported
