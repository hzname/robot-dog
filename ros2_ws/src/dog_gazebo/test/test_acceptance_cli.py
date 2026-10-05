"""Pure-pytest checks of dog_gazebo.acceptance: the argument surface, the cell
launch/walk_check argument builders, the repeat plan and --dry-run - no ROS,
no Gazebo; the execution path is exercised with a fake runner (D-03, D-24)."""

import ast
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from dog_gazebo import acceptance
from dog_gazebo import acceptance_stats as stats
from dog_gazebo import walk_check_args

PACKAGE_ROOT = Path(__file__).resolve().parents[1]  # ros2_ws/src/dog_gazebo
ENV = {'ROS_DISTRO': 'jazzy'}


def _boom(*args, **kwargs):
    raise AssertionError('the runner must not be called')


# ---------- parse_args

def test_parse_defaults():
    args = acceptance.parse_args([], environ=dict(ENV, GITHUB_SHA='abc'))
    assert args.repeats == 5 == acceptance.DEFAULT_REPEATS
    assert tuple(args.cells) == tuple(stats.CELLS)
    assert args.servo_model == 'ideal'
    assert args.domain == acceptance.DOMAIN_BASE == 80
    assert args.floor_ratio == stats.DEFAULT_FLOOR_RATIO == 0.2
    assert args.out == 'acceptance_jazzy_ideal.json'
    assert args.summary == 'acceptance_jazzy_ideal_summary.md'
    assert args.git_sha == 'abc'
    assert args.strict is False and args.dry_run is False


@pytest.mark.parametrize('argv, environ', [
    (['--repeats', '0'], ENV),
    (['--repeats', '21'], ENV),
    (['--distro', 'humble'], ENV),
    (['--servo-model', 'foo'], ENV),
    (['--cells', 'nope'], ENV),
    (['--cells', ''], ENV),
    (['--cells', 'flat_A_bwd10,flat_A_bwd10'], ENV),
    (['--cells', 'all,flat_A_bwd10'], ENV),
    (['--domain', '-1'], ENV),
    (['--domain', '93'], ENV),
    (['--floor-ratio', '0'], ENV),
    (['--floor-ratio', 'nan'], ENV),
    (['--floor-ratio', '1.5'], ENV),
])
def test_parse_rejects(argv, environ):
    with pytest.raises(SystemExit) as exc:
        acceptance.parse_args(argv, environ=environ)
    assert exc.value.code == 2


def test_parse_missing_ros_distro(capsys):
    with pytest.raises(SystemExit) as exc:
        acceptance.parse_args([], environ={})
    assert exc.value.code == 2
    assert 'ROS_DISTRO' in capsys.readouterr().err


# ---------- cell argument builders

def test_cell_launch_args():
    assert acceptance.cell_launch_args('flat_A_bwd10', 'ideal') == []
    assert acceptance.cell_launch_args('flat_B_bwd10', 'ideal') == [
        'heading_hold:=false', 'slope_compensation:=false']
    assert acceptance.cell_launch_args('flat_A_bwd10', 'real') == ['servo_model:=real']
    assert acceptance.cell_launch_args('flat_B_bwd05', 'real') == [
        'heading_hold:=false', 'slope_compensation:=false', 'servo_model:=real']
    with pytest.raises(ValueError):
        acceptance.cell_launch_args('nope', 'ideal')
    with pytest.raises(ValueError):
        acceptance.cell_launch_args('flat_A_bwd10', 'nope')


def test_cell_walk_check_args():
    assert acceptance.cell_walk_check_args('flat_A_bwd10') == [
        '--maneuvers', 'backward,left,right', '--backward-speed', '0.1',
        '--min-ratio', '0', '--backward-ratio', '0']
    assert acceptance.cell_walk_check_args('flat_B_bwd05') == [
        '--maneuvers', 'backward', '--backward-speed', '0.05',
        '--min-ratio', '0', '--backward-ratio', '0']
    # the same tail parses through the walk_check argument module
    for name in stats.CELLS:
        parsed, ros = walk_check_args.parse_args(acceptance.cell_walk_check_args(name))
        assert ros == []
        assert parsed.maneuvers == stats.CELLS[name]['maneuvers']
        assert parsed.backward_speed == pytest.approx(abs(stats.CELLS[name]['cmd_vx']))
        assert parsed.min_ratio == 0 and parsed.backward_ratio == 0


# ---------- repeat plan and domains

def test_plan_runs():
    runs = acceptance.plan_runs(tuple(stats.CELLS), 3)
    assert len(runs) == 15
    assert [r['cell'] for r in runs] == [name for name in stats.CELLS for _ in range(3)]
    assert [r['seed'] for r in runs if r['cell'] == 'rocks10_A_bwd10'] == [1, 2, 3]
    assert all(r['seed'] == 0 for r in runs if r['cell'] != 'rocks10_A_bwd10')
    assert acceptance.plan_runs(('flat_B_bwd05',), 2) == [
        {'cell': 'flat_B_bwd05', 'repeat': 1, 'seed': 0},
        {'cell': 'flat_B_bwd05', 'repeat': 2, 'seed': 0}]


def test_domain_for():
    domains = [acceptance.domain_for(80, i) for i in range(25)]
    assert domains == [80 + i % 10 for i in range(25)]
    assert domains[:10] == list(range(80, 90))
    assert all(d not in range(41, 44) and d not in range(60, 78) for d in domains)


# ---------- --dry-run

def test_dry_run_prints_plan(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    code = acceptance.main(['--dry-run', '--distro', 'jazzy', '--cells', 'flat_B_bwd05',
                            '--repeats', '2'], runner=_boom, environ={})
    out = capsys.readouterr().out
    assert code == 0
    lines = [ln for ln in out.splitlines() if ln.startswith('DRY-RUN ')]
    assert len(lines) == 2
    assert 'domain=80' in lines[0] and 'domain=81' in lines[1]
    assert 'heading_hold:=false' in lines[0]
    assert '--maneuvers backward --backward-speed 0.05' in lines[0]
    assert 'planned: 2 simulations' in out
    assert not list(tmp_path.iterdir())


def test_dry_run_all_cells(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    code = acceptance.main(['--dry-run', '--distro', 'jazzy', '--repeats', '5'],
                           runner=_boom, environ={})
    out = capsys.readouterr().out
    assert code == 0
    assert len([ln for ln in out.splitlines() if ln.startswith('DRY-RUN ')]) == 25
    assert 'planned: 25 simulations' in out


# ---------- execution with a fake runner

def _clock(step=10.0):
    state = {'t': 0.0}

    def clock():
        value = state['t']
        state['t'] += step
        return value
    return clock


def _walk(ratio, dyaw5=2.0):
    """A walk_check --trace dict: stand plus the three scoring maneuvers."""
    results = [{'name': 'stand', 'ok': True, 'detail': 'state=stand z=0.150', 'z': 0.15}]
    for name in ('backward', 'left', 'right'):
        results.append({'name': name, 'ok': True, 'detail': 'ok', 'ratio': ratio,
                        'dyaw5_deg': dyaw5, 'tilt_deg': 5.0, 'z': 0.15})
    return {'terrain': 'flat', 'level': 0.0, 'results': results}


def _never_stood():
    return {'terrain': 'flat', 'level': 0.0,
            'results': [{'name': 'stand', 'ok': False, 'detail': 'state=passive z=0.150',
                         'z': 0.15}]}


def _error():
    return {'terrain': 'flat', 'level': 0.0, 'error': 'timeout', 'results': []}


def _fell():
    return {'terrain': 'flat', 'level': 0.0,
            'results': [{'name': 'stand', 'ok': True, 'detail': 'state=stand z=0.150', 'z': 0.15},
                        {'name': 'backward', 'ok': False,
                         'detail': 'skipped: robot has fallen', 'fallen': True,
                         'tilt_deg': 75.0, 'z': 0.15}]}


class _Runner:
    """A scripted runner: records every call and refuses overlapping calls."""

    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []
        self.busy = False

    def __call__(self, kind, level, seed, domain, extra, launch_args=(), keep=None, sim_log=None):
        assert not self.busy, 'runner called while the previous call is running'
        self.busy = True
        try:
            self.calls.append({'kind': kind, 'level': level, 'seed': seed, 'domain': domain,
                               'extra': list(extra), 'launch_args': list(launch_args),
                               'keep': keep, 'sim_log': sim_log})
            return self.responses.pop(0)
        finally:
            self.busy = False


def _read(path):
    with open(path) as handle:
        return json.load(handle)


def test_flat_a_five_repeats_to_verdict(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    ratios = [0.52, 0.60, 0.66, 0.70, 0.80]
    runner = _Runner([_walk(r) for r in ratios])
    code = acceptance.main(['--cells', 'flat_A_bwd10', '--repeats', '5', '--distro', 'jazzy',
                            '--git-sha', 'abc', '--out', 'acc.json'],
                           runner=runner, clock=_clock(), environ={'GITHUB_ACTIONS': 'true'})
    assert code == 0
    assert len(runner.calls) == 5
    data = _read('acc.json')
    assert data['schema'] == 1 and data['git_sha'] == 'abc'
    assert data['verdict']['pass'] is True
    cell = data['cells']['flat_A_bwd10']
    assert [run['ratio'] for run in cell['runs']] == pytest.approx(ratios)
    assert cell['runs'][0]['wall_s'] == 10.0
    assert cell['summary']['n'] == 5
    summary = Path('acc_summary.md').read_text()
    assert summary.startswith('### Backward acceptance: jazzy / ideal (scoring)')
    assert not list(tmp_path.glob('*.tmp'))


def test_strict_reads_verdict(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    argv = ['--cells', 'flat_A_bwd10', '--repeats', '5', '--distro', 'jazzy', '--out', 'acc.json']
    code = acceptance.main(argv, runner=_Runner([_walk(0.30)] * 5), clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 0                      # n == 5: the verdict fails, nothing is broken
    code = acceptance.main(argv + ['--strict'], runner=_Runner([_walk(0.30)] * 5), clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 1
    one = ['--cells', 'flat_A_bwd10', '--repeats', '1', '--distro', 'jazzy', '--out', 'one.json']
    code = acceptance.main(one, runner=_Runner([_walk(0.70)]), clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 0
    assert _read('one.json')['verdict']['pass'] is False
    code = acceptance.main(one + ['--strict'], runner=_Runner([_walk(0.70)]), clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 1


def test_runner_arguments(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    runner = _Runner([_walk(0.6) for _ in range(25)])
    code = acceptance.main(['--cells', 'all', '--repeats', '5', '--distro', 'jazzy',
                            '--servo-model', 'real', '--out', 'acc.json'],
                           runner=runner, clock=_clock(), environ={'GITHUB_ACTIONS': 'true'})
    assert code == 0
    assert len(runner.calls) == 25
    for call, record in zip(runner.calls, acceptance.plan_runs(tuple(stats.CELLS), 5)):
        name = record['cell']
        cell = stats.CELLS[name]
        assert call['kind'] == cell['terrain'] and call['level'] == cell['level']
        assert call['seed'] == record['seed']
        assert call['extra'] == acceptance.cell_walk_check_args(name)
        assert call['launch_args'] == acceptance.cell_launch_args(name, 'real')
        assert call['keep'] is None
    assert [call['domain'] for call in runner.calls] == [80 + i % 10 for i in range(25)]
    logs = [call['sim_log'] for call in runner.calls]
    assert logs[0] == 'acc_flat_A_bwd10_r01.sim.log'
    assert logs[20] == 'acc_rocks10_A_bwd10_r01.sim.log'
    assert [call['seed'] for call in runner.calls[20:]] == [1, 2, 3, 4, 5]


def test_incremental_flush(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    seen = []

    class Runner:
        def __call__(self, *args, **kwargs):
            count = 0
            if os.path.exists('acc.json'):
                data = _read('acc.json')
                count = sum(len(cell['runs']) for cell in data['cells'].values())
            seen.append(count)
            return _walk(0.6)

    code = acceptance.main(['--cells', 'flat_A_bwd10', '--repeats', '2', '--distro', 'jazzy',
                            '--out', 'acc.json'], runner=Runner(), clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 0
    assert seen == [0, 1]                 # at call k the file holds k-1 records
    assert _read('acc.json')['cells']['flat_A_bwd10']['summary']['n'] == 2


def test_relaunch_once(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    runner = _Runner([_never_stood(), _walk(0.6)])
    code = acceptance.main(['--cells', 'flat_A_bwd10', '--repeats', '1', '--distro', 'jazzy',
                            '--out', 'acc.json'], runner=runner, clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 0
    assert len(runner.calls) == 2
    assert runner.calls[0]['sim_log'] == 'acc_flat_A_bwd10_r01.sim.log'
    assert runner.calls[1]['sim_log'] == 'acc_flat_A_bwd10_r01.retry.sim.log'
    assert _read('acc.json')['cells']['flat_A_bwd10']['runs'][0]['status'] == 'ok'
    # both never_stood in a row: no second relaunch, the attempt is no_stand
    runner2 = _Runner([_never_stood() for _ in range(4)])
    code = acceptance.main(['--cells', 'flat_A_bwd10', '--repeats', '1', '--distro', 'jazzy',
                            '--out', 'acc2.json'], runner=runner2, clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 1
    assert len(runner2.calls) == 4
    runs = _read('acc2.json')['cells']['flat_A_bwd10']['runs']
    assert [run['status'] for run in runs] == ['no_stand', 'no_stand']


def test_no_stand_replaced_up_to_2n(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    runner = _Runner([_never_stood() for _ in range(4)] + [_walk(0.6) for _ in range(5)])
    code = acceptance.main(['--cells', 'flat_A_bwd10', '--repeats', '5', '--distro', 'jazzy',
                            '--out', 'acc.json'], runner=runner, clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 0
    assert len(runner.calls) == 9
    cell = _read('acc.json')['cells']['flat_A_bwd10']
    assert len(cell['runs']) == 7
    assert cell['summary']['n'] == 5 and cell['summary']['n_invalid'] == 2
    # all no_stand: the replacements stop at 2*n attempts
    runner2 = _Runner([_never_stood() for _ in range(20)])
    code = acceptance.main(['--cells', 'flat_A_bwd10', '--repeats', '5', '--distro', 'jazzy',
                            '--out', 'acc2.json'], runner=runner2, clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 1
    assert len(runner2.calls) == 20
    cell = _read('acc2.json')['cells']['flat_A_bwd10']
    assert len(cell['runs']) == 10 and cell['summary']['n'] == 0


def test_error_is_not_replaced(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    runner = _Runner([_error() for _ in range(5)])
    code = acceptance.main(['--cells', 'flat_A_bwd10', '--repeats', '5', '--distro', 'jazzy',
                            '--out', 'acc.json'], runner=runner, clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 1
    assert len(runner.calls) == 5         # an error is not replaced: n attempts, not 2*n
    cell = _read('acc.json')['cells']['flat_A_bwd10']
    assert [run['status'] for run in cell['runs']] == ['error'] * 5
    assert cell['summary']['n'] == 0


def test_fell_counts_as_valid(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    code = acceptance.main(['--cells', 'flat_A_bwd10', '--repeats', '1', '--distro', 'jazzy',
                            '--out', 'acc.json'], runner=_Runner([_fell()]), clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 0
    cell = _read('acc.json')['cells']['flat_A_bwd10']
    assert cell['runs'][0]['status'] == 'fell'
    assert cell['summary']['n'] == 1 and cell['summary']['n_invalid'] == 0
    assert cell['summary']['falls'] == 1


def test_local_warning(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    code = acceptance.main(['--dry-run', '--distro', 'jazzy'], runner=_boom, environ={})
    assert code == 0
    assert 'GitHub Actions' not in capsys.readouterr().err
    argv = ['--cells', 'flat_A_bwd10', '--repeats', '1', '--distro', 'jazzy', '--out', 'acc.json']
    code = acceptance.main(argv, runner=_Runner([_walk(0.6)]), clock=_clock(), environ={})
    assert code == 0
    assert 'GitHub Actions' in capsys.readouterr().err
    code = acceptance.main(argv, runner=_Runner([_walk(0.6)]), clock=_clock(),
                           environ={'GITHUB_ACTIONS': 'true'})
    assert code == 0
    assert 'GitHub Actions' not in capsys.readouterr().err


def test_invalid_distro_never_starts():
    calls = []

    def runner(*args, **kwargs):
        calls.append(args)
        return _walk(0.6)

    with pytest.raises(SystemExit) as exc:
        acceptance.main(['--distro', 'humble'], runner=runner, environ={})
    assert exc.value.code == 2
    assert calls == []


# ---------- guard rails: purity and packaging

def test_module_is_sequential_and_pure():
    src = (PACKAGE_ROOT / 'dog_gazebo' / 'acceptance.py').read_text()
    imported = set()
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
    banned = ('rclpy', 'numpy', 'threading', 'multiprocessing', 'concurrent', 'asyncio',
              'subprocess')
    assert not any(m == b or m.startswith(b + '.') for m in imported for b in banned)
    assert not any(m == 'walk_check' or m.endswith('.walk_check') for m in imported)
    assert 'run_record(' in src


def test_packaging_declares_acceptance_entry():
    setup_py = (PACKAGE_ROOT / 'setup.py').read_text()
    assert "'acceptance = dog_gazebo.acceptance:main'," in setup_py
    assert "extras_require={'test': ['pytest']}," in setup_py


def test_module_entry_point(tmp_path):
    env = dict(os.environ, PYTHONPATH=str(PACKAGE_ROOT))
    result = subprocess.run([sys.executable, '-m', 'dog_gazebo.acceptance', '--dry-run',
                             '--distro', 'jazzy', '--cells', 'flat_B_bwd05', '--repeats', '1'],
                            capture_output=True, text=True, cwd=tmp_path, env=env)
    assert result.returncode == 0
    assert result.stdout.count('DRY-RUN ') == 1
    assert not list(tmp_path.iterdir())
