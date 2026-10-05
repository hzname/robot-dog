"""Pure-pytest checks of dog_gazebo.acceptance: the argument surface, the cell
launch/walk_check argument builders, the repeat plan and --dry-run - no ROS,
no Gazebo; the execution path is exercised with a fake runner (D-03, D-24)."""

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
