"""The replay's arithmetic is testable without ROS: phases, indices and the verdict (TEL-05).

The ROS half only publishes and records; everything that could be wrong about *what* the
drill means is here.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import joy_replay as jr  # noqa: E402


def test_every_scenario_has_phases_with_expectations():
    for name, phases in jr.scenarios().items():
        assert phases, name
        for phase in phases:
            assert phase.expect in ('moving', 'stopped', 'unset'), (name, phase)
            assert phase.seconds > 0.5, (name, phase)


def test_the_drop_scenarios_actually_go_quiet():
    for name in ('drop', 'return', 'stuck', 'stuck_held'):
        assert any(p.silence for p in jr.scenarios()[name]), name


def test_a_stuck_stick_is_not_allowed_to_move_the_robot_alone():
    """The point of TEL-05: a deflected stick with no deadman must not drive."""
    stuck = jr.scenarios()['stuck']
    assert stuck[-1].expect == 'stopped'
    assert stuck[-1].axes.get('vx', 0) > 0.5      # the stick really is hard over
    assert not stuck[-1].buttons.get('deadman')   # and the deadman is not held


def test_a_stick_and_deadman_held_on_return_is_operator_intent():
    """The honest limit: a pad that reports both held is asking for motion."""
    phases = jr.scenarios()['stuck_held']
    assert phases[-1].expect == 'moving'
    assert phases[-1].buttons.get('deadman') == 1


def test_indices_follow_the_xbox_layout():
    layout = dict(jr.DEFAULT)
    phase = jr.Phase('x', 1.0, 'moving', axes={'vx': 1.0, 'wz': -0.5},
                     buttons={'deadman': 1})
    axes, buttons = phase.joy(layout)
    assert axes[1] == 1.0 and axes[3] == -0.5
    assert buttons[4] == 1
    assert len(axes) == layout['axes_count'] and len(buttons) == layout['deadman_count']


def test_timeline_and_phase_lookup():
    phases = jr.scenarios()['return']
    line = jr.timeline(phases)
    assert line[0][0] == 0.0
    assert line[-1][1] == sum(p.seconds for p in phases)
    assert jr.phase_at(phases, 0.1) is phases[0]
    assert jr.phase_at(phases, line[1][0] + 0.1) is phases[1]
    assert jr.phase_at(phases, 10_000.0) is None


def test_classify_reads_any_axis_and_ignores_noise():
    assert jr.classify([0.0, 0.0, 0.0]) == 'stopped'
    assert jr.classify([0.001, -0.002, 0.003]) == 'stopped'
    assert jr.classify([0.0, 0.0, 0.2]) == 'moving'      # a turn is motion too


def test_a_robot_that_stops_in_time_passes():
    phases = jr.scenarios()['drop']
    # drive to 1.5 s, then everything is zero after the stream died
    samples = [(t / 10.0, [0.05, 0.0, 0.0] if t < 15 else [0.0, 0.0, 0.0])
               for t in range(0, 28)]
    rows = jr.judge(phases, samples)
    assert all(ok for _, ok, _ in rows), rows


def test_a_robot_that_keeps_going_fails_loudly():
    phases = jr.scenarios()['drop']
    samples = [(t / 10.0, [0.05, 0.0, 0.0]) for t in range(0, 28)]
    rows = jr.judge(phases, samples)
    assert not rows[-1][1]
    assert 'ждали stopped' in rows[-1][2]
    assert 'ПЛОХО' in jr.render(rows)


def test_a_phase_without_data_is_not_a_pass():
    rows = jr.judge(jr.scenarios()['drive'], [])
    # The opening phase expects exactly that - no pad, no command - and passes on it;
    # the driving phase that follows does not.
    assert rows[0][1], rows
    assert not any(ok for _, ok, _ in rows[1:]), rows
    assert any('no data' in note for _, _, note in rows)


def test_list_and_unknown_scenario_through_the_cli(capsys):
    assert jr.main(['--list']) == 0
    out = capsys.readouterr().out
    for name in jr.scenarios():
        assert name in out
    assert jr.main(['--only', 'nope']) == 2
    assert 'нет такого сценария' in capsys.readouterr().err


def test_a_steady_command_is_not_absence_of_data():
    """joy_teleop publishes on change: one message, then silence, is a held command."""
    phases = jr.scenarios()['drive']
    samples = [(0.1, [0.0, 0.0, 0.0]), (0.9, [0.05, 0.0, 0.0])]
    rows = jr.judge(phases, samples)
    assert rows[-1][1], rows


def test_last_at_holds_the_previous_value():
    samples = [(1.0, [0.1, 0.0, 0.0]), (2.0, [0.0, 0.0, 0.0])]
    assert jr.last_at(samples, 0.5) is None
    assert jr.last_at(samples, 1.5) == [0.1, 0.0, 0.0]
    assert jr.last_at(samples, 5.0) == [0.0, 0.0, 0.0]
    assert jr.last_at([], 1.0) is None
