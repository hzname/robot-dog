"""Pure-pytest checks of dog_gazebo.walk_check_args and the walk_check changes:
the argument surface (old flags untouched, --maneuvers, --backward-speed), the
maneuver tables, lie_wanted and dyaw5_deg - no ROS, no Gazebo (D-01, D-02)."""

import ast
import importlib
import math
import sys
import types
from pathlib import Path

import pytest

from dog_gazebo import walk_check_args as wa

PACKAGE_ROOT = Path(__file__).resolve().parents[1]  # ros2_ws/src/dog_gazebo


# ---------- fixture: dog_gazebo.walk_check imported without rclpy

def _stub(name, **attrs):
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    return mod


class _Node:
    def create_subscription(self, *args, **kwargs):
        return None

    def create_publisher(self, *args, **kwargs):
        return None


class _QoS:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class _Policies:
    TRANSIENT_LOCAL = 'transient_local'
    RELIABLE = 'reliable'


class _String:
    def __init__(self, data=''):
        self.data = data


class _Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


@pytest.fixture
def wc_module(monkeypatch):
    """dog_gazebo.walk_check imported fresh on stub rclpy and message modules."""
    rclpy = _stub('rclpy', create_node=lambda *a, **k: _Node(), init=lambda *a, **k: None,
                  shutdown=lambda *a, **k: None, spin_once=lambda *a, **k: None)
    qos = _stub('rclpy.qos', DurabilityPolicy=_Policies, ReliabilityPolicy=_Policies,
                QoSProfile=_QoS, qos_profile_sensor_data=10)
    setattr(rclpy, 'qos', qos)
    geometry_msgs = _stub('geometry_msgs')
    geometry_msgs_msg = _stub('geometry_msgs.msg', Twist=_Twist)
    setattr(geometry_msgs, 'msg', geometry_msgs_msg)
    nav_msgs = _stub('nav_msgs')
    nav_msgs_msg = _stub('nav_msgs.msg', Odometry=object)
    setattr(nav_msgs, 'msg', nav_msgs_msg)
    sensor_msgs = _stub('sensor_msgs')
    sensor_msgs_msg = _stub('sensor_msgs.msg', Imu=object, JointState=object)
    setattr(sensor_msgs, 'msg', sensor_msgs_msg)
    std_msgs = _stub('std_msgs')
    std_msgs_msg = _stub('std_msgs.msg', String=_String)
    setattr(std_msgs, 'msg', std_msgs_msg)
    for name, mod in (('rclpy', rclpy), ('rclpy.qos', qos),
                      ('geometry_msgs', geometry_msgs), ('geometry_msgs.msg', geometry_msgs_msg),
                      ('nav_msgs', nav_msgs), ('nav_msgs.msg', nav_msgs_msg),
                      ('sensor_msgs', sensor_msgs), ('sensor_msgs.msg', sensor_msgs_msg),
                      ('std_msgs', std_msgs), ('std_msgs.msg', std_msgs_msg)):
        monkeypatch.setitem(sys.modules, name, mod)
    sys.modules.pop('dog_gazebo.walk_check', None)
    module = importlib.import_module('dog_gazebo.walk_check')
    try:
        yield module
    finally:
        sys.modules.pop('dog_gazebo.walk_check', None)


def _checker(wc, **kwargs):
    """A real WalkCheck on the stub nodes; the simulation side stays fake."""
    checker = wc.WalkCheck(**kwargs)
    checker.odom = types.SimpleNamespace(pose=types.SimpleNamespace(pose=types.SimpleNamespace(
        position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    checker.state = 'stand'
    return checker


def _simulate(checker, yaw_step=None, tilt=5.0):
    """Replace spin/height/tilt/cmd with a scripted stand-in; return the log."""
    calls = {'cmd': []}

    def spin(seconds, publish=None):
        if yaw_step is not None:
            if seconds == 5.0:
                checker.yaw_unwrapped += math.radians(8)
            elif seconds == 1.5:
                checker.yaw_unwrapped += math.radians(3)
        return tilt

    class Cmd:
        def publish(self, msg):
            calls['cmd'].append(msg.data)
            if msg.data == 'lie':
                checker.state = 'lying'

    checker.spin = spin
    checker.height = lambda: 0.02 if checker.phase == 'lie' else 0.15
    checker.tilt = lambda: tilt
    checker.cmd = Cmd()
    return calls


# ---------- tracer: --maneuvers/--backward-speed reach WalkCheck, dyaw5 first

def test_tracer_backward_005_dyaw5(wc_module):
    args, ros = wa.parse_args(['--maneuvers', 'backward', '--backward-speed', '0.05'])
    assert ros == []
    checker = _checker(wc_module, backward_speed=args.backward_speed, maneuvers=args.maneuvers,
                       min_ratio=0, backward_ratio=0)
    _simulate(checker, yaw_step=True)
    checker.maneuver('backward', -0.05, 0, 0, 5.0, ('x', -0.25))
    values = checker.results[-1][3]
    assert values['dyaw5_deg'] == pytest.approx(8.0)   # 8 deg during the command part
    assert values['dyaw_deg'] == pytest.approx(11.0)   # 11 deg after the 1.5 s coast


# ---------- parse_args

def test_parse_defaults():
    args, ros = wa.parse_args([])
    assert (args.trace, args.terrain, args.level) == (None, 'flat', 0.0)
    assert (args.min_ratio, args.backward_ratio) == (0.4, None)
    assert (args.max_tilt, args.seconds, args.record) == (20.0, 5.0, False)
    assert args.maneuvers is None
    assert args.backward_speed == wa.DEFAULT_BACKWARD_SPEED == 0.10
    assert ros == []


def test_parse_old_flags_and_ros_tail():
    args, ros = wa.parse_args(['--backward-ratio', '0.2', '--record', '--trace', 'x.json',
                               '--ros-args', '-p', 'use_sim_time:=true'])
    assert args.backward_ratio == 0.2
    assert args.record is True and args.trace == 'x.json'
    assert ros == ['--ros-args', '-p', 'use_sim_time:=true']


def test_parse_maneuvers_in_routine_order():
    args, _ = wa.parse_args(['--maneuvers', 'arc_right,backward,forward'])
    assert args.maneuvers == ('forward', 'backward', 'arc_right')


@pytest.mark.parametrize('text', ['sideways', 'backward,backward', '', 'backward,,left'])
def test_parse_maneuvers_rejects(text):
    with pytest.raises(SystemExit) as exc:
        wa.parse_args(['--maneuvers', text])
    assert exc.value.code == 2


def test_parse_backward_speed_accepts_magnitudes():
    for text in ('0.05', '0.5'):
        args, _ = wa.parse_args(['--backward-speed', text])
        assert args.backward_speed == float(text)


@pytest.mark.parametrize('text', ['0', 'nan', 'inf', '-0.05', '0.6'])
def test_parse_backward_speed_rejects(text):
    with pytest.raises(SystemExit) as exc:
        wa.parse_args(['--backward-speed', text])
    assert exc.value.code == 2


def test_parse_backward_speed_negative_hint(capsys):
    with pytest.raises(SystemExit):
        wa.parse_args(['--backward-speed', '-0.05'])
    assert 'magnitude' in capsys.readouterr().err


# ---------- maneuver tables

T = 5.0


def test_default_plan_is_golden():
    assert wa.maneuver_plan('flat', T) == (
        ('forward', 0.12, 0, 0, T, ('x', 0.12 * T)),
        ('backward', -0.10, 0, 0, T, ('x', -0.10 * T)),
        ('left', 0, 0.06, 0, T, ('y', 0.06 * T)),
        ('right', 0, -0.06, 0, T, ('y', -0.06 * T)),
        ('turn_ccw', 0, 0, 0.5, T, ('yaw', 0.5 * T)),
        ('turn_cw', 0, 0, -0.5, T, ('yaw', -0.5 * T)),
        ('arc_left', 0.10, 0, 0.3, T, ('yaw', 0.3 * T)),
        ('arc_right', 0.10, 0, -0.3, T, ('yaw', -0.3 * T)),
    )
    assert wa.maneuver_plan('slope', T) == (
        ('forward', 0.12, 0, 0, T, ('x', 0.12 * T)),
        ('left', 0, 0.06, 0, T, ('y', 0.06 * T)),
        ('right', 0, -0.06, 0, T, ('y', -0.06 * T)),
        ('turn_ccw', 0, 0, 0.5, T * 0.5, ('yaw', 0.25 * T)),
        ('turn_cw', 0, 0, -0.5, T * 0.5, ('yaw', -0.25 * T)),
        ('backward', -0.10, 0, 0, T * 1.4, ('x', -0.14 * T)),
    )


def test_plan_subset_and_speed():
    plan = wa.maneuver_plan('flat', T, 0.05, ('backward',))
    assert plan == (('backward', -0.05, 0, 0, T, ('x', -0.05 * T)),)
    plan = wa.maneuver_plan('flat', T, 0.05, ('backward', 'forward'))
    assert [row[0] for row in plan] == ['forward', 'backward']
    assert plan[0][1] == 0.12 and plan[1][1] == -0.05
    # slope ignores both flags: the old table, the old speed
    assert wa.maneuver_plan('slope', T, 0.05, ('backward',)) == wa.maneuver_plan('slope', T)


def test_lie_wanted():
    assert wa.lie_wanted('flat', None) is True
    assert wa.lie_wanted('flat', wa.FLAT_MANEUVERS) is True
    assert wa.lie_wanted('flat', ('backward',)) is False
    assert wa.lie_wanted('slope', ('backward',)) is True


# ---------- run() through the fake simulation

def test_run_full_routine_is_ten_checks(wc_module, capsys):
    checker = _checker(wc_module, min_ratio=0, backward_ratio=0)
    _simulate(checker)
    code = checker.run()
    out = capsys.readouterr().out
    assert code == 0
    assert '10/10 passed' in out
    assert [r[0] for r in checker.results] == [
        'stand', 'forward', 'backward', 'left', 'right', 'turn_ccw', 'turn_cw',
        'arc_left', 'arc_right', 'lie']


def test_run_subset_skips_lie(wc_module, capsys):
    checker = _checker(wc_module, min_ratio=0, backward_ratio=0, maneuvers=('backward',))
    calls = _simulate(checker)
    code = checker.run()
    out = capsys.readouterr().out
    assert code == 0
    assert '2/2 passed' in out
    assert [r[0] for r in checker.results] == ['stand', 'backward']
    assert 'lie' not in calls['cmd']


def test_run_slope_flags_note(wc_module, capsys):
    checker = _checker(wc_module, kind='slope', level=6.0, min_ratio=0, backward_ratio=0,
                       maneuvers=('backward',), backward_speed=0.05)
    _simulate(checker)
    code = checker.run()
    out = capsys.readouterr().out
    assert code == 0
    assert 'note: --maneuvers and --backward-speed do not apply on slope' in out
    assert [r[0] for r in checker.results] == [
        'stand', 'forward', 'left', 'right', 'turn_ccw', 'turn_cw', 'backward', 'lie']
    assert '8/8 passed' in out


def test_maneuver_skip_record_has_no_dyaw5(wc_module):
    checker = _checker(wc_module, min_ratio=0, backward_ratio=0)
    _simulate(checker, yaw_step=True)
    checker.fallen = True
    checker.maneuver('backward', -0.05, 0, 0, 5.0, ('x', -0.25))
    name, ok, detail, values = checker.results[-1]
    assert values.get('fallen') is True
    assert 'dyaw5_deg' not in values


# ---------- purity

def test_module_is_pure():
    src = (PACKAGE_ROOT / 'dog_gazebo' / 'walk_check_args.py').read_text()
    imported = set()
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
    banned = ('rclpy', 'numpy', 'geometry_msgs', 'nav_msgs', 'sensor_msgs', 'std_msgs')
    assert not any(m == b or m.startswith(b + '.') for m in imported for b in banned)
    assert not any(m == 'walk_check' or m.endswith('.walk_check') for m in imported)
    assert {'argparse', 'math'} <= imported
