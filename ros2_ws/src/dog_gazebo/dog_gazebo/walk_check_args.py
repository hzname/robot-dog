"""walk_check arguments and maneuver tables - the pure part of walk_check.

Only argparse and math: pytest runs this module on Python 3.12 and 3.14
without rclpy or numpy, and the acceptance CLI parses the same tail through
parse_args(). `--maneuvers` selects a subset of the flat routine in routine
order (the lie check is skipped for a subset), `--backward-speed` sets the
backward speed on the flat (magnitude, m/s); on slope both are ignored:

  ros2 run dog_gazebo walk_check --maneuvers backward --backward-speed 0.05
  ros2 run dog_gazebo walk_check --maneuvers backward,left,right

maneuver_plan() keeps the old run() table bit for bit, so the default routine
(10 checks on the flat: stand, 8 maneuvers, lie) and the push-CI command
`walk_check --backward-ratio 0.2` are unchanged.
"""

import argparse
import math

FLAT_MANEUVERS = ('forward', 'backward', 'left', 'right',
                  'turn_ccw', 'turn_cw', 'arc_left', 'arc_right')
SLOPE_MANEUVERS = ('forward', 'left', 'right', 'turn_ccw', 'turn_cw', 'backward')
DEFAULT_BACKWARD_SPEED = 0.10
# Sanity bound for the flag [m/s]; the flat walk itself is about 0.10.
MAX_BACKWARD_SPEED = 0.5


def parse_maneuvers(text):
    """`--maneuvers` value: names from FLAT_MANEUVERS, no duplicates and no
    empty names, returned as a tuple in routine order."""
    names = [part.strip() for part in text.split(',')]
    if not names or any(not name for name in names):
        raise argparse.ArgumentTypeError('give comma separated maneuver names')
    unknown = [name for name in names if name not in FLAT_MANEUVERS]
    if unknown:
        raise argparse.ArgumentTypeError('unknown maneuver(s) %s (known: %s)'
                                         % (', '.join(unknown), ', '.join(FLAT_MANEUVERS)))
    if len(set(names)) != len(names):
        raise argparse.ArgumentTypeError('duplicate maneuver name in %r' % (text,))
    wanted = set(names)
    return tuple(name for name in FLAT_MANEUVERS if name in wanted)


def parse_backward_speed(text):
    """`--backward-speed` value: a finite magnitude in (0, MAX_BACKWARD_SPEED]."""
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError('not a number: %r' % (text,))
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError('must be finite, got %r' % (text,))
    if value < 0:
        raise argparse.ArgumentTypeError('give the speed magnitude, e.g. 0.05 [m/s]')
    if value == 0 or value > MAX_BACKWARD_SPEED:
        raise argparse.ArgumentTypeError('must be in (0, %.1f] m/s, got %r'
                                         % (MAX_BACKWARD_SPEED, text))
    return value


def build_parser():
    """The walk_check parser: every old flag with the same name, default and
    help, plus --maneuvers and --backward-speed."""
    ap = argparse.ArgumentParser(description='Drive the simulated dog and check the odometry.')
    ap.add_argument('--trace', help='write results + odometry trace to this JSON file')
    ap.add_argument('--terrain', default='flat', choices=['flat', 'slope', 'waves', 'rough'])
    ap.add_argument('--level', type=float, default=0.0, help='slope [deg] or obstacle height [mm]')
    ap.add_argument('--min-ratio', type=float, default=0.4, help='share of the command to pass')
    ap.add_argument('--backward-ratio', type=float,
                    help='... for the backward manoeuvre (default: --min-ratio); 0 = only no fall, '
                    'no tilt, no sagging (backward on uneven ground is the weak manoeuvre, TERRAIN.md)')
    ap.add_argument('--maneuvers', type=parse_maneuvers, default=None,
                    help='comma separated subset of the flat routine in routine order '
                         '(default: all eight; a subset skips the lie check; on slope ignored)')
    ap.add_argument('--backward-speed', type=parse_backward_speed, default=DEFAULT_BACKWARD_SPEED,
                    metavar='MPS', help='backward speed magnitude [m/s] (default: %.2f; '
                    'on slope ignored)' % DEFAULT_BACKWARD_SPEED)
    ap.add_argument('--max-tilt', type=float, default=20.0, help='body tilt vs. the ground [deg]')
    ap.add_argument('--seconds', type=float, default=5.0, help='duration of each maneuver')
    ap.add_argument('--record', action='store_true',
                    help='with --trace: also record joints and IMU at 30 Hz')
    return ap


def parse_args(argv=None):
    """parse_known_args(): the ROS tail (--ros-args ...) goes back to rclpy.init."""
    return build_parser().parse_known_args(argv)


def maneuver_plan(kind, seconds, backward_speed=DEFAULT_BACKWARD_SPEED, selected=None):
    """The run() table: (name, vx, vy, wz, duration, (axis, target)).

    Flat (and waves/rough, as before) uses backward_speed and selected; slope
    keeps its own table, where selected and the speed do not apply (the old
    literal -0.14 * T stays bit for bit).
    """
    T = seconds
    if kind == 'slope':
        return (
            ('forward', 0.12, 0, 0, T, ('x', 0.12 * T)),
            ('left', 0, 0.06, 0, T, ('y', 0.06 * T)),
            ('right', 0, -0.06, 0, T, ('y', -0.06 * T)),
            ('turn_ccw', 0, 0, 0.5, T * 0.5, ('yaw', 0.25 * T)),
            ('turn_cw', 0, 0, -0.5, T * 0.5, ('yaw', -0.25 * T)),
            ('backward', -0.10, 0, 0, T * 1.4, ('x', -0.14 * T)),
        )
    rows = (
        ('forward', 0.12, 0, 0, T, ('x', 0.12 * T)),
        ('backward', -backward_speed, 0, 0, T, ('x', -backward_speed * T)),
        ('left', 0, 0.06, 0, T, ('y', 0.06 * T)),
        ('right', 0, -0.06, 0, T, ('y', -0.06 * T)),
        ('turn_ccw', 0, 0, 0.5, T, ('yaw', 0.5 * T)),
        ('turn_cw', 0, 0, -0.5, T, ('yaw', -0.5 * T)),
        ('arc_left', 0.10, 0, 0.3, T, ('yaw', 0.3 * T)),
        ('arc_right', 0.10, 0, -0.3, T, ('yaw', -0.3 * T)),
    )
    if selected is None:
        return rows
    wanted = set(selected)
    return tuple(row for row in rows if row[0] in wanted)


def lie_wanted(kind, selected):
    """The lie check runs on slope and for the full flat routine only."""
    if kind == 'slope' or selected is None:
        return True
    return set(selected) == set(FLAT_MANEUVERS)
