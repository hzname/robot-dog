#!/usr/bin/env python3
"""Replay a gamepad that misbehaves, and judge whether the robot obeyed the rules (TEL-05).

A real Bluetooth pad is needed for the last word on TEL-05 - the link, the pairing, the
driver. Everything up to that is mapping and timeouts, and that can be exercised without a
pad: this publishes /dog/joy the way a pad would, and checks /dog/cmd_vel the way the robot
answers.

The scenarios are the ones a bench session runs by hand:

    drive        deadman held, stick forward      robot moves
    drop         the stream stops                 robot stops within joy_timeout
    return       the stream comes back, centred, deadman released   stays stopped
    stuck        stick held hard, stream dies, comes back with the stick still hard and
                 the deadman released             stays stopped
    stuck_held   both still held on return        moves again - the operator is holding
                 it, and a pad that reports the truth has no way to know better

The last one is the honest limit of the software: a deflected stick that comes back
deflected while the deadman is held is indistinguishable from the operator asking for
motion.

    python3 tools/joy_replay/joy_replay.py                      # all scenarios
    python3 tools/joy_replay/joy_replay.py --only stuck,drop
    python3 tools/joy_replay/joy_replay.py --list

Indices are the Xbox profile from config/teleop.yaml; for a PlayStation pad pass
--axis-vx/--axis-vy/--axis-wz/--button-deadman.
"""

import argparse
import math
import sys
import time

# sensor_msgs/Joy layout: axes[1] = left stick up/down (+1 up), axes[0] = left/right,
# axes[3] = right stick left/right. Buttons are 0/1, deadman is LB by default.
DEFAULT = {'axis_vx': 1, 'axis_vy': 0, 'axis_wz': 3, 'button_deadman': 4,
           'deadman_count': 8, 'axes_count': 8}
STOP_EPS = 0.005      # [m/s] below this the robot counts as standing still
SETTLE = 0.6          # [s] the beginning of a phase is ignored: the robot is reacting


class Phase(object):
    """One stretch of the scenario: what the pad sends, and what the robot must do.

    `silence` is a phase with no messages at all - the pad is gone.
    """

    def __init__(self, name, seconds, expect, axes=None, buttons=None, silence=False):
        self.name = name
        self.seconds = seconds
        # 'moving' | 'stopped' | 'unset' - the last one means "nothing has been commanded
        # yet", which is what a robot with no pad attached must look like.
        self.expect = expect
        self.axes = axes or {}
        self.buttons = buttons or {}
        self.silence = silence

    def joy(self, layout):
        axes = [0.0] * layout['axes_count']
        buttons = [0] * layout['deadman_count']
        for key, value in self.axes.items():
            axes[layout['axis_' + key]] = value
        for key, value in self.buttons.items():
            buttons[layout['button_' + key]] = value
        return axes, buttons

    def __repr__(self):
        return 'Phase(%s, %.1fs, %s%s)' % (self.name, self.seconds, self.expect,
                                           ', silence' if self.silence else '')


def scenarios():
    """The five drills, in the order a bench session runs them."""
    forward = {'vx': 1.0}
    held = {'deadman': 1}
    return {
        'drive': [
            Phase('ждём пульт', 0.8, 'unset'),
            Phase('держись и вперёд', 1.5, 'moving', axes=forward, buttons=held),
        ],
        'drop': [
            Phase('едем', 1.5, 'moving', axes=forward, buttons=held),
            Phase('связь пропала', 1.2, 'stopped', silence=True),
        ],
        'return': [
            Phase('едем', 1.2, 'moving', axes=forward, buttons=held),
            Phase('связь пропала', 1.0, 'stopped', silence=True),
            Phase('пульт вернулся, стик в центре', 1.5, 'stopped'),
        ],
        'stuck': [
            Phase('едем', 1.2, 'moving', axes=forward, buttons=held),
            Phase('связь пропала со стиком в упоре', 1.0, 'stopped', silence=True),
            Phase('пульт вернулся, стик всё ещё в упоре', 1.5, 'stopped',
                  axes={'vx': 0.95}),
        ],
        'stuck_held': [
            Phase('едем', 1.2, 'moving', axes=forward, buttons=held),
            Phase('связь пропала', 1.0, 'stopped', silence=True),
            Phase('вернулся со стиком и гашеткой', 1.5, 'moving',
                  axes={'vx': 0.95}, buttons=held),
        ],
    }


def timeline(phases):
    """[(start, end, phase)] - the wall clock the scenario runs on."""
    out = []
    t = 0.0
    for phase in phases:
        out.append((t, t + phase.seconds, phase))
        t += phase.seconds
    return out


def phase_at(phases, t):
    """Which phase a moment belongs to; None past the end."""
    for start, end, phase in timeline(phases):
        if start <= t < end:
            return phase
    return None


def last_at(samples, t):
    """The command in force at `t`: the last one published at or before it.

    joy_teleop publishes when the command changes, not at a fixed rate, so a steady
    "forward" produces one message and then silence - which is not the same as "no
    command". The robot keeps obeying the last one until it is replaced.
    """
    best = None
    for stamp, values in samples:
        if stamp <= t:
            best = values
        else:
            break
    return best


def classify(values):
    """Does this stretch of cmds count as motion? Any axis above the noise floor."""
    biggest = max([abs(v) for v in values], default=0.0)
    return 'moving' if biggest > STOP_EPS else 'stopped'


def judge(phases, samples):
    """[(phase name, ok, note)] from (t, [cmd_vel components]) samples.

    A phase is judged on its second half: the first moments belong to the reaction the
    previous phase set off, and pretending otherwise would blame the robot for physics.
    """
    rows = []
    for start, end, phase in timeline(phases):
        lo = start + max(SETTLE, 0.5 * phase.seconds)
        # What is in force at the end of the judged window is what the robot is doing.
        values = last_at(samples, end - 1e-3)
        if values is None and lo > 0.0:
            values = last_at(samples, lo)
        seen = classify(values) if values is not None else 'no data'
        ok = seen == ('no data' if phase.expect == 'unset' else phase.expect)
        rows.append((phase.name, ok, 'ждали %s, вышло %s' % (phase.expect, seen)))
    return rows


def render(rows):
    lines = []
    for name, ok, note in rows:
        lines.append('%s %s: %s' % ('OK  ' if ok else 'ПЛОХО', name, note))
    good = sum(1 for _, ok, _ in rows if ok)
    lines.append('')
    lines.append('итого: %d из %d' % (good, len(rows)))
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Пульт, который ведёт себя плохо (TEL-05).')
    ap.add_argument('--only', help='через запятую: какие сценарии гонять')
    ap.add_argument('--list', action='store_true', help='показать сценарии и выйти')
    ap.add_argument('--speed', type=float, default=1.0, help='ускорить сценарий, x')
    ap.add_argument('--axis-vx', type=int, default=DEFAULT['axis_vx'])
    ap.add_argument('--axis-vy', type=int, default=DEFAULT['axis_vy'])
    ap.add_argument('--axis-wz', type=int, default=DEFAULT['axis_wz'])
    ap.add_argument('--button-deadman', type=int, default=DEFAULT['button_deadman'])
    a = ap.parse_args(argv)

    all_of = scenarios()
    if a.list:
        for name, phases in all_of.items():
            print('%s: %s' % (name, ', '.join('%s %.1fs→%s' % (p.name, p.seconds, p.expect)
                                              for p in phases)))
        return 0
    if a.only:
        wanted = [n.strip() for n in a.only.split(',') if n.strip()]
        unknown = [n for n in wanted if n not in all_of]
        if unknown:
            print('нет такого сценария: %s (есть: %s)' % (', '.join(unknown),
                                                          ', '.join(sorted(all_of))),
                  file=sys.stderr)
            return 2
    else:
        wanted = list(all_of)

    layout = dict(DEFAULT, axis_vx=a.axis_vx, axis_vy=a.axis_vy, axis_wz=a.axis_wz,
                  button_deadman=a.button_deadman)
    return run(wanted, all_of, layout, a.speed)


def run(wanted, all_of, layout, speed=1.0):
    """The ROS half: publish the pad, record cmd_vel, judge at the end."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Joy
        from geometry_msgs.msg import Twist
    except ImportError:
        print('нет rclpy: запускайте на роботе или в контейнере', file=sys.stderr)
        return 2

    rclpy.init()
    node = Node('joy_replay')
    pub = node.create_publisher(Joy, '/dog/joy', 10)
    samples = []          # (t, [vx, vy, wz]) from cmd_vel, relative to the scenario start
    clock = {'base': time.monotonic()}

    def on_vel(msg):
        samples.append((time.monotonic() - clock['base'],
                        [msg.linear.x, msg.linear.y, msg.angular.z]))

    node.create_subscription(Twist, '/dog/cmd_vel', on_vel, 10)
    print('пульт: %.0f Гц, сценарии: %s' % (50, ', '.join(wanted)))

    rows = []
    for name in wanted:
        phases = all_of[name]
        samples.clear()
        base = time.monotonic()
        clock['base'] = base     # each scenario is judged on its own clock
        print('\n== %s' % name)
        for start_of_phase, end_of_phase, phase in timeline(phases):
            print('   %-40s %s' % (phase.name, phase.expect))
            while True:
                now = (time.monotonic() - base) * speed
                if now >= end_of_phase:
                    break
                if not phase.silence:
                    axes, buttons = phase.joy(layout)
                    msg = Joy()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.axes = [float(v) for v in axes]
                    msg.buttons = [int(v) for v in buttons]
                    pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(max(0.0, 0.02 / speed))
        rows += judge(phases, samples)

    node.destroy_node()
    rclpy.shutdown()
    print('\n' + render(rows))
    return 0 if all(ok for _, ok, _ in rows) else 1


if __name__ == '__main__':
    sys.exit(main())
