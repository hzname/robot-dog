#!/usr/bin/env python3
"""Verdict on a floor run, from the recording, not from memory (FLOOR-01/FLOOR-02).

The criterion for the first floor phase is a ten-minute walk with no falls, no latched
faults, no I2C errors, the servo rail above 5.5 V and the servo cases under 60 °C. Four of
those are visible in a bag; temperature comes from an IR thermometer after the run and is
passed in by hand.

    python3 floor_run.py --bag ~/dog-bags/2026-11-02-10-14 --temp-c 52

Reads: /dog/power (rail), /dog/imu/data (tilt), /dog/state and /dog/estop (faults),
/dog/diagnostics (bus errors from the driver), /dog/odom (distance). Exit codes: 0 - the run
counts; 1 - it does not; 2 - the bag could not be read.

The judgement is a pure function, so the rules are tested without ROS.
"""

import argparse
import importlib
import math
import os
import sys

# What the phase-8 criterion actually demands, in one place.
LIMITS = {
    'duration_s': 600.0,        # ten minutes of walking
    'min_voltage_v': 5.5,       # servo rail must never sag below this
    'max_temp_c': 60.0,         # servo case temperature after the run
    'soft_tilt_deg': 22.0,      # the governor's soft limit from phase 4 - a warning line
    'hard_tilt_deg': 32.0,      # past this the robot was falling
}

TOPICS = ('/dog/power', '/dog/imu/data', '/dog/state', '/dog/estop',
          '/dog/diagnostics', '/dog/odom')

OK, BAD, UNKNOWN = 'ok', 'плохо', 'не проверено'


def _quat_tilt(q):
    """(roll, pitch) in degrees from a quaternion - what the tilt guard sees."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


class Run:
    """Everything the judgement needs, gathered from a bag (or made up in a test)."""

    def __init__(self):
        self.duration = 0.0
        self.voltage = []       # (t, V)
        self.current = []       # (t, A drawn)
        self.tilt = []          # (t, roll_deg, pitch_deg)
        self.states = []        # (t, state)
        self.estops = []        # (t, bool)
        self.diag = []          # (t, source, message)
        self.odom = []          # (t, x, y)

    def add(self, topic, msg, t):
        if topic.endswith('/power'):
            self.voltage.append((t, float(msg.voltage)))
            self.current.append((t, abs(float(msg.current))))
        elif topic.endswith('/imu/data'):
            self.tilt.append((t,) + _quat_tilt(msg.orientation))
        elif topic.endswith('/state'):
            self.states.append((t, str(msg.data)))
        elif topic.endswith('/estop'):
            self.estops.append((t, bool(msg.data)))
        elif topic.endswith('/diagnostics'):
            for status in msg.status:
                self.diag.append((t, status.name, status.message))
        elif topic.endswith('/odom'):
            p = msg.pose.pose.position
            self.odom.append((t, float(p.x), float(p.y)))

    def shift(self, dt):
        """Times relative to the start of the recording - what the operator thinks in."""
        for name in ('voltage', 'current', 'tilt', 'states', 'estops', 'diag', 'odom'):
            setattr(self, name, [(row[0] + dt,) + row[1:] for row in getattr(self, name)])

    def distance(self):
        """Path length from odometry, in metres: a run that did not move proves nothing."""
        return sum(math.hypot(x1 - x0, y1 - y0)
                   for (_, x0, y0), (_, x1, y1) in zip(self.odom, self.odom[1:]))


def _class(typename):
    pkg, _, name = typename.split('/')
    return getattr(importlib.import_module('%s.msg' % pkg), name)


def _storage(path):
    import pathlib

    return 'mcap' if any('.mcap' in p.name for p in pathlib.Path(path).glob('*')) else 'sqlite3'


def from_bag(path, topics=TOPICS):
    """A Run filled from a recorded bag. Needs ROS; the judgement below does not."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=_storage(path)),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))

    run = Run()
    stamps = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        stamps.append(stamp)
        try:
            msg = deserialize_message(data, _class(types[topic]))
        except (KeyError, AttributeError, ModuleNotFoundError):
            continue                                    # a type nobody can deserialise
        run.add(topic, msg, stamp / 1e9)
    if stamps:
        run.duration = (max(stamps) - min(stamps)) / 1e9
        run.shift(-min(stamps) / 1e9)
    return run


def judge(run, temp_c=None, limits=None):
    """[(state, text)] with state OK | BAD | UNKNOWN, in reading order."""
    lim = dict(LIMITS, **(limits or {}))
    out = []

    ok = run.duration >= lim['duration_s']
    out.append((OK if ok else BAD,
                'длительность %.0f с (нужно %.0f)' % (run.duration, lim['duration_s'])))

    if run.voltage:
        low = min(v for _, v in run.voltage)
        out.append((OK if low > lim['min_voltage_v'] else BAD,
                    'шина: минимум %.2f В (нужно выше %.1f)' % (low, lim['min_voltage_v'])))
    else:
        out.append((UNKNOWN, 'шина: в записи нет /dog/power'))

    # A latch is a fault still there at the end; a press that was released is a person
    # doing their job, and that is only worth counting.
    latched = bool(run.estops) and run.estops[-1][1]
    presses = sum(1 for i, (_, v) in enumerate(run.estops) if v and
                  (i == 0 or not run.estops[i - 1][1]))
    if run.estops:
        out.append((BAD if latched else OK,
                    'E-STOP: защёлкнут в конце' if latched else
                    'E-STOP: не защёлкнут (нажатий за прогон: %d)' % presses))
    elif run.states:
        # The driver announces a fault once, when it latches: a clean run has no estop
        # messages at all, and calling that "unknown" would refuse every good run. The
        # state machine publishes its state continuously, so it answers instead.
        latched = run.states[-1][1] == 'estop'
        out.append((BAD if latched else OK,
                    'E-STOP: защёлкнут (по состоянию)' if latched else
                    'E-STOP: не защёлкнут (по состоянию: сообщений /dog/estop не было)'))
    else:
        out.append((UNKNOWN, 'E-STOP: в записи нет ни /dog/estop, ни /dog/state'))

    bus = [(t, src, text) for t, src, text in run.diag if 'bus' in text.lower()]
    if run.diag:
        out.append((BAD if bus else OK,
                    'ошибки шины: %d%s' % (len(bus), (' — ' + bus[0][2]) if bus else '')))
    else:
        out.append((UNKNOWN, 'ошибки шины: в записи нет /dog/diagnostics'))

    if run.tilt:
        peak = max(max(abs(r), abs(p)) for _, r, p in run.tilt)
        state = BAD if peak >= lim['hard_tilt_deg'] else (
            OK if peak < lim['soft_tilt_deg'] else UNKNOWN)
        out.append((state, 'наклон корпуса: пик %.1f° (мягкий предел %.0f°, жёсткий %.0f°)'
                    % (peak, lim['soft_tilt_deg'], lim['hard_tilt_deg'])))
    else:
        out.append((UNKNOWN, 'наклон: в записи нет /dog/imu/data'))

    if temp_c is None:
        out.append((UNKNOWN, 'температура серв: не измерялась (ИК-термометр после прогона, '
                             '--temp-c)'))
    else:
        out.append((OK if temp_c <= lim['max_temp_c'] else BAD,
                    'температура серв: %.0f °C (предел %.0f)' % (temp_c, lim['max_temp_c'])))

    if run.odom:
        out.append((OK, 'путь %.1f м, точек одометрии %d' % (run.distance(), len(run.odom))))
    else:
        out.append((UNKNOWN, 'путь: в записи нет /dog/odom'))

    if run.current:
        drawn = sorted(row[1] for row in run.current)
        p99 = drawn[max(0, int(math.ceil(0.99 * len(drawn))) - 1)]
        out.append((OK, 'ток: p99 %.2f А, максимум %.2f А' % (p99, drawn[-1])))
    return out


def verdict(lines):
    """(passes, summary) - unproven counts as not yet, never as passed."""
    bad = [t for s, t in lines if s == BAD]
    unknown = [t for s, t in lines if s == UNKNOWN]
    if bad:
        # Unproven points are named too: after a failed run they are what to look at next.
        tail = (' | не проверено: ' + '; '.join(unknown)) if unknown else ''
        return False, 'НЕ ГОДИТСЯ: ' + '; '.join(bad) + tail
    if unknown:
        return False, 'НЕ ЗАВЕРШЕНО: ' + '; '.join(unknown)
    return True, 'ГОДИТСЯ: прогон закрывает критерий'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True, help='запись прогона')
    ap.add_argument('--temp-c', type=float, help='температура корпуса серв после прогона')
    a = ap.parse_args(argv)

    if not os.path.isdir(a.bag):
        print('ошибка: нет записи %s' % a.bag, file=sys.stderr)
        return 2
    try:
        run = from_bag(a.bag)
    except Exception as e:                        # noqa: BLE001 - any ROS/IO error
        print('ошибка: запись не читается: %s' % e, file=sys.stderr)
        return 2
    if not (run.voltage or run.tilt or run.states):
        print('ошибка: в записи нет ни одного из нужных топиков (%s)' % ', '.join(TOPICS),
              file=sys.stderr)
        return 2

    lines = judge(run, a.temp_c)
    label = {OK: 'ok', BAD: 'ПЛОХО', UNKNOWN: 'не проверено'}
    for state, text in lines:
        print('%-13s %s' % (label[state], text))
    ok, summary = verdict(lines)
    print(summary)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
