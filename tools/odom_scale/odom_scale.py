#!/usr/bin/env python3
"""How far the robot thinks it went, against how far it actually went (GAIT-07).

Dead reckoning multiplies the twist it was given by `odom.scale`; the gait walks a little
less than its twist because the feet slip, so the number has to be measured once on the
floor. Measure a straight two metres with a tape, walk them at a low speed while recording,
and let this tool divide:

    python3 odom_scale.py --bag ~/dog-bags/2026-11-02-10-14 --measured-m 2.0

Prints the scale and the line for `robot.yaml` (`odom: scale: …`). Exit codes: 0 - the
number is usable; 1 - the walk does not look like a straight measured run; 2 - no odometry
in the bag.

Judgement is a pure function over positions, so it is tested without ROS.
"""

import argparse
import importlib
import math
import os
import sys

# A walk that is not straight makes "path length" and "distance between the ends" disagree;
# turning during a two-metre run mixes the two and the scale comes out meaningless.
STRAIGHT_TOLERANCE = 1.05      # path / chord: 5 % of wiggle is a floor, not a turn
PLAUSIBLE = (0.7, 1.3)         # outside this, the wrong segment was measured


def path_length(points):
    """Length of the walked polyline, in metres."""
    return sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(points, points[1:]))


def chord(points):
    """Straight-line distance between the first and the last point."""
    if len(points) < 2:
        return 0.0
    (x0, y0), (x1, y1) = points[0], points[-1]
    return math.hypot(x1 - x0, y1 - y0)


def scale_from(measured_m, reported_m):
    """measured / reported - the factor that turns what the robot says into the truth."""
    if reported_m <= 0.0:
        return float('nan')
    return measured_m / reported_m


def judge(points, measured_m):
    """[(state, text)] with state 'ok' | 'плохо' | 'не проверено', then the scale."""
    out = []
    walked = path_length(points)
    straight = chord(points)

    out.append(('ok', 'точек одометрии %d, путь %.3f м, по прямой %.3f м'
                % (len(points), walked, straight)))

    if walked <= 0.0:
        out.append(('плохо', 'робот не двигался: масштаб не из чего считать'))
        return out

    # An out-and-back ends where it started: the robot reported two metres of walking and
    # stands at the origin. There is no distance to divide by, and pretending otherwise
    # produces a confident 1.0.
    if straight <= 1e-6:
        out.append(('плохо', 'прогон вернулся в начало: путь %.2f м, а по прямой ноль — '
                             'мерить надо прямой проход' % walked))
        return out

    # Prefer the chord: on a straight walk it is the measured distance, while the path
    # adds every wobble of dead reckoning to the same two metres.
    if walked / straight > STRAIGHT_TOLERANCE:
        out.append(('плохо', 'прогон не похож на прямой: путь на %.0f %% длиннее прямой, '
                             'во время замера робот поворачивал'
                    % (100.0 * (walked / straight - 1.0))))
        basis = straight
    else:
        basis = straight

    scale = scale_from(measured_m, basis)
    ok = PLAUSIBLE[0] <= scale <= PLAUSIBLE[1]
    out.append(('ok' if ok else 'плохо',
                'odom.scale = %.3f (измерено %.2f м, робот насчитал %.3f м)'
                % (scale, measured_m, basis)))
    if ok:
        out.append(('ok', 'в robot.yaml: odom:\n               scale: %.3f' % scale))
    else:
        out.append(('плохо', 'масштаб вне %.1f…%.1f — похоже, мерили не тот участок или '
                             'запись захватила лишнее' % PLAUSIBLE))
    return out


def from_bag(path, topic='/dog/odom'):
    """Positions from a bag: [(x, y), ...] in the order they were recorded."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=_storage(path)),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        return []
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    pkg, _, name = types[topic].split('/')
    cls = getattr(importlib.import_module('%s.msg' % pkg), name)
    points = []
    while reader.has_next():
        _topic, data, _stamp = reader.read_next()
        msg = deserialize_message(data, cls)
        p = msg.pose.pose.position
        points.append((float(p.x), float(p.y)))
    return points


def _storage(path):
    import pathlib

    return 'mcap' if any('.mcap' in p.name for p in pathlib.Path(path).glob('*')) else 'sqlite3'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True, help='запись прохода (топик /dog/odom)')
    ap.add_argument('--measured-m', type=float, required=True,
                    help='сколько метров робот прошёл на самом деле (рулетка)')
    ap.add_argument('--topic', default='/dog/odom')
    a = ap.parse_args(argv)

    if a.measured_m <= 0:
        print('ошибка: --measured-m должно быть больше нуля', file=sys.stderr)
        return 2
    if not os.path.isdir(a.bag):
        print('ошибка: нет записи %s' % a.bag, file=sys.stderr)
        return 2
    try:
        points = from_bag(a.bag, a.topic)
    except Exception as e:                       # noqa: BLE001 - any ROS/IO error
        print('ошибка: запись не читается: %s' % e, file=sys.stderr)
        return 2
    if len(points) < 2:
        print('ошибка: в записи нет %s' % a.topic, file=sys.stderr)
        return 2

    lines = judge(points, a.measured_m)
    label = {'ok': 'ok', 'плохо': 'ПЛОХО', 'не проверено': 'не проверено'}
    for state, text in lines:
        print('%-13s %s' % (label[state], text))
    bad = [t for s, t in lines if s == 'плохо']
    if bad:
        print('НЕ ГОДИТСЯ: ' + '; '.join(bad))
        return 1
    print('ГОДИТСЯ: масштаб счисления измерен')
    return 0


if __name__ == '__main__':
    sys.exit(main())
