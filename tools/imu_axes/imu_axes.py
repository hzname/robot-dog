#!/usr/bin/env python3
"""Check the IMU's axes and sign before the robot is allowed to trust it (CAL-08).

The bench procedure used to be eyes-only: "tilt the nose down - pitch should grow". This
turns it into a verdict. Three movements, each compared against what REP-103 demands:

    rest         robot still, level          |roll|, |pitch| < 5 deg, gravity along +z
    nose down    tilt the nose down          pitch grows
    left up      raise the robot's left side roll grows
    counter-cw   turn it counter-clockwise   angular velocity z positive

The last one matters most: a wrong gyro sign makes heading hold steer the robot away from
its course instead of keeping it.

Two layers, like the rest of tools/: this file is pure arithmetic over recorded samples
and is tested without ROS; run_ros.py subscribes to /dog/imu/data and drives the steps.

    python3 -m tools.imu_axes.run_ros                    # on the robot, interactive
    python3 tools/imu_axes/imu_axes.py rest.csv nose.csv left.csv ccw.csv   # offline
"""

import argparse
import math
import sys

try:
    from collections import namedtuple
except ImportError:  # pragma: no cover
    raise

G = 9.81

# t [s], accel [m/s^2] in body axes, gyro [rad/s], roll/pitch [deg] as the node reports them
Sample = namedtuple('Sample', 't ax ay az wx wy wz roll pitch')

REQUIRED_TILT_DEG = 5.0     # below this the operator did not actually move the robot
LEVEL_TOLERANCE_DEG = 5.0
RATE_MIN_HZ = 50.0
GRAVITY_RANGE = (9.0, 10.6)


def sample(line):
    """One CSV row -> Sample; ';' or ',' separates, extra text is ignored."""
    row = line.replace(';', ',').split(',')
    if len(row) < 9:
        raise ValueError('нужно 9 чисел: t,ax,ay,az,wx,wy,wz,roll,pitch')
    return Sample(*[float(v) for v in row[:9]])


def read_csv(path):
    out = []
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            text = line.strip()
            if not text or text.startswith('#'):
                continue
            try:
                out.append(sample(text))
            except ValueError:
                continue
    return out


def mean(values):
    return sum(values) / len(values) if values else 0.0


def mean_accel(samples):
    return (mean([s.ax for s in samples]), mean([s.ay for s in samples]),
            mean([s.az for s in samples]))


def spread(values):
    """Peak-to-peak: a still robot gives a small number even if the mean is biased."""
    return (max(values) - min(values)) if values else 0.0


def rest_report(rest):
    """What a still robot shows: level, gravity direction, rate, gyro noise in deg/s."""
    if len(rest) < 5:
        return {'ok': False, 'why': 'слишком мало отсчётов в покое: нужно хотя бы 5'}
    ax, ay, az = mean_accel(rest)
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    times = [s.t for s in rest]
    span = times[-1] - times[0]
    hz = (len(rest) - 1) / span if span > 0 else 0.0
    noise = max(spread([s.wx for s in rest]), spread([s.wy for s in rest]),
                spread([s.wz for s in rest])) * 180.0 / math.pi
    return {
        'ok': GRAVITY_RANGE[0] <= norm <= GRAVITY_RANGE[1] and hz >= RATE_MIN_HZ,
        'accel': (ax, ay, az),
        'norm': norm,
        'hz': hz,
        'noise_dps': noise,
        'tilt': max(abs(mean([s.roll for s in rest])), abs(mean([s.pitch for s in rest]))),
        'why': '' if GRAVITY_RANGE[0] <= norm <= GRAVITY_RANGE[1] else
               'модуль ускорения %.2f м/с² вне 9.0-10.6: датчик не в покое или шкала врёт' % norm,
    }


def moved(rest, step, attr):
    """How far the operator moved the angle during a step, from the rest baseline."""
    base = mean([getattr(s, attr) for s in rest])
    values = [getattr(s, attr) - base for s in step]
    extreme = max(values, key=abs) if values else 0.0
    return extreme


def judge(rest, nose_down, left_up, ccw):
    """The verdict: what each movement said, whether the axes can be trusted, and which
    `axes:` line to put in imu.yaml when the mount is one of the three ordinary ones."""
    report = rest_report(rest)
    lines = []
    problems = []
    if not report['ok'] and report['why']:
        lines.append('покой: ' + report['why'])
    ax, ay, az = report['accel']
    lines.append('покой: g = (%+.2f, %+.2f, %+.2f) м/с², |g| = %.2f, %.0f Гц, шум гироскопа %.2f °/с'
                 % (ax, ay, az, report['norm'], report['hz'], report['noise_dps']))
    if report['tilt'] > LEVEL_TOLERANCE_DEG:
        lines.append('покой: робот стоит неровно (%.1f°) — поставьте его на ровное место'
                     % report['tilt'])

    pitch = moved(rest, nose_down, 'pitch')
    roll = moved(rest, left_up, 'roll')
    yaw_rate = max([s.wz for s in ccw], key=abs) if ccw else 0.0

    if abs(pitch) < REQUIRED_TILT_DEG:
        problems.append('наклон носа: движение не увидел (%.1f°), повторите и наклоняйте сильнее'
                        % pitch)
    else:
        lines.append('наклон носа вниз: тангаж %+.1f° — %s'
                     % (pitch, 'верно' if pitch > 0 else 'ЗНАК НЕВЕРНЫЙ'))
        if pitch <= 0:
            problems.append('тангаж растёт вниз, а должен вверх')

    if abs(roll) < REQUIRED_TILT_DEG:
        problems.append('подъём левого бока: движение не увидел (%.1f°), повторите' % roll)
    else:
        lines.append('подъём левого бока: крен %+.1f° — %s'
                     % (roll, 'верно' if roll > 0 else 'ЗНАК НЕВЕРНЫЙ'))
        if roll <= 0:
            problems.append('крен растёт при опускании левого бока, а должен при подъёме')

    if abs(yaw_rate) < 0.2:
        problems.append('поворот вокруг вертикали: вращения не увидел, крутите быстрее')
    else:
        lines.append('поворот против часовой: gyro z %+.0f °/с — %s'
                     % (yaw_rate * 180.0 / math.pi,
                        'верно' if yaw_rate > 0 else 'ЗНАК НЕВЕРНЫЙ'))
        if yaw_rate <= 0:
            problems.append('гироскоп Z инвертирован: удержание курса будет уводить робота')

    seen = (abs(pitch) >= REQUIRED_TILT_DEG, abs(roll) >= REQUIRED_TILT_DEG,
            abs(yaw_rate) >= 0.2)
    axes_line, mount = axes_advice(ax, ay, az, pitch, roll, yaw_rate, seen)
    return {'ok': not problems, 'lines': lines, 'problems': problems,
            'axes': axes_line, 'mount': mount, 'report': report}


def axes_advice(ax, ay, az, pitch, roll, yaw_rate, seen=(True, True, True)):
    """The `axes:` string for the ordinary mounts, and nothing when the plate is not one.

    Only proper rotations are offered: an odd number of sign flips is a mirror, and a
    mirrored frame makes every cross product (and so every gyro integral) wrong.

    `seen` says whether each movement was large enough to judge. A step nobody performed
    is not evidence for a sign, so no advice is given until all three are seen.
    """
    if not all(seen):
        return None, 'не все движения выполнены: подсказку по axes дам после них'
    flat_up = az > 0.8 * math.sqrt(ax * ax + ay * ay + az * az)
    flat_down = az < -0.8 * math.sqrt(ax * ax + ay * ay + az * az)
    signs_ok = pitch > 0 and roll > 0 and yaw_rate > 0
    signs_all = pitch < 0 and roll < 0 and yaw_rate < 0

    if flat_up and signs_ok:
        return 'x,y,z', 'плата лежит горизонтально, как на схеме'
    if flat_up and signs_all:
        return '-x,-y,z', 'плата повёрнута на 180° вокруг вертикали'
    if flat_down and roll < 0 and yaw_rate < 0:
        return 'x,-y,-z', 'плата перевёрнута: компонентами вниз'
    if not (flat_up or flat_down):
        return None, ('плата стоит не горизонтально: g не смотрит вдоль оси Z — '
                      'сверьтесь со схемой в docs/DEPLOYMENT.md, этап 7')
    return None, ('знаки не складываются ни в один правильный поворот: проверьте, что '
                  'наклоны и поворот делались в ту сторону, и что `axes` в imu.yaml не '
                  'портит уже верную плату')


def render(verdict):
    out = list(verdict['lines'])
    if verdict['problems']:
        out.append('')
        for p in verdict['problems']:
            out.append('НЕ ТАК: ' + p)
        out.append('')
        out.append('Оси не проверены. В imu.yaml должно остаться calibrated: false.')
    else:
        out.append('')
        out.append('ИТОГ: оси и знак гироскопа верны, ' + verdict['mount'])
        if verdict['axes']:
            out.append('в imu.yaml: axes: "%s"' % verdict['axes'])
        out.append('отметку calibrated: true ставит imu_node сам после замера смещения;')
        out.append('убедившись в этом, включите slope.compensation и heading.hold.')
    return '\n'.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Разбор записанных шагов проверки IMU.')
    ap.add_argument('rest', help='покой (CSV: t,ax,ay,az,wx,wy,wz,roll,pitch)')
    ap.add_argument('nose_down', help='наклон носа вниз')
    ap.add_argument('left_up', help='подъём левого бока')
    ap.add_argument('ccw', help='поворот против часовой')
    a = ap.parse_args(argv)
    try:
        verdict = judge(read_csv(a.rest), read_csv(a.nose_down),
                        read_csv(a.left_up), read_csv(a.ccw))
    except OSError as e:
        print('ошибка: %s' % e, file=sys.stderr)
        return 2
    print(render(verdict))
    return 0 if verdict['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
