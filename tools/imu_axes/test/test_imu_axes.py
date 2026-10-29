"""The IMU verdict is arithmetic over samples, so it is tested without a robot (CAL-08).

The cases mirror the ones a bench session actually produces: a correct mount, the three
ordinary wrong ones, an operator who did not move enough, and a plate that is not lying
flat at all.
"""

import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import imu_axes as ia  # noqa: E402


def rows(roll=0.0, pitch=0.0, yaw_rate_dps=0.0, g=9.81, n=100, dt=0.01, direction=1.0,
         noise=0.0):
    """A still segment: the given angles held for `n` samples."""
    out = []
    r = math.radians(roll) * direction
    p = math.radians(pitch) * direction
    ax = -g * math.sin(p)
    ay = g * math.sin(r) * math.cos(p)
    az = g * math.cos(r) * math.cos(p)
    for i in range(n):
        out.append(ia.Sample(i * dt, ax + noise * (i % 3 - 1), ay, az,
                             0.0, 0.0, math.radians(yaw_rate_dps) * direction,
                             roll * direction, pitch * direction))
    return out


def good_steps():
    return (rows(), rows(pitch=15.0), rows(roll=15.0), rows(yaw_rate_dps=40.0))


def test_a_level_mount_with_the_right_signs_passes():
    v = ia.judge(*good_steps())
    assert v['ok'], v['problems']
    assert v['axes'] == 'x,y,z'
    assert 'x,y,z' in ia.render(v)


def test_gravity_is_read_from_the_accelerometer():
    report = ia.rest_report(rows())
    ax, ay, az = report['accel']
    assert ax == pytest.approx(0.0, abs=1e-9)
    assert az == pytest.approx(9.81, abs=0.01)
    assert report['hz'] == pytest.approx(100.0, rel=0.02)


def test_a_slow_topic_fails_the_rate_check():
    report = ia.rest_report(rows(n=20, dt=0.1))
    assert report['hz'] == pytest.approx(10.0, abs=0.5)
    assert not report['ok']


def test_an_operator_who_did_not_move_gets_asked_again():
    steps = (rows(), rows(pitch=1.0), rows(roll=15.0), rows(yaw_rate_dps=40.0))
    v = ia.judge(*steps)
    assert not v['ok']
    assert any('повторите' in p for p in v['problems'])
    assert v['axes'] is None


def test_an_inverted_gyro_is_called_out_by_name():
    steps = (rows(), rows(pitch=15.0), rows(roll=15.0), rows(yaw_rate_dps=-40.0))
    v = ia.judge(*steps)
    assert not v['ok']
    assert any('гироскоп Z инвертирован' in p for p in v['problems'])
    assert 'калибровк' in ia.render(v).lower() or 'calibrated' in ia.render(v)


def test_a_plate_turned_by_half_a_turn_gets_a_proper_rotation():
    # The whole body frame is rotated 180 deg about the vertical: both angles and the
    # yaw rate flip sign, gravity still points up. '-x,-y,z' is a rotation, not a mirror.
    steps = (rows(), rows(pitch=15.0, direction=-1.0), rows(roll=15.0, direction=-1.0),
             rows(yaw_rate_dps=40.0, direction=-1.0))
    v = ia.judge(*steps)
    assert not v['ok'] or v['axes'] == '-x,-y,z'
    assert v['axes'] == '-x,-y,z'


def test_an_upside_down_plate_is_recognised_by_gravity():
    # Components down: gravity reads along -z, roll and the yaw rate flip with it.
    steps = (rows(g=-9.81), rows(pitch=15.0), rows(roll=15.0, direction=-1.0),
             rows(yaw_rate_dps=40.0, direction=-1.0))
    v = ia.judge(*steps)
    assert v['axes'] == 'x,-y,-z'


def test_a_plate_on_its_side_is_not_given_a_guess():
    side = rows()
    # 90 deg about y: gravity now along +x, and the angles read from the accelerometer
    # are meaningless for the ordinary mounts.
    side = [s._replace(ax=9.81, az=0.0) for s in side]
    steps = (side, rows(pitch=15.0), rows(roll=15.0), rows(yaw_rate_dps=40.0))
    v = ia.judge(*steps)
    assert v['axes'] is None
    assert 'не горизонтально' in v['mount']


def test_a_mirror_is_never_suggested():
    # Only a single axis flipped: no proper rotation explains it, so the tool must not
    # invent an `axes` line - a mirrored frame breaks every cross product.
    steps = (rows(), rows(pitch=15.0), rows(roll=15.0, direction=-1.0),
             rows(yaw_rate_dps=40.0))
    v = ia.judge(*steps)
    assert v['axes'] is None


def test_csv_round_trip(tmp_path):
    path = tmp_path / 'rest.csv'
    with open(path, 'w', encoding='utf-8') as f:
        f.write('# t,ax,ay,az,wx,wy,wz,roll,pitch\n')
        for s in rows():
            f.write('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n' % s)
    back = ia.read_csv(str(path))
    assert len(back) == 100
    assert back[0].az == pytest.approx(9.81, abs=0.01)


def test_rows_with_missing_columns_are_skipped(tmp_path):
    path = tmp_path / 'bad.csv'
    path.write_text('1,2,3\n\n0.1,0,0,9.8,0,0,0,0,0\n', encoding='utf-8')
    assert len(ia.read_csv(str(path))) == 1


def test_the_cli_says_ok_with_a_zero_exit(tmp_path):
    paths = []
    for name, seg in zip(['rest', 'nose', 'left', 'ccw'], good_steps()):
        p = tmp_path / (name + '.csv')
        with open(p, 'w', encoding='utf-8') as f:
            for s in seg:
                f.write('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n' % s)
        paths.append(str(p))
    assert ia.main(paths) == 0


def test_the_cli_fails_when_the_axes_do_not_check_out(tmp_path):
    steps = (rows(), rows(pitch=-15.0), rows(roll=15.0), rows(yaw_rate_dps=40.0))
    paths = []
    for name, seg in zip(['rest', 'nose', 'left', 'ccw'], steps):
        p = tmp_path / (name + '.csv')
        with open(p, 'w', encoding='utf-8') as f:
            for s in seg:
                f.write('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n' % s)
        paths.append(str(p))
    assert ia.main(paths) == 1
