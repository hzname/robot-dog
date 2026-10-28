import math
import os
import re
import shutil
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import robot_setup as rs  # noqa: E402


@pytest.fixture
def cfg(tmp_path):
    for f in ('robot.yaml', 'servos.yaml'):
        shutil.copy(os.path.join(rs.CONFIG, f), tmp_path / f)
    return str(tmp_path)


def test_current_config_is_valid_and_roundtrips_unchanged(cfg):
    v = rs.load(cfg)
    msgs, info = rs.validate(v)
    assert not [m for m in msgs if m[0] == 'error'], msgs
    before = [open(os.path.join(cfg, f)).read() for f in ('robot.yaml', 'servos.yaml')]
    assert rs.save(cfg, v) == '(ничего не изменилось)'
    assert [open(os.path.join(cfg, f)).read() for f in ('robot.yaml', 'servos.yaml')] == before


def test_save_changes_only_values_and_keeps_comments(cfg):
    v = rs.load(cfg)
    v['thigh'], v['calf'], v['hip_x'] = 110, 120, 95
    diff = rs.save(cfg, v)
    text = open(os.path.join(cfg, 'robot.yaml')).read()
    assert 'thigh: 0.11 ' in text and '# thigh axis -> knee axis' in text
    r = yaml.safe_load(text)['/**']['ros__parameters']['geometry']
    assert (r['thigh'], r['calf'], r['hip_x']) == (0.11, 0.12, 0.095)
    assert diff.count('\n-') == 3 or diff.count('\n-      ') == 3
    assert os.path.exists(os.path.join(cfg, 'robot.yaml.bak'))


def test_body_com_x_is_a_form_field_in_the_body_group():
    f = rs.FIELDS['body_com_x']
    assert (f[2], f[3], f[4], f[5], f[6], f[7]) == ('мм', 'description', 'body_com_x', 1000, -60, 60)
    group = next(g for g in rs.GROUPS if g[0] == 'body')
    assert any(x[0] == 'body_com_x' for x in group[3])


def test_body_com_x_is_loaded_saved_and_keeps_comments(cfg):
    v = rs.load(cfg)
    assert v['body_com_x'] == 0.0
    v['body_com_x'] = 12.5
    diff = rs.save(cfg, v)
    text = open(os.path.join(cfg, 'robot.yaml')).read()
    assert 'body_com_x: 0.0125' in text
    assert '# [m] trunk centre of mass forward of the body centre' in text
    assert yaml.safe_load(text)['/**']['ros__parameters']['description']['body_com_x'] == 0.0125
    assert rs.load(cfg)['body_com_x'] == 12.5
    removed = [ln for ln in diff.splitlines() if ln.startswith('-') and not ln.startswith('---')]
    assert len(removed) == 1, diff


def test_body_com_x_out_of_range_is_an_error():
    v = rs.load(rs.CONFIG)
    v['body_com_x'] = 70
    assert any(lv == 'error' and 'вне' in t for lv, t in rs.validate(v)[0])


def test_body_com_x_beyond_the_hip_axes_warns():
    base = rs.load(rs.CONFIG)
    msgs = rs.validate(dict(base, body_com_x=55, hip_x=50))[0]
    assert any(lv == 'warn' and 'дальше осей бёдер' in t for lv, t in msgs)
    assert not [m for m in msgs if m[0] == 'error']
    assert not any(lv == 'warn' and 'дальше осей бёдер' in t for lv, t in rs.validate(dict(base, body_com_x=0))[0])


def test_cli_enters_body_com_x_end_to_end(cfg, monkeypatch):
    def fake_input(prompt=''):
        if 'body_com_x' in prompt:
            return '12,5'
        if 'Сохранить' in prompt:
            return 'y'
        return ''

    monkeypatch.setattr('builtins.input', fake_input)
    assert rs.cli(cfg) == 0
    r = yaml.safe_load(open(os.path.join(cfg, 'robot.yaml')))['/**']['ros__parameters']
    assert r['description']['body_com_x'] == 0.0125


def test_hip_limits_mirror_for_right_legs(cfg):
    v = rs.load(cfg)
    v['hip_out'], v['hip_in'] = 35, 10
    rs.save(cfg, v)
    s = yaml.safe_load(open(os.path.join(cfg, 'servos.yaml')))['/**/servo_driver']['ros__parameters']
    assert (s['lf_hip_joint']['min_deg'], s['lf_hip_joint']['max_deg']) == (-10.0, 35.0)  # + = left = outward
    assert (s['rr_hip_joint']['min_deg'], s['rr_hip_joint']['max_deg']) == (-35.0, 10.0)
    r = yaml.safe_load(open(os.path.join(cfg, 'robot.yaml')))['/**']['ros__parameters']['description']
    assert r['hip_limits_deg'] == [-35.0, 35.0]
    assert rs.load(cfg)['hip_out'] == 35 and rs.load(cfg)['hip_in'] == 10


def test_linkage_and_coupling_written_and_removed(cfg):
    v = rs.load(cfg)
    v.update({'calf_servo_arm_mm': 15, 'calf_joint_arm_mm': 20, 'calf_rod_mm': 95, 'calf_axis_distance_mm': 95,
              'calf_coupled': 1})
    assert not [m for m in rs.validate(v)[0] if m[0] == 'error']
    rs.save(cfg, v)
    s = yaml.safe_load(open(os.path.join(cfg, 'servos.yaml')))['/**/servo_driver']['ros__parameters']
    c = s['rr_calf_joint']
    assert (c['servo_arm_mm'], c['joint_arm_mm'], c['rod_mm'], c['axis_distance_mm']) == (15, 20, 95, 95)
    assert c['coupled_to'] == 'rr_thigh_joint' and c['coupling'] == 1.0
    v['calf_coupled'] = 0
    rs.save(cfg, v)
    s = yaml.safe_load(open(os.path.join(cfg, 'servos.yaml')))['/**/servo_driver']['ros__parameters']
    assert 'coupled_to' not in s['rr_calf_joint']


def test_validation_catches_what_would_break_the_robot():
    base = rs.load(rs.CONFIG)
    v = dict(base, stand_height=260)  # longer than the leg
    assert any('нога достаёт' in t for lv, t in rs.validate(v)[0] if lv == 'error')
    v = dict(base, calf_max=-100)  # stand pose (calf -89 deg) outside the calf limits
    assert any('угол колена в стойке' in t for lv, t in rs.validate(v)[0] if lv == 'error')
    v = dict(base, thigh_servo_arm_mm=15, thigh_axis_distance_mm=0)
    assert any('расстояние между осями' in t for lv, t in rs.validate(v)[0] if lv == 'error')
    v = dict(base, thigh_servo_arm_mm=15, thigh_joint_arm_mm=15, thigh_rod_mm=300, thigh_axis_distance_mm=60)
    assert any('не замыкается' in t for lv, t in rs.validate(v)[0] if lv == 'error')
    v = dict(base, total_mass=3000)
    assert any('отличается от веса' in t for lv, t in rs.validate(v)[0] if lv == 'warn')


def test_stand_angles_match_the_controller():
    q1, q2 = rs.stand_angles(105, 105, 150)
    # same IK as dog_control / dog_description.stand_angles
    L = 105
    assert abs(L * math.cos(math.radians(q1)) + L * math.cos(math.radians(q1 + q2)) - 150) < 1e-6
    assert abs(L * math.sin(math.radians(q1)) + L * math.sin(math.radians(q1 + q2))) < 1e-6
    assert -95 < q2 < -85


def test_sensor_fields_lists_and_flags(cfg):
    v = rs.load(cfg)
    assert v['tof_fc_pitch_deg'] == 20 and v['gs2'] == 1 and v['tof_offset_fr'] == 0
    v['tof_fc_pitch_deg'], v['tof_offset_fr'], v['gs2'], v['guard_max_step'] = 25, 4, 0, 25
    assert not [m for m in rs.validate(v)[0] if m[0] == 'error']
    rs.save(cfg, v)
    text = open(os.path.join(cfg, 'robot.yaml')).read()
    r = yaml.safe_load(text)['/**']['ros__parameters']
    assert r['sensors']['tof_pitch_deg'] == [40.0, 40.0, 25.0, 40.0]
    assert r['perception']['tof_offsets'] == [0.0, 0.004, 0.0, 0.0]
    assert r['sensors']['gs2'] is False and r['perception']['guard_max_step'] == 0.025
    assert '# VL53L1X ToF sensors' in text  # comments kept
    assert rs.load(cfg)['gs2'] == 0


# reference values printed by the C++ Linkage of servo_driver.cpp, RESEARCH Pattern 2
KNEE_RATIO_CPP = {-30: 1.605, -20: 1.427, -10: 1.357, 0: 1.334, 10: 1.344, 20: 1.392, 30: 1.507}


@pytest.mark.parametrize('delta', sorted(KNEE_RATIO_CPP))
def test_knee_ratio_matches_the_cpp_linkage(delta):
    got = rs.knee_ratio_max(15, 20, 95, 95, lo_deg=-90 + delta - 0.05, hi_deg=-90 + delta + 0.05)
    assert got == pytest.approx(KNEE_RATIO_CPP[delta], abs=0.003)


def test_knee_ratio_of_the_shipped_example_and_special_cases():
    ratio = rs.knee_ratio_max(15, 20, 95, 95)
    assert ratio is not None and 1.37 <= ratio <= 1.41
    assert rs.knee_ratio_max(15, 15, 95, 95) == pytest.approx(1.0, abs=0.01)
    assert rs.knee_ratio_max(0, 0, 0, 0) == 1.0
    assert rs.knee_ratio_max(15, 15, 300, 60) is None


def test_load_reads_servo_knee_ratio():
    assert rs.load(rs.CONFIG)['servo_knee_ratio'] == 1.0


def test_knee_ratio_warning_in_validate_and_check(cfg, capsys):
    v = rs.load(cfg)
    v.update({'calf_servo_arm_mm': 15, 'calf_joint_arm_mm': 20, 'calf_rod_mm': 95,
              'calf_axis_distance_mm': 95})
    msgs = rs.validate(v)[0]
    assert any(lv == 'warn' and 'servo.knee_ratio' in t and 'меньше вычисленного' in t for lv, t in msgs)
    assert any(lv == 'info' and 'тяга колена' in t for lv, t in msgs)
    assert not any(lv == 'warn' and 'servo.knee_ratio' in t
                   for lv, t in rs.validate(dict(v, servo_knee_ratio=1.45))[0])
    without = dict(v)
    without.pop('servo_knee_ratio')
    assert not any(lv == 'warn' and 'servo.knee_ratio' in t for lv, t in rs.validate(without)[0])
    assert not any(lv == 'warn' and 'servo.knee_ratio' in t for lv, t in rs.validate(rs.load(rs.CONFIG))[0])
    rs.save(cfg, v)
    capsys.readouterr()
    assert rs.main(['--check', '--config', cfg]) == 0
    assert 'servo.knee_ratio' in capsys.readouterr().out


def test_sensor_geometry_is_checked(cfg):
    v = rs.load(cfg)
    msgs = [t for lv, t in rs.validate(v)[0] if lv == 'info']
    assert any(t.startswith('линия GS2') for t in msgs) and any(t.startswith('лидары: крест') for t in msgs)
    v['gs2_pitch_deg'] = 10  # nearly flat: the line is beyond the 0.3 m range
    assert any(lv == 'warn' and 'GS2' in t for lv, t in rs.validate(v)[0])
    v['gs2_pitch_deg'], v['tof_fl_pitch_deg'] = 40, 2
    assert any(lv == 'warn' and 'ToF FL' in t for lv, t in rs.validate(v)[0])


def test_sensor_errors_do_not_block_save_and_check_while_the_sensors_are_not_on_the_robot(cfg):
    v = rs.load(cfg)
    v.update({'thigh': 130, 'calf': 130, 'stand_height': 200, 'max_height': 205})
    msgs = rs.validate(v)[0]
    assert not [m for m in msgs if m[0] == 'error'], msgs
    assert any(lv == 'warn' and 'GS2 не достаёт до пола' in t and 'датчика нет на роботе' in t for lv, t in msgs)
    assert rs.save(cfg, v) != '(ничего не изменилось)'
    assert rs.main(['--check', '--config', cfg]) == 0


def test_sensor_errors_block_for_sensors_on_the_robot(cfg, monkeypatch):
    monkeypatch.setattr(rs, 'SENSORS_ON_ROBOT', frozenset({'gs2', 'tof', 'x_lidar'}))
    v = rs.load(cfg)
    v.update({'thigh': 130, 'calf': 130, 'stand_height': 200, 'max_height': 205,
              'gs2_pitch_deg': 10, 'tof_fl_pitch_deg': 2, 'x_lidar_tilt_deg': 0})
    msgs = rs.validate(v)[0]
    assert any(lv == 'error' and 'GS2 не достаёт до пола' in t for lv, t in msgs)
    assert any(lv == 'error' and 'ToF FL' in t for lv, t in msgs)
    assert any(lv == 'error' and 'лидары: плоскость не пересекает пол' in t for lv, t in msgs)


def test_sensor_checks_are_skipped_when_the_flag_is_off():
    v = rs.load(rs.CONFIG)
    v.update({'gs2': 0, 'tof': 0, 'x_lidar': 0, 'stand_height': 200})
    msgs = [t for _, t in rs.validate(v)[0]]
    assert not any('GS2' in t or 'ToF' in t or 'лидары' in t for t in msgs)


def test_shipped_knee_is_coupled_and_validates_without_warnings():
    v = rs.load(rs.CONFIG)
    assert v['calf_coupled'] == 1
    msgs, _ = rs.validate(v)
    assert not [m for m in msgs if m[0] in ('error', 'warn')], msgs
    assert any(lv == 'info' and 'coupled_to' in t for lv, t in msgs)


def test_editing_calf_max_keeps_the_coupling_and_the_window(cfg):
    servos_p = os.path.join(cfg, 'servos.yaml')
    before = yaml.safe_load(open(servos_p))['/**/servo_driver']['ros__parameters']
    v = rs.load(cfg)
    v['calf_max'] = -45.0
    rs.save(cfg, v)
    after = yaml.safe_load(open(servos_p))['/**/servo_driver']['ros__parameters']
    for leg in ('lf', 'rf', 'lr', 'rr'):
        old, new = before['%s_calf_joint' % leg], after['%s_calf_joint' % leg]
        assert (new['min_deg'], new['max_deg']) == (-140.0, -45.0)
        for key in ('coupled_to', 'coupling', 'offset_deg', 'range_deg', 'pulse_min_us', 'pulse_max_us',
                    'servo_arm_mm', 'direction', 'channel'):
            assert new[key] == old[key], (leg, key)
    robot = yaml.safe_load(open(os.path.join(cfg, 'robot.yaml')))['/**']['ros__parameters']
    assert robot['description']['calf_limits_deg'] == [-140.0, -45.0]


# --- CAL-06: the calibration checks -------------------------------------------

def _set_offset(cfg, joint, offset, direction=None):
    path = os.path.join(cfg, 'servos.yaml')
    text = open(path).read()
    pattern = re.compile(r'(%s:\s*\{[^}]*?offset_deg:\s*)([-+0-9.]+)' % joint)
    assert pattern.search(text), joint
    text = pattern.sub(lambda m: m.group(1) + str(offset), text, count=1)
    if direction is not None:
        text = re.sub(r'(%s:\s*\{[^}]*?direction:\s*)([-+0-9]+)' % joint,
                      lambda m: m.group(1) + str(direction), text, count=1)
    open(path, 'w').write(text)


def test_servo_check_reports_the_pairs_that_drifted(cfg):
    v = rs.load(cfg)
    msgs = rs.check_servos(cfg, v)
    texts = ' | '.join(t for _, t in msgs)
    assert 'разброс смещений' in texts
    # The shipped file already carries a real asymmetry, so the check must see it.
    assert any(lv == 'warn' and 'разошлись' in t for lv, t in msgs), msgs


def test_servo_check_accepts_a_symmetric_calibration(cfg):
    for joint, offset in (('lf_thigh_joint', 30.0), ('rf_thigh_joint', -30.0),
                          ('lr_thigh_joint', 30.0), ('rr_thigh_joint', -30.0)):
        _set_offset(cfg, joint, offset)
    for joint, direction in (('lf_thigh_joint', 1), ('rf_thigh_joint', -1)):
        _set_offset(cfg, joint, 30.0 if joint.startswith('lf') else -30.0, direction)
    msgs = rs.check_servos(cfg, rs.load(cfg))
    thigh = [t for lv, t in msgs if t.startswith('thigh')]
    assert not [t for t in thigh if 'разошлись' in t], thigh


def test_servo_check_flags_one_direction_on_mirrored_servos(cfg):
    _set_offset(cfg, 'rf_thigh_joint', -56.0, direction=1)   # same sign as lf now
    msgs = rs.check_servos(cfg, rs.load(cfg))
    assert any('одинаковое направление' in t for _, t in msgs), msgs


def test_servo_check_flags_a_servo_at_the_edge_in_the_stand_pose(cfg):
    # The stand pulse is 1370 us at offset 0 and moves ~9.4 us per degree, so an
    # offset of 85 deg lands inside the 5 % band at the low edge (below 605 us).
    _set_offset(cfg, 'lf_hip_joint', 85.0)
    msgs = rs.check_servos(cfg, rs.load(cfg))
    assert any('у края' in t for _, t in msgs), msgs
