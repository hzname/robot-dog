import math
import os
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


def test_sensor_geometry_is_checked(cfg):
    v = rs.load(cfg)
    msgs = [t for lv, t in rs.validate(v)[0] if lv == 'info']
    assert any(t.startswith('линия GS2') for t in msgs) and any(t.startswith('лидары: крест') for t in msgs)
    v['gs2_pitch_deg'] = 10  # nearly flat: the line is beyond the 0.3 m range
    assert any(lv == 'error' and 'GS2' in t for lv, t in rs.validate(v)[0])
    v['gs2_pitch_deg'], v['tof_fl_pitch_deg'] = 40, 2
    assert any(lv == 'error' and 'ToF FL' in t for lv, t in rs.validate(v)[0])
