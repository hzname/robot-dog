#!/usr/bin/env python3
"""robot_setup: enter the measured robot parameters (docs/DEPLOYMENT.md, stage 1)
and save them into robot.yaml / servos.yaml.

  python3 tools/robot_setup/robot_setup.py            # web form on http://localhost:8765
  python3 tools/robot_setup/robot_setup.py --cli      # text mode (e.g. over SSH on the robot)
  python3 tools/robot_setup/robot_setup.py --check    # only validate the current files

Values are entered in millimetres, grams and degrees and stored in the units of
the config files (metres, kilograms, degrees). Only the values are changed:
comments and layout of the files stay as they are; the previous version is
kept as *.bak. Needs Python 3.8+ and PyYAML.
"""

import argparse
import difflib
import http.server
import json
import math
import os
import re
import shutil
import sys
import webbrowser

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
CONFIG = os.path.join(REPO, 'ros2_ws', 'src', 'dog_bringup', 'config')
IMG = os.path.join(REPO, 'docs', 'img')
LEGS = ('lf', 'rf', 'lr', 'rr')
KINDS = ('hip', 'thigh', 'calf')
TOF = ('fl', 'fr', 'fc', 'rc')  # order of sensors.tof_names

# ------------------------------------------------------------------ the form
# (id, label, unit, section, key, scale file->form, min, max, help)
GROUPS = [
    ('leg', 'Нога: длины звеньев', ['measure_leg_side.svg', 'measure_leg_rear.svg'], [
        ('thigh', 'thigh — ось бедра → ось колена', 'мм', 'geometry', 'thigh', 1000, 40, 250,
         'от центра оси бедра (вал сервы thigh) до центра оси колена'),
        ('calf', 'calf — ось колена → точка касания', 'мм', 'geometry', 'calf', 1000, 40, 250,
         'от центра оси колена до точки, которой стопа касается пола'),
        ('hip_offset', 'hip_offset — ось отведения → плоскость ноги', 'мм', 'geometry', 'hip_offset', 1000, 0, 120,
         'поперёк корпуса: от оси отведения до средней плоскости бедра и голени'),
        ('knee_direction', 'Колено сгибается', '', 'geometry', 'knee_direction', 1, -1, 1,
         '-1 = назад (как в v1 и v2), +1 = вперёд'),
    ]),
    ('body', 'Корпус: где стоят ноги', ['measure_body_top.svg'], [
        ('hip_x', 'hip_x — центр → оси передних ног', 'мм', 'geometry', 'hip_x', 1000, 30, 300,
         'половина расстояния между осями бедра передних и задних ног'),
        ('hip_y', 'hip_y — центр → ось отведения', 'мм', 'geometry', 'hip_y', 1000, 15, 150,
         'половина расстояния между осями отведения левых и правых ног'),
        ('body_length', 'Длина корпуса', 'мм', 'description', 'body_length', 1000, 50, 500, 'для модели'),
        ('body_width', 'Ширина корпуса', 'мм', 'description', 'body_width', 1000, 30, 300, 'для модели'),
        ('body_height', 'Высота корпуса', 'мм', 'description', 'body_height', 1000, 10, 200, 'для модели'),
        ('foot_radius', 'Радиус стопы', 'мм', 'description', 'foot_radius', 1000, 0, 40,
         'резиновый наконечник; для модели'),
    ]),
    ('stance', 'Стойка', ['measure_leg_side.svg'], [
        ('stand_height', 'stand_height — ось бедра над полом в стойке', 'мм', 'stance', 'stand_height', 1000, 50, 400,
         'выбирается так, чтобы колено было согнуто на 60–100°; см. подсказку справа'),
        ('lie_height', 'Высота «лёжа»', 'мм', 'stance', 'lie_height', 1000, 20, 300, 'корпус почти касается пола'),
        ('min_height', 'Ниже всего с пульта', 'мм', 'stance', 'min_height', 1000, 30, 400, ''),
        ('max_height', 'Выше всего с пульта', 'мм', 'stance', 'max_height', 1000, 50, 450, 'меньше, чем thigh + calf'),
    ]),
    ('mass', 'Массы (весы)', [], [
        ('body_mass', 'Корпус + электроника + батарея + 4 сервы hip', 'г', 'description', 'body_mass', 1000, 100, 5000, ''),
        ('hip_mass', 'Звено hip одной ноги (с сервой бедра)', 'г', 'description', 'hip_mass', 1000, 5, 500, ''),
        ('thigh_mass', 'Бедро одной ноги (с сервой колена)', 'г', 'description', 'thigh_mass', 1000, 5, 500, ''),
        ('calf_mass', 'Голень одной ноги', 'г', 'description', 'calf_mass', 1000, 5, 500, ''),
        ('total_mass', 'Робот целиком на весах (для сверки)', 'г', None, None, 1, 0, 10000,
         'не сохраняется: сравнивается с суммой масс выше'),
    ]),
    ('limits', 'Пределы суставов (до упора минус 5°)', ['measure_leg_side.svg', 'measure_leg_rear.svg'], [
        ('hip_out', 'hip: наружу от вертикали', '°', None, None, 1, 5, 90, 'левая нога — влево, правая — вправо'),
        ('hip_in', 'hip: внутрь (под корпус)', '°', None, None, 1, 0, 90, ''),
        ('thigh_min', 'thigh: минимум (стопа вперёд)', '°', None, None, 1, -135, 90, 'углы по схеме «нога сбоку»'),
        ('thigh_max', 'thigh: максимум (стопа назад)', '°', None, None, 1, -90, 180, ''),
        ('calf_min', 'calf: минимум (колено согнуто сильнее всего)', '°', None, None, 1, -180, 0, 'отрицательный'),
        ('calf_max', 'calf: максимум (колено разогнуто)', '°', None, None, 1, -170, 30, ''),
    ]),
    ('link', 'Привод через тягу (0 = серва на оси сустава)', ['measure_linkage.svg'], [
        (f'{k}_{f}', f'{k}: {lab}', 'мм', None, None, 1, 0, 300, '')
        for k in KINDS for f, lab in (('servo_arm_mm', 'рычаг сервы'), ('joint_arm_mm', 'рычаг сустава (0 = как у сервы)'),
                                      ('rod_mm', 'тяга (0 = как расстояние между осями)'),
                                      ('axis_distance_mm', 'ось сервы ↔ ось сустава'))
    ] + [('calf_coupled', 'Тяга колена опирается на корпус (серва колена в блоке бедра)', '', None, None, 1, 0, 1,
          'тогда серва задаёт сумму «колено + бедро»: coupled_to = <нога>_thigh_joint')]),
    ('sensors', 'Датчики (docs/HEAD.md): положение от центра корпуса', ['measure_sensors_top.svg',
                                                                         'measure_sensors_side.svg'], [
        ('x_lidar', 'Лидары «крестом» стоят (1 — да, 0 — нет)', '', 'sensors', 'x_lidar', 1, 0, 1, ''),
        ('x_lidar_x', 'Лидары: x — вперёд от центра', 'мм', 'sensors', 'x_lidar_x', 1000, -200, 300, ''),
        ('x_lidar_y', 'Лидары: y — влево (правый зеркально)', 'мм', 'sensors', 'x_lidar_y', 1000, 0, 150, ''),
        ('x_lidar_z', 'Лидары: z — луч над осями бедра', 'мм', 'sensors', 'x_lidar_z', 1000, -100, 200, ''),
        ('x_lidar_tilt_deg', 'Лидары: плоскость наклонена вниз, α', '°', 'sensors', 'x_lidar_tilt_deg', 1, 0, 60,
         '30° по умолчанию; см. HEAD.md'),
        ('x_lidar_yaw_deg', 'Лидары: направление наклона, β', '°', 'sensors', 'x_lidar_yaw_deg', 1, 0, 90,
         'правый наклонён влево, левый — вправо'),
        ('gs2', 'GS2 стоит (1 — да, 0 — нет)', '', 'sensors', 'gs2', 1, 0, 1, ''),
        ('gs2_x', 'GS2: x — вперёд от центра', 'мм', 'sensors', 'gs2_x', 1000, -200, 300, 'окно лазера'),
        ('gs2_y', 'GS2: y — влево', 'мм', 'sensors', 'gs2_y', 1000, -150, 150, ''),
        ('gs2_z', 'GS2: z — над осями бедра (ниже — минус)', 'мм', 'sensors', 'gs2_z', 1000, -150, 150, ''),
        ('gs2_pitch_deg', 'GS2: наклон вниз', '°', 'sensors', 'gs2_pitch_deg', 1, 0, 80, 'линия должна лечь '
         'на пол перед передними стопами, но ближе 0.3 м от датчика'),
        ('tof', 'VL53L1X стоят (1 — да, 0 — нет)', '', 'sensors', 'tof', 1, 0, 1, 'четыре: FL, FR, FC, RC'),
    ] + [
        (f'tof_{n}_{a}', f'ToF {n.upper()}: {lab}', u, 'sensors', f'tof_{a}[{k}]', sc, lo, hi, '')
        for k, n in enumerate(TOF)
        for a, lab, u, sc, lo, hi in (('x', 'x', 'мм', 1000, -300, 300), ('y', 'y', 'мм', 1000, -150, 150),
                                      ('z', 'z', 'мм', 1000, -150, 150), ('pitch_deg', 'наклон вниз', '°', 1, 0, 89),
                                      ('yaw_deg', 'поворот влево', '°', 1, -180, 180))
    ]),
    ('perception', 'Восприятие и реакция на препятствия (docs/PERCEPTION.md)', [], [
        ('threshold', 'Порог лидаров: выше / ниже пола', 'мм', 'perception', 'threshold', 1000, 5, 60, ''),
    ] + [
        (f'tof_offset_{n}', f'Поправка ToF {n.upper()} (замер на ровном полу)', 'мм', 'perception',
         f'tof_offsets[{k}]', 1000, -60, 60, 'измерено минус ожидалось; DEPLOYMENT.md, этап 14')
        for k, n in enumerate(TOF)
    ] + [
        ('guard', 'Реакция на препятствия включена (1/0)', '', 'perception', 'guard', 1, 0, 1,
         'замедление, высокий шаг, остановка'),
        ('guard_stop_dist', 'Остановка: центр корпуса не ближе', 'мм', 'perception', 'guard_stop_dist', 1000, 150, 1000,
         '300 мм = стопы примерно в 0.2 м от препятствия'),
        ('guard_climb_max', 'Перешагивать уступы до', 'мм', 'perception', 'guard_climb_max', 1000, 0, 80,
         'выше — остановка'),
        ('guard_max_step', 'Самый высокий шаг', 'мм', 'perception', 'guard_max_step', 1000, 10, 50,
         'выше 30 мм рысь раскачивается и робот опрокидывается (TERRAIN.md)'),
    ]),
]
FIELDS = {f[0]: f for g in GROUPS for f in g[3]}
BOOL = {'x_lidar', 'gs2', 'tof', 'guard'}
LIST_RE = re.compile(r'^(\w+)\[(\d+)\]$')


# ------------------------------------------------------------------ files
def paths(config_dir):
    return os.path.join(config_dir, 'robot.yaml'), os.path.join(config_dir, 'servos.yaml')


def load(config_dir):
    """Current values of every form field (form units)."""
    robot_p, servos_p = paths(config_dir)
    r = yaml.safe_load(open(robot_p))['/**']['ros__parameters']
    s = yaml.safe_load(open(servos_p))['/**/servo_driver']['ros__parameters']
    v = {}
    for fid, f in FIELDS.items():
        if f[3]:
            val = _get(r.get(f[3], {}), f[4])
            if isinstance(val, bool):
                v[fid] = int(val)
            elif val is not None:
                v[fid] = round(val * f[5], 3)
    lf = {k: s[f'lf_{k}_joint'] for k in KINDS}
    rf_hip = s['rf_hip_joint']
    v['hip_out'], v['hip_in'] = lf['hip']['max_deg'], -lf['hip']['min_deg']
    if rf_hip.get('min_deg') is not None:  # right leg: outward is negative
        v['hip_out'] = max(v['hip_out'], -rf_hip['min_deg'])
    v['thigh_min'], v['thigh_max'] = lf['thigh']['min_deg'], lf['thigh']['max_deg']
    v['calf_min'], v['calf_max'] = lf['calf']['min_deg'], lf['calf']['max_deg']
    for k in KINDS:
        for f in ('servo_arm_mm', 'joint_arm_mm', 'rod_mm', 'axis_distance_mm'):
            v[f'{k}_{f}'] = float(lf[k].get(f, 0.0) or 0.0)
    v['calf_coupled'] = 1 if lf['calf'].get('coupled_to') else 0
    names = r.get('sensors', {}).get('tof_names')
    if names is not None and list(names) != list(TOF):
        raise SystemExit(f'sensors.tof_names = {names}: форма рассчитана на {list(TOF)}')
    v['total_mass'] = 0
    return v


def _get(section, key):
    """section[key], or one element for keys like 'tof_x[2]'."""
    m = LIST_RE.match(key)
    if not m:
        return section.get(key)
    lst = section.get(m.group(1))
    return lst[int(m.group(2))] if isinstance(lst, list) and int(m.group(2)) < len(lst) else None


def fmt(x):
    """Number as the config files write it: up to 4 decimals, at least one."""
    s = f'{x:.4f}'.rstrip('0')
    return s + '0' if s.endswith('.') else s


def _same(old, new):
    try:
        return float(old) == float(new)
    except ValueError:
        return old.strip() == new.strip()


def set_nested(lines, section, key, value):
    """Set `section: {key: value}` under /**/ros__parameters, keeping comments."""
    sec_re = re.compile(rf'^(\s*){re.escape(section)}:\s*(#.*)?$')
    i = next((k for k, ln in enumerate(lines) if sec_re.match(ln)), None)
    if i is None:
        raise KeyError(f'section {section!r} not found')
    base = len(sec_re.match(lines[i]).group(1))
    key_re = re.compile(rf'^(\s*){re.escape(key)}:(\s*)([^#\n]*?)(\s*#.*)?$')
    for k in range(i + 1, len(lines)):
        ln = lines[k]
        if ln.strip() and not ln.lstrip().startswith('#') and len(ln) - len(ln.lstrip()) <= base:
            break
        m = key_re.match(ln.rstrip('\n'))
        if m:
            old = m.group(3)
            if _same(old, value) or (old.startswith('[') and value.startswith('[') and
                                     [float(x) for x in old.strip('[]').split(',')] ==
                                     [float(x) for x in value.strip('[]').split(',')]):
                return
            new = value if len(value) >= len(old) else value.ljust(len(old))
            lines[k] = f'{m.group(1)}{key}:{m.group(2)}{new}{m.group(4) or ""}\n'
            return
    raise KeyError(f'{section}.{key} not found')


def set_flow(line, fields):
    """Set keys inside a one-line flow map `name: {a: 1, b: 2}`; adds missing keys."""
    for key, value in fields.items():
        pat = re.compile(rf'(\b{re.escape(key)}:\s*)([^,}}]+?)(\s*[,}}])')
        found = pat.search(line)
        if found:
            if not _same(found.group(2), value):
                line = pat.sub(lambda m: m.group(1) + value.rjust(len(m.group(2))) + m.group(3), line, count=1)
        else:
            line = re.sub(r'\s*}\s*$', f', {key}: {value}}}\n', line.rstrip('\n'), count=1) \
                if line.rstrip().endswith('}') else line
            if not line.endswith('\n'):
                line += '\n'
    return line


def remove_flow(line, key):
    return re.sub(rf',\s*{re.escape(key)}:\s*[^,}}]+', '', line)


# ------------------------------------------------------------------ checks
def stand_angles(thigh, calf, h):
    """Stand pose with the foot under the thigh axis (as dog_control): (thigh, calf) [deg]."""
    c = (h * h - thigh * thigh - calf * calf) / (2 * thigh * calf)
    if abs(c) > 1:
        return None
    q2 = -math.acos(c)
    q1 = math.atan2(-calf * math.sin(q2), thigh + calf * math.cos(q2))
    return math.degrees(q1), math.degrees(q2)


def linkage_closes(a, b, c, d):
    """Four-bar at the servo centre: servo arm a perpendicular to the axis line d,
    joint arm b, rod c. True if the rod can connect the two arms."""
    if a <= 0:
        return True
    if d <= 0:
        return False
    b = b or a
    c = c or d
    e = math.hypot(d, a)  # servo pin -> joint axis
    return abs(b - c) <= e <= b + c


def validate(v):
    """[(level, text)] with level 'error' | 'warn' | 'info', and derived numbers."""
    out, info = [], {}
    for fid, f in FIELDS.items():
        x = v.get(fid)
        if x is None or (isinstance(x, str) and not x.strip()):
            out.append(('error', f'не заполнено: {f[1]}'))
            continue
        try:
            x = float(x)
        except ValueError:
            out.append(('error', f'не число: {f[1]}'))
            continue
        if fid == 'total_mass' and x == 0:
            continue
        if not (f[6] <= x <= f[7]):
            out.append(('error', f'{f[1]}: {x:g} {f[2]} вне {f[6]}…{f[7]}'))
    if any(lv == 'error' for lv, _ in out):
        return out, info
    g = {k: float(v[k]) for k in FIELDS}
    th, cf = g['thigh'], g['calf']
    reach, short = th + cf, abs(th - cf)
    info['reach_mm'] = reach
    if int(g['knee_direction']) not in (-1, 1):
        out.append(('error', 'knee_direction: только -1 или +1'))
    for name in ('stand_height', 'lie_height', 'min_height', 'max_height'):
        if not (short + 10 <= g[name] <= reach - 5):
            out.append(('error', f'{name} = {g[name]:g} мм: нога достаёт только от {short + 10:g} до {reach - 5:g} мм'))
    if not (g['lie_height'] < g['min_height'] <= g['stand_height'] <= g['max_height']):
        out.append(('error', 'нужно: лёжа < минимум ≤ стойка ≤ максимум'))
    ang = stand_angles(th, cf, g['stand_height'])
    if ang:
        q1, q2 = ang
        info['stand_thigh_deg'], info['stand_calf_deg'] = round(q1, 1), round(q2, 1)
        knee = 180 + q2
        info['knee_inner_deg'] = round(knee, 1)
        out.append(('info', f'в стойке: thigh {q1:+.0f}°, calf {q2:+.0f}° (угол в колене {knee:.0f}°)'))
        if not (60 <= -q2 <= 110):
            out.append(('warn', f'в стойке колено согнуто на {-q2:.0f}° (лучше 60–100°): '
                                f'поменяйте stand_height (при calf −90° это {math.hypot(th, cf):.0f} мм)'))
        if not (g['thigh_min'] + 3 <= q1 <= g['thigh_max'] - 3):
            out.append(('error', f'угол бедра в стойке {q1:+.0f}° вне пределов thigh {g["thigh_min"]:g}…{g["thigh_max"]:g}°'))
        if not (g['calf_min'] + 3 <= q2 <= g['calf_max'] - 3):
            out.append(('error', f'угол колена в стойке {q2:+.0f}° вне пределов calf {g["calf_min"]:g}…{g["calf_max"]:g}°'))
    if g['thigh_min'] >= g['thigh_max'] or g['calf_min'] >= g['calf_max']:
        out.append(('error', 'пределы: минимум должен быть меньше максимума'))
    if g['hip_out'] < 15:
        out.append(('warn', 'hip наружу меньше 15°: шаг вбок и развороты будут упираться'))
    if g['hip_x'] > g['body_length'] / 2 + 30:
        out.append(('warn', 'hip_x больше половины длины корпуса: проверьте, это половина расстояния перёд–зад'))
    if g['hip_y'] > g['body_width'] / 2 + 60:
        out.append(('warn', 'hip_y сильно больше половины ширины корпуса: проверьте, это до оси отведения'))
    m = g['body_mass'] + 4 * (g['hip_mass'] + g['thigh_mass'] + g['calf_mass'])
    info['model_mass_g'] = round(m)
    out.append(('info', f'масса по модели {m:.0f} г'))
    if g['total_mass'] > 0 and abs(m - g['total_mass']) > 0.08 * g['total_mass']:
        out.append(('warn', f'сумма масс {m:.0f} г отличается от веса робота {g["total_mass"]:.0f} г больше чем на 8 %'))
    for k in KINDS:
        a, b, c, d = (g[f'{k}_{f}'] for f in ('servo_arm_mm', 'joint_arm_mm', 'rod_mm', 'axis_distance_mm'))
        if a > 0 and d <= 0:
            out.append(('error', f'{k}: при тяге нужно расстояние между осями'))
        elif not linkage_closes(a, b, c, d):
            out.append(('error', f'{k}: тяга не замыкается (рычаги {a:g}/{b or a:g}, тяга {c or d:g}, оси {d:g} мм)'))
        elif a > 0 and (b or a) < a:
            out.append(('info', f'{k}: рычаг сустава короче рычага сервы — ход сустава больше хода сервы'))
    if int(g['calf_coupled']) and g['calf_servo_arm_mm'] <= 0:
        out.append(('warn', 'колено «от корпуса» обычно бывает только с тягой (servo_arm_mm > 0)'))
    sensor_checks(g, out, info)
    return out, info


def _rot(roll, pitch, yaw):
    """URDF rpy -> 3x3 (as dog_description / dog_perception)."""
    cr, sr, cp, sp, cy, sy = (f(math.radians(a)) for a in (roll, pitch, yaw) for f in (math.cos, math.sin))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _floor_hit(p, R, a, floor_z):
    """Where the ray at angle a in the sensor's x-y plane meets the floor z = floor_z: (range, x, y) or None."""
    u = [R[i][0] * math.cos(a) + R[i][1] * math.sin(a) for i in range(3)]
    if u[2] >= -1e-6:
        return None
    t = (floor_z - p[2]) / u[2]
    return (t, p[0] + t * u[0], p[1] + t * u[1]) if t > 0 else None


def sensor_checks(g, out, info):
    """Where the sensors meet the floor in the stand pose (all in mm, body frame)."""
    floor = -g['stand_height']
    foot_x, foot_y = g['hip_x'], g['hip_y'] + g['hip_offset']
    if int(g['gs2']):
        p, R = (g['gs2_x'], g['gs2_y'], g['gs2_z']), _rot(0, g['gs2_pitch_deg'], 0)
        hit = _floor_hit(p, R, 0.0, floor)
        if hit is None or hit[0] > 300:
            out.append(('error', 'GS2 не достаёт до пола в стойке (дальность 300 мм): увеличьте наклон вниз'))
        else:
            r, x, _ = hit
            half = math.acos(min(1.0, r / 300.0))
            w = r * math.tan(min(half, math.radians(50)))
            info['gs2_line_x_mm'] = round(x)
            out.append(('info', f'линия GS2: {x:.0f} мм от центра, {x - foot_x:.0f} мм перед стопами, '
                                f'ширина ±{w:.0f} мм, центральный луч {r:.0f} мм'))
            if r > 255:
                out.append(('warn', f'GS2: центральный луч {r:.0f} мм — при кивке корпуса пол уйдёт за 300 мм'))
            if x - foot_x < 40:
                out.append(('warn', 'GS2: линия ближе 40 мм к передним стопам — не успеет затормозить'))
            if w < foot_y + 30:
                out.append(('warn', f'GS2: линия ±{w:.0f} мм не накрывает линии стоп (±{foot_y:.0f} мм)'))
    if int(g['tof']):
        for n in TOF:
            p = (g[f'tof_{n}_x'], g[f'tof_{n}_y'], g[f'tof_{n}_z'])
            R = _rot(0, g[f'tof_{n}_pitch_deg'], g[f'tof_{n}_yaw_deg'])
            hit = _floor_hit(p, R, 0.0, floor)
            if hit is None or hit[0] > 1000:
                out.append(('error', f'ToF {n.upper()}: луч не попадает на пол ближе 1 м'))
                continue
            r, x, y = hit
            out.append(('info', f'ToF {n.upper()}: пятно на полу x {x:.0f}, y {y:+.0f} мм, дальность {r:.0f} мм'))
            if n in ('fl', 'fr') and abs(abs(y) - foot_y) > 40:
                out.append(('warn', f'ToF {n.upper()}: пятно в {abs(abs(y) - foot_y):.0f} мм от линии стопы'))
            if n != 'rc' and x < foot_x + 30:
                out.append(('warn', f'ToF {n.upper()}: пятно не впереди передних стоп'))
    if int(g['x_lidar']):
        crosses = []
        for side in (1, -1):
            p = (g['x_lidar_x'], side * g['x_lidar_y'], g['x_lidar_z'])
            R = _rot(0, g['x_lidar_tilt_deg'], -side * g['x_lidar_yaw_deg'])
            best = None
            for k in range(3600):
                hit = _floor_hit(p, R, math.radians(k / 10.0), floor)
                if hit and hit[1] > 0 and abs(hit[2]) < 3 and (best is None or hit[1] < best):
                    best = hit[1]
            crosses.append(best)
        if None in crosses:
            out.append(('error', 'лидары: плоскость не пересекает пол впереди на оси — увеличьте наклон α'))
        else:
            x = max(crosses)
            info['x_lidar_cross_mm'] = round(x)
            out.append(('info', f'лидары: крест на полу в {x:.0f} мм от центра ({x - foot_x:.0f} мм перед стопами)'))
            if x > 1200:
                out.append(('warn', 'лидары: крест дальше 1.2 м — узел берёт плоскость пола только до 1.2 м'))


# ------------------------------------------------------------------ save
def render(config_dir, v):
    """New text of robot.yaml and servos.yaml."""
    robot_p, servos_p = paths(config_dir)
    rl = open(robot_p).readlines()
    cur = yaml.safe_load(''.join(rl))['/**']['ros__parameters']
    lists = {}  # (section, key) -> list of values, for keys like 'tof_x[2]'
    for fid, f in FIELDS.items():
        if not f[3]:
            continue
        if fid in BOOL:
            set_nested(rl, f[3], f[4], 'true' if int(float(v[fid])) else 'false')
            continue
        val = float(v[fid]) / f[5]
        m = LIST_RE.match(f[4])
        if m:
            key = (f[3], m.group(1))
            if key not in lists:
                lists[key] = list(cur[f[3]][m.group(1)])
            lists[key][int(m.group(2))] = val
            continue
        set_nested(rl, f[3], f[4], str(int(val)) if fid == 'knee_direction' else fmt(val))
    for (section, key), vals in lists.items():
        set_nested(rl, section, key, '[' + ', '.join(fmt(float(x)) for x in vals) + ']')
    hmax = max(float(v['hip_out']), float(v['hip_in']))
    set_nested(rl, 'description', 'hip_limits_deg', f'[{-hmax:.1f}, {hmax:.1f}]')
    set_nested(rl, 'description', 'thigh_limits_deg', f'[{float(v["thigh_min"]):.1f}, {float(v["thigh_max"]):.1f}]')
    set_nested(rl, 'description', 'calf_limits_deg', f'[{float(v["calf_min"]):.1f}, {float(v["calf_max"]):.1f}]')
    sl = open(servos_p).readlines()
    for i, ln in enumerate(sl):
        m = re.match(r'\s*(lf|rf|lr|rr)_(hip|thigh|calf)_joint:\s*{', ln)
        if not m:
            continue
        leg, kind = m.groups()
        f = {}
        if kind == 'hip':  # joint convention: + = foot left; outward is + for left legs
            out_, in_ = float(v['hip_out']), float(v['hip_in'])
            lo, hi = (-in_, out_) if leg[0] == 'l' else (-out_, in_)
        else:
            lo, hi = float(v[f'{kind}_min']), float(v[f'{kind}_max'])
        f['min_deg'], f['max_deg'] = f'{lo:.1f}', f'{hi:.1f}'
        for key in ('servo_arm_mm', 'joint_arm_mm', 'rod_mm', 'axis_distance_mm'):
            val = float(v[f'{kind}_{key}'])
            if key == 'servo_arm_mm' or val > 0 or float(v[f'{kind}_servo_arm_mm']) > 0:
                f[key] = f'{val:.1f}'
        if kind == 'calf':
            if int(float(v['calf_coupled'])):
                f['coupled_to'] = f'{leg}_thigh_joint'
                f['coupling'] = '1.0'
            else:
                ln = remove_flow(remove_flow(ln, 'coupled_to'), 'coupling')
        sl[i] = set_flow(ln, f)
    return ''.join(rl), ''.join(sl)


def save(config_dir, v):
    robot_p, servos_p = paths(config_dir)
    new_r, new_s = render(config_dir, v)
    diff = ''
    for p, new in ((robot_p, new_r), (servos_p, new_s)):
        old = open(p).read()
        if old != new:
            diff += ''.join(difflib.unified_diff(old.splitlines(True), new.splitlines(True),
                                                 os.path.relpath(p, REPO), os.path.relpath(p, REPO)))
            shutil.copyfile(p, p + '.bak')
            open(p, 'w').write(new)
    for p in (robot_p, servos_p):  # must still be valid YAML
        yaml.safe_load(open(p))
    return diff or '(ничего не изменилось)'


NEXT = ('Дальше: проверить в симуляции, что робот с этими размерами ходит (DEPLOYMENT.md, этап 1):\n'
        '  cd ros2_ws && colcon build && source install/setup.bash\n'
        '  ros2 launch dog_gazebo sim.launch.py headless:=true web:=false &\n'
        '  ros2 run dog_gazebo walk_check\n'
        'и закоммитить robot.yaml и servos.yaml.')


# ------------------------------------------------------------------ CLI
def cli(config_dir):
    v = load(config_dir)
    print('Параметры робота (этап 1). Enter — оставить текущее значение, q — выйти без сохранения.\n'
          f'Схемы: {IMG}\n')
    for gid, title, imgs, fields in GROUPS:
        print(f'== {title}' + (f'   [схема: {", ".join(imgs)}]' if imgs else ''))
        for fid, label, unit, *_rest in fields:
            help_ = _rest[-1]
            while True:
                cur = v.get(fid, '')
                ans = input(f'  {label}{" (" + unit + ")" if unit else ""} [{cur}]{" — " + help_ if help_ else ""}: ').strip()
                if ans.lower() == 'q':
                    print('выход без сохранения')
                    return 1
                if not ans:
                    break
                try:
                    v[fid] = float(ans.replace(',', '.'))
                    break
                except ValueError:
                    print('    нужно число')
        print()
    msgs, _ = validate(v)
    for lv, t in msgs:
        print({'error': 'ОШИБКА', 'warn': 'внимание', 'info': '·'}[lv], t)
    if any(lv == 'error' for lv, _ in msgs):
        print('\nЕсть ошибки — не сохраняю. Запустите ещё раз.')
        return 2
    if input('\nСохранить в robot.yaml и servos.yaml? [y/N] ').strip().lower() not in ('y', 'д', 'yes', 'да'):
        return 1
    print(save(config_dir, v))
    print(NEXT)
    return 0


# ------------------------------------------------------------------ web
PAGE = r'''<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Параметры робота</title>
<style>
:root{--bg:#f4f6f8;--card:#fff;--ink:#12161b;--ink2:#48515e;--mut:#737d8a;--line:#d3d9e0;--acc:#2a78d6;--ok:#1d7a1d;--bad:#b3282a;--warn:#8a5a00}
@media (prefers-color-scheme:dark){:root{--bg:#0f1216;--card:#171b21;--ink:#e8ebef;--ink2:#b4bbc6;--mut:#8791a0;--line:#333a45;--acc:#5b9df0;--ok:#6fd46f;--bad:#f28b8b;--warn:#e8b64c}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,sans-serif;padding:24px 16px 80px}
main{max-width:1180px;margin:0 auto;display:flex;flex-direction:column;gap:22px}h1{margin:0;font-size:26px}p{margin:0;color:var(--ink2)}
section{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:18px;display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1.1fr);gap:20px}
section h2{grid-column:1/-1;margin:0;font-size:18px}.imgs img{width:100%;border:1px solid var(--line);border-radius:6px;background:#fff;margin-bottom:8px}
label{display:grid;grid-template-columns:1fr 110px 34px;gap:8px;align-items:center;padding:6px 0;border-bottom:1px solid var(--line)}
label small{display:block;color:var(--mut);font-size:12px}input{font:inherit;padding:5px 8px;border:1px solid var(--line);border-radius:6px;background:var(--bg);color:var(--ink);width:100%;text-align:right}
input:focus{outline:2px solid var(--acc)}.u{color:var(--mut);font-size:13px}
#bar{position:sticky;bottom:0;background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 16px;display:flex;flex-direction:column;gap:8px}
#msgs div{font-size:14px}.error{color:var(--bad);font-weight:600}.warn{color:var(--warn)}.info{color:var(--ink2)}
button{font:inherit;font-weight:600;padding:8px 18px;border-radius:8px;border:0;background:var(--acc);color:#fff;cursor:pointer}button:disabled{opacity:.4;cursor:default}
pre{white-space:pre-wrap;font-size:12px;background:var(--bg);padding:10px;border-radius:6px;max-height:300px;overflow:auto}
@media(max-width:860px){section{grid-template-columns:1fr}}
</style></head><body><main>
<h1>Параметры робота — этап 1</h1>
<p>Меряйте по схемам справа. Размеры в миллиметрах, массы в граммах, углы в градусах. Проверка идёт сразу; «Сохранить» пишет значения в robot.yaml и servos.yaml (комментарии сохраняются, старые версии — *.bak).</p>
<div id="form"></div>
<div id="bar"><div id="msgs"></div><div><button id="save">Сохранить в конфиг</button></div><pre id="diff" hidden></pre></div>
</main><script>
const G = __GROUPS__, V = __VALUES__;
const form = document.getElementById('form');
for (const [gid, title, imgs, fields] of G) {
  const s = document.createElement('section');
  s.innerHTML = `<h2>${title}</h2><div class="fields"></div><div class="imgs">${imgs.map(i => `<img src="img/${i}" alt="схема">`).join('')}</div>`;
  const box = s.querySelector('.fields');
  for (const f of fields) {
    const [id, label, unit, , , , mn, mx, help] = f;
    const l = document.createElement('label');
    l.innerHTML = `<span>${label}${help ? `<small>${help}</small>` : ''}</span><input id="${id}" inputmode="decimal" value="${V[id] ?? ''}" title="${mn}…${mx}"><span class="u">${unit}</span>`;
    box.appendChild(l);
  }
  if (!imgs.length) s.style.gridTemplateColumns = '1fr';
  form.appendChild(s);
}
const vals = () => Object.fromEntries([...document.querySelectorAll('input')].map(i => [i.id, i.value.replace(',', '.')]));
let t;
async function check() {
  const r = await (await fetch('api/validate', {method: 'POST', body: JSON.stringify(vals())})).json();
  document.getElementById('msgs').innerHTML = r.messages.map(([lv, t]) => `<div class="${lv}">${lv === 'error' ? '✘ ' : lv === 'warn' ? '⚠ ' : '· '}${t}</div>`).join('');
  document.getElementById('save').disabled = r.messages.some(m => m[0] === 'error');
}
form.addEventListener('input', () => { clearTimeout(t); t = setTimeout(check, 250); });
document.getElementById('save').onclick = async () => {
  const r = await (await fetch('api/save', {method: 'POST', body: JSON.stringify(vals())})).json();
  const d = document.getElementById('diff'); d.hidden = false; d.textContent = r.diff + '\n\n' + r.next;
};
check();
</script></body></html>'''


def serve(config_dir, port, open_browser):
    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                page = PAGE.replace('__GROUPS__', json.dumps(GROUPS, ensure_ascii=False)) \
                           .replace('__VALUES__', json.dumps(load(config_dir), ensure_ascii=False))
                return self._send(200, page, 'text/html; charset=utf-8')
            m = re.fullmatch(r'/img/([\w.-]+\.svg)', self.path)
            if m and os.path.exists(os.path.join(IMG, m.group(1))):
                return self._send(200, open(os.path.join(IMG, m.group(1)), 'rb').read(), 'image/svg+xml')
            self._send(404, 'not found', 'text/plain')

        def do_POST(self):
            v = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
            msgs, info = validate(v)
            if self.path == '/api/validate':
                return self._send(200, json.dumps({'messages': msgs, 'info': info}, ensure_ascii=False),
                                  'application/json')
            if self.path == '/api/save':
                if any(lv == 'error' for lv, _ in msgs):
                    return self._send(400, json.dumps({'diff': 'есть ошибки', 'next': ''}), 'application/json')
                diff = save(config_dir, {k: float(x) for k, x in v.items()})
                return self._send(200, json.dumps({'diff': diff, 'next': NEXT}, ensure_ascii=False),
                                  'application/json')
            self._send(404, 'not found', 'text/plain')

    srv = http.server.ThreadingHTTPServer(('127.0.0.1', port), H)
    url = f'http://localhost:{port}/'
    print(f'Форма параметров: {url}  (Ctrl-C — выход)')
    if open_browser:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default=CONFIG, help='folder with robot.yaml and servos.yaml')
    ap.add_argument('--cli', action='store_true', help='text mode instead of the web form')
    ap.add_argument('--check', action='store_true', help='only validate the current files')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--no-browser', action='store_true')
    a = ap.parse_args(argv)
    if a.check:
        msgs, _ = validate(load(a.config))
        for lv, t in msgs:
            print({'error': 'ОШИБКА', 'warn': 'внимание', 'info': '·'}[lv], t)
        return 2 if any(lv == 'error' for lv, _ in msgs) else 0
    if a.cli:
        return cli(a.config)
    return serve(a.config, a.port, not a.no_browser)


if __name__ == '__main__':
    sys.exit(main())
