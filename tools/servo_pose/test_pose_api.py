#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Макет-тест API страницы поз (без I2C): подстройка, инверсия, память."""
import json
import os
import sys
import urllib.request

BASE = 'http://127.0.0.1:8090'
ok = True


def check(cond, msg):
    global ok
    print(('OK   ' if cond else 'FAIL ') + msg)
    ok = ok and cond


def get(p):
    with urllib.request.urlopen(BASE + p, timeout=5) as r:
        return json.load(r)


def js(state, ch):
    return [x for x in state['joints'] if x['ch'] == ch][0]


s = get('/api/state')
check(not s['real'], 'режим макета (I2C нет)')
check(len(s['joints']) == 12, '12 суставов в состоянии')
j0 = js(s, 0)
check(j0['held_us'] is None and j0['trim_deg'] == 0.0 and j0['mount_us'] == 1370.0,
      'ch0 старт: отпущен, трим 0.0, цель 1370.0')

s = get('/api/nudge?ch=0&d=1')                      # LF hip, dir +1
j0 = js(s, 0)
check(j0['trim_deg'] == 1.0 and j0['mount_us'] == 1379.4,
      'ch0 (dir +1) +1°: трим +1.0, цель 1379.4 (импульс растёт)')
check(j0['held_us'] is not None and abs(j0['held_us'] - 1379.4) <= 2.5,
      'ch0 держится ~1379.4 мкс (тик 4.88 мкс): %s' % j0['held_us'])
check(j0['offset_deg'] == -1.0, 'ch0 offset_deg = 0 − 1 = −1.0')

s = get('/api/nudge?ch=3&d=1')                      # RF hip, dir −1
j3 = js(s, 3)
check(j3['trim_deg'] == 1.0 and j3['mount_us'] == 1360.6,
      'ch3 (dir −1) +1°: цель 1360.6 — импульс зеркально (инверсия)')
check(j3['held_us'] is not None and abs(j3['held_us'] - 1360.6) <= 2.5,
      'ch3 держится ~1360.6 мкс: %s' % j3['held_us'])

for _ in range(45):
    s = get('/api/nudge?ch=0&d=-1')
check(js(s, 0)['trim_deg'] == -30.0, 'ch0 предел подстройки снизу (−30°)')

s = get('/api/reset?ch=0')
j0 = js(s, 0)
check(j0['trim_deg'] == 0.0 and j0['mount_us'] == 1370.0, 'ch0 сброс подстройки -> 1370.0')

s = get('/api/set?ch=1')
j1 = js(s, 1)
check(j1['held_us'] is not None and abs(j1['held_us'] - j1['mount_us']) <= 2.5,
      'ch1 «держать» -> держится ~цель')
s = get('/api/release?ch=1')
check(js(s, 1)['held_us'] is None, 'ch1 «снять» -> отпущен')

s = get('/api/set?leg=LF')
check(all(js(s, c)['held_us'] is not None for c in (0, 1, 2)), '«держать лапу LF» -> все 3 канала')
s = get('/api/release?all=1')
check(all(js(s, x)['held_us'] is None for x in range(12)), '«снять всё» -> 12 отпущены')

e = get('/api/export')
check(len(e['joints']) == 12 and 'offset_deg' in e['joints'][0], 'экспорт: 12 суставов с offset_deg')

p = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'servo_pose_trim.json')
check(os.path.exists(p), 'servo_pose_trim.json создан')
data = json.load(open(p))
check(data['trims']['0'] == 0.0 and data['trims']['3'] == 1.0,
      'в файле: ch0=0.0, ch3=1.0 (память)')

# схемы-картинки
import urllib.error
with urllib.request.urlopen(BASE + '/', timeout=5) as r:
    html = r.read().decode('utf-8')
check('/img/assembly_pose.svg' in html, 'страница ссылается на assembly_pose.svg')
try:
    with urllib.request.urlopen(BASE + '/img/assembly_pose.svg', timeout=5) as r:
        code, body = r.status, r.read()
except urllib.error.HTTPError as e:
    code, body = e.code, b''
check(code == 200 and b'<svg' in body[:200], '/img/assembly_pose.svg отдаётся как svg')
try:
    with urllib.request.urlopen(BASE + '/img/nope.svg', timeout=5) as r:
        code = r.status
except urllib.error.HTTPError as e:
    code = e.code
check(code == 404, 'нет файла -> 404')
try:
    with urllib.request.urlopen(BASE + '/img/..%2F..%2Fetc%2Fpasswd', timeout=5) as r:
        code = r.status
except urllib.error.HTTPError as e:
    code = e.code
check(code == 404, 'обход пути заблокирован')
check('class="layout"' in html, 'двухколоночная раскладка (управление + схема справа)')
check('details class="imgs" open' in html, 'схема открыта (правая колонка)')
check(html.index('class="right"') > html.index('id="box"'), 'схема в .right после блока кнопок')
check(html.count('<div class="btns">') == 2, 'кнопки в два ряда (держать/снять; −1°/+1°)')
check('/img/measure_leg_side.svg' not in html and '/img/measure_leg_rear.svg' not in html,
      'схемы измерений убраны (осталась только монтажная поза)')

# кнопка «перенести в servos.yaml» (пишем в тестовую копию, не в репо)
import re as _re
for ch in range(12):
    get('/api/reset?ch=%d' % ch)
get('/api/nudge?ch=1&d=1'); get('/api/nudge?ch=1&d=1')
get('/api/nudge?ch=2&d=-1')
d = get('/api/apply?dry=1')
check(d['ok'] and d['dry'] and len(d['changed']) == 2, 'apply dry: 2 изменения')
dr = get('/api/apply')
check(dr['ok'] and not dr['dry'] and len(dr['changed']) == 2, 'apply: записано 2 изменения')
p2 = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'servos_apply_test.yaml')
text = open(p2).read()
def off(name):
    m = _re.search(name + r':\s+\{.*?offset_deg:\s*(-?[\d.]+)', text)
    return m.group(1) if m else '?'
check(off('lf_thigh_joint') == '43.0' and off('lf_calf_joint') == '-89.0'
      and off('lf_hip_joint') == '0.0',
      'в servos.yaml: lf_thigh 43.0, lf_calf −89.0, lf_hip 0.0 (получено %s/%s/%s)'
      % (off('lf_thigh_joint'), off('lf_calf_joint'), off('lf_hip_joint')))
check(os.path.exists(p2 + '.bak'), 'бэкап .bak создан')
d2 = get('/api/apply')
check(d2['ok'] and not d2['changed'], 'повторный apply: менять нечего')

print('ИТОГ:', 'PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
