#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Страница выставления монтажных положений серв (v2) с подстройкой.

Запуск:   python3 servo_pose_server.py [порт=8080]
Страница: http://<хост>:8080/
API:      /api/state
          /api/nudge?ch=N&d=1|-1     подстройка ±1 шаг (шаг 1° сустава)
          /api/reset?ch=N            сброс подстройки канала
          /api/set?ch=N | ?leg=LF    держать (импульс = центре + подстройка)
          /api/release?ch=N | ?leg=… / ?all=1
          /api/export                offset_deg для servos.yaml

Прямые записи в PCA9685 (/dev/i2c-0 @0x40) — как драйвер проекта:
канал N = регистры 0x06+4N, [ON=0, OFF=тики], «отпустить» = бит full-off.
Если I2C недоступен (ПК/WSL) — режим макета: страницу можно смотреть и щёлкать.

Подстройка «помнится» в servo_pose_trim.json и переносится в offset_deg
servos.yaml скриптом servo_pose_apply.py: offset_deg = опорный_угол −
подстройка (hip 0°, thigh +45°, calf −90°). Шаг применяется в системе углов
сустава, поэтому у правых серв (direction −1) импульс меняется зеркально —
это и есть проверка инверсирования.
"""
import fcntl
import json
import os
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

try:
    sys.stdout.reconfigure(line_buffering=True)    # живой лог при redirect в файл
except Exception:                                  # noqa: BLE001
    pass

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080

CENTER_US = 1370.0
US_PER_DEG = (2220.0 - 520.0) / 180.0      # как в servos.yaml: 9.444 мкс/°
STEP_DEG = 1.0                             # шаг подстройки, ° сустава
TRIM_LIMIT_DEG = 10.0                      # предел подстройки, °
I2C_SLAVE = 0x0703
ADDR = 0x40
DEV = '/dev/i2c-0'
HZ = 25e6 / (4096.0 * 122.0)               # прескейл 121 -> 50.0288 Гц
PERIOD_US = 1e6 / HZ

TRIM_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'servo_pose_trim.json')

# Суставы: канал -> (нога, сустав, имя в servos.yaml, direction, опорный угол°)
JOINTS = [
    (0,  'LF', 'hip',   'lf_hip_joint',   +1,   0.0),
    (1,  'LF', 'thigh', 'lf_thigh_joint', +1,  45.0),
    (2,  'LF', 'calf',  'lf_calf_joint',  +1, -90.0),
    (3,  'RF', 'hip',   'rf_hip_joint',   -1,   0.0),
    (4,  'RF', 'thigh', 'rf_thigh_joint', -1,  45.0),
    (5,  'RF', 'calf',  'rf_calf_joint',  -1, -90.0),
    (6,  'LB', 'hip',   'lr_hip_joint',   +1,   0.0),
    (7,  'LB', 'thigh', 'lr_thigh_joint', +1,  45.0),
    (8,  'LB', 'calf',  'lr_calf_joint',  +1, -90.0),
    (9,  'RB', 'hip',   'rr_hip_joint',   -1,   0.0),
    (10, 'RB', 'thigh', 'rr_thigh_joint', -1,  45.0),
    (11, 'RB', 'calf',  'rr_calf_joint',  -1, -90.0),
]
BY_CH = {j[0]: j for j in JOINTS}
LEGS = ['LF', 'LB', 'RF', 'RB']


def load_trim():
    try:
        with open(TRIM_FILE) as f:
            data = json.load(f)
        return {int(k): float(v) for k, v in data.get('trims', {}).items()}
    except FileNotFoundError:
        return {}
    except Exception as exc:                       # noqa: BLE001
        print('ВНИМАНИЕ: %s повреждён (%s) — беру нули' % (TRIM_FILE, exc))
        return {}


TRIM = load_trim()


def save_trim():
    tmp = TRIM_FILE + '.tmp'
    with open(tmp, 'w') as f:
        json.dump({'step_deg': STEP_DEG,
                   'updated': time.strftime('%Y-%m-%d %H:%M:%S'),
                   'trims': {str(ch): round(TRIM.get(ch, 0.0), 2)
                             for ch, *_ in JOINTS}}, f, ensure_ascii=False, indent=1)
    os.replace(tmp, TRIM_FILE)


class Bus:
    """PCA9685: реальные записи, либо макет без I2C."""

    def __init__(self):
        self.real = False
        self.fd = -1
        self.mock = {}
        try:
            fd = os.open(DEV, os.O_RDWR)
            fcntl.ioctl(fd, I2C_SLAVE, ADDR)
            self.fd = fd
            self.real = True
            self._chip_init()
        except Exception as exc:                       # noqa: BLE001
            print('ВНИМАНИЕ: %s недоступен (%s) — режим макета' % (DEV, exc))

    def _chip_init(self):
        """Выход из sleep + ~50 Гц (после сброса питания чип в дефолте)."""
        os.write(self.fd, bytes([0x00, 0x11]))
        os.write(self.fd, bytes([0xFE, 0x79]))
        os.write(self.fd, bytes([0x00, 0x21]))

    @staticmethod
    def ticks(us):
        return min(max(int(round(us / PERIOD_US * 4096.0)), 0), 4095)

    def set_us(self, ch, us):
        t = self.ticks(us)
        if self.real:
            os.write(self.fd, bytes([0x06 + 4 * ch, 0, 0, t & 0xFF, (t >> 8) & 0x1F]))
        self.mock[ch] = t

    def off(self, ch):
        if self.real:
            os.write(self.fd, bytes([0x06 + 4 * ch, 0, 0, 0, 0x10]))
        self.mock[ch] = None

    def read(self, ch):
        """Текущее OFF-значение канала: None = full-off, иначе тики."""
        if self.real:
            os.write(self.fd, bytes([0x08 + 4 * ch]))
            lo = os.read(self.fd, 1)[0]
            os.write(self.fd, bytes([0x09 + 4 * ch]))
            hi = os.read(self.fd, 1)[0]
            if hi & 0x10:
                return None
            return ((hi & 0x0F) << 8) | lo
        return self.mock.get(ch)


BUS = Bus()


def mount_us(ch):
    """Монтажный импульс канала: центр + подстройка (в углах сустава)."""
    _, _, _, _, direction, _ = BY_CH[ch]
    return CENTER_US + direction * TRIM.get(ch, 0.0) * US_PER_DEG


def joint_state(ch):
    _, leg, joint, yaml_name, direction, ref = BY_CH[ch]
    t = BUS.read(ch)
    held = None if (t is None or t == 0) else round(t * PERIOD_US / 4096.0, 1)
    trim = TRIM.get(ch, 0.0)
    return {
        'ch': ch, 'leg': leg, 'joint': joint, 'yaml': yaml_name,
        'dir': direction, 'ref_deg': ref,
        'held_us': held,
        'trim_deg': round(trim, 2),
        'mount_us': round(mount_us(ch), 1),
        'offset_deg': round(ref - trim, 2),
    }


def all_state():
    return {'real': BUS.real, 'step_deg': STEP_DEG,
            'joints': [joint_state(ch) for ch, *_ in JOINTS]}


def channel_list(q, path):
    """Разбор адресации: ?ch=N или ?leg=LF или ?all=1."""
    if (q.get('all') or [''])[0] == '1':
        return [ch for ch, *_ in JOINTS]
    if q.get('ch'):
        try:
            ch = int(q['ch'][0])
        except ValueError:
            return None
        return [ch] if ch in BY_CH else None
    if q.get('leg'):
        leg = q['leg'][0].upper()
        return [ch for ch, *_ in JOINTS if BY_CH[ch][1] == leg] or None
    return None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):              # тихий лог
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _html(self):
        body = PAGE.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):                                  # noqa: N802
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path in ('/', '/index.html'):
            return self._html()
        if u.path == '/api/state':
            return self._json(all_state())
        if u.path == '/api/export':
            return self._json({'step_deg': STEP_DEG,
                               'joints': [joint_state(ch) for ch, *_ in JOINTS]})
        if u.path == '/api/release_all':
            for ch, *_ in JOINTS:
                BUS.off(ch)
            return self._json(all_state())
        if u.path == '/api/nudge':
            chs = channel_list(q, u.path)
            if not chs or len(chs) != 1:
                return self._json({'error': 'nudge needs one ch'}, 400)
            try:
                d = +1 if int((q.get('d') or ['1'])[0]) >= 0 else -1
            except ValueError:
                return self._json({'error': 'bad d'}, 400)
            ch = chs[0]
            trim = TRIM.get(ch, 0.0) + d * STEP_DEG
            TRIM[ch] = max(-TRIM_LIMIT_DEG, min(TRIM_LIMIT_DEG, trim))
            BUS.set_us(ch, mount_us(ch))
            save_trim()
            return self._json(all_state())
        if u.path == '/api/reset':
            chs = channel_list(q, u.path)
            if not chs:
                return self._json({'error': 'reset needs ch'}, 400)
            for ch in chs:
                TRIM[ch] = 0.0
                if BUS.read(ch) is not None:
                    BUS.set_us(ch, CENTER_US)
            save_trim()
            return self._json(all_state())
        if u.path in ('/api/set', '/api/release'):
            chs = channel_list(q, u.path)
            if not chs:
                return self._json({'error': 'set/release needs ch, leg or all'}, 400)
            for ch in chs:
                if u.path == '/api/set':
                    BUS.set_us(ch, mount_us(ch))
                else:
                    BUS.off(ch)
            return self._json(all_state())
        return self._json({'error': 'not found'}, 404)


PAGE = """<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Монтажные положения</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; font-family: system-ui, -apple-system, sans-serif; }
  body { margin: 0; background: #111; color: #eee; padding: 14px; max-width: 620px; }
  h1 { font-size: 18px; margin: 0 0 10px; }
  .tabs { display: flex; gap: 8px; margin-bottom: 12px; }
  .tabs button { flex: 1; padding: 14px 0; font-size: 18px; font-weight: 700;
                 background: #222; color: #bbb; border: 1px solid #333; border-radius: 10px; }
  .tabs button.active { background: #2d4; color: #000; border-color: #2d4; }
  .card { background: #1a1a1a; border: 1px solid #333; border-radius: 12px;
          padding: 12px; margin-bottom: 10px; }
  .card h2 { margin: 0 0 2px; font-size: 16px; }
  .dim { color: #888; font-size: 12px; font-weight: 400; }
  .st { font-size: 13px; color: #bbb; margin: 6px 0 10px; line-height: 1.45; }
  .st b { color: #2d4; font-weight: 600; } .st.off b { color: #888; }
  .btns { display: flex; gap: 8px; }
  .btns button { flex: 1; padding: 13px 0; font-size: 15px; font-weight: 700;
                 border: none; border-radius: 10px; }
  .b-step { background: #2a6; color: #000; }
  .b-hold { background: #2d4; color: #000; }
  .b-rel  { background: #444; color: #eee; }
  .lnk { background: none; border: none; color: #777; font-size: 12px;
         text-decoration: underline; padding: 4px 0; margin-top: 4px; }
  .legrow { display: flex; gap: 8px; margin-bottom: 10px; }
  .legrow button { flex: 1; padding: 12px 0; font-size: 14px; background: #334;
                   color: #cde; border: 1px solid #445; border-radius: 10px; }
  footer { margin-top: 6px; display: flex; gap: 10px; align-items: center; }
  footer button { padding: 12px 16px; font-size: 15px; background: #522; color: #fdd;
                  border: 1px solid #733; border-radius: 10px; }
  #conn { font-size: 12px; color: #888; margin-left: auto; }
  .hint { font-size: 12px; color: #777; margin-top: 12px; line-height: 1.5; }
  .hint code { color: #9a8; }
</style>
</head>
<body>
<h1>Монтажные положения + подстройка</h1>
<div class="tabs" id="tabs"></div>
<div id="box"></div>
<div class="legrow" id="legrow"></div>
<footer>
  <button id="allrel">Отпустить всё</button>
  <a class="lnk" href="/api/export" target="_blank">экспорт offset_deg</a>
  <span id="conn">связь: —</span>
</footer>
<div class="hint">
  «держать» = импульс <b>1370 мкс + подстройка</b> канала (монтажная поза: плечо
  вертикально, бедро +45°, колено −90°). Кнопки <b>±1°</b> — подстройка одним шагом
  в любую сторону (шаг = 1° сустава). Значения помнятся в
  <code>servo_pose_trim.json</code> и переносятся в <code>offset_deg</code>
  servos.yaml скриптом <code>servo_pose_apply.py</code> — это то, что увидят ноды.<br>
  У правых ног сервы стоят зеркально (<code>direction: −1</code>): шаг применяется
  в углах сустава, поэтому импульс у них меняется в обратную сторону — так
  проверяется инверсирование.<br>
  Во время калибровки стек ROS должен быть остановлен. Работай при свободных
  лапах, рука на выключателе питания.
</div>
<script>
const LEGS = ['LF', 'LB', 'RF', 'RB'];
let cur = 'LF';
let STATE = null;

const tabs = document.getElementById('tabs');
for (const l of LEGS) {
  const b = document.createElement('button');
  b.textContent = l;
  b.onclick = () => { cur = l; renderTabs(); render(); };
  tabs.appendChild(b);
}
function renderTabs() {
  [...tabs.children].forEach((b, i) => b.classList.toggle('active', LEGS[i] === cur));
}

async function api(p) {
  try { const r = await fetch(p); return await r.json(); } catch (e) { return null; }
}

function jointLine(j) {
  const head = j.joint + ' <span class="dim">· канал ' + j.ch +
    ' · dir ' + (j.dir > 0 ? '+1' : '−1') +
    (j.dir < 0 ? ' (зеркально)' : '') + '</span>';
  return head;
}
function jointStatus(j) {
  const trim = (j.trim_deg > 0 ? '+' : '') + j.trim_deg.toFixed(1);
  const off = j.offset_deg.toFixed(1);
  if (j.held_us == null)
    return '<span class="off">отпущено</span> · цель <b>' + j.mount_us.toFixed(1) +
           '</b> мкс · подстройка <b>' + trim + '°</b> · offset <b>' + off + '°</b>';
  return 'сейчас <b>' + j.held_us.toFixed(1) + '</b> мкс (цель ' + j.mount_us.toFixed(1) +
         ') · подстройка <b>' + trim + '°</b> · offset <b>' + off + '°</b>';
}

function row(j) {
  const d = document.createElement('div');
  d.className = 'card';
  d.innerHTML = '<h2>' + jointLine(j) + '</h2>' +
    '<div class="st" id="st-' + j.ch + '">—</div>' +
    '<div class="btns">' +
      '<button class="b-step">−1°</button>' +
      '<button class="b-hold">держать</button>' +
      '<button class="b-rel">снять</button>' +
      '<button class="b-step">+1°</button>' +
    '</div>' +
    '<button class="lnk">сброс подстройки</button>';
  const [b1, b2, b3, b4] = d.querySelectorAll('.btns button');
  b1.onclick = () => act('/api/nudge?ch=' + j.ch + '&d=-1');
  b2.onclick = () => act('/api/set?ch=' + j.ch);
  b3.onclick = () => act('/api/release?ch=' + j.ch);
  b4.onclick = () => act('/api/nudge?ch=' + j.ch + '&d=1');
  d.querySelector('.lnk').onclick = () => act('/api/reset?ch=' + j.ch);
  return d;
}

function render() {
  if (!STATE) return;
  const box = document.getElementById('box');
  box.innerHTML = '';
  for (const j of STATE.joints) {
    if (j.leg !== cur) continue;
    box.appendChild(row(j));
    put(j.ch, j);
  }
  // кнопки «лапа целиком»
  const lr = document.getElementById('legrow');
  lr.innerHTML = '';
  const h = document.createElement('button');
  h.textContent = 'держать лапу ' + cur + ' целиком';
  h.onclick = () => act('/api/set?leg=' + cur);
  const r = document.createElement('button');
  r.textContent = 'снять лапу ' + cur;
  r.onclick = () => act('/api/release?leg=' + cur);
  lr.appendChild(h); lr.appendChild(r);
}

function put(ch, j) {
  const el = document.getElementById('st-' + ch);
  if (!el) return;
  el.innerHTML = jointStatus(j);
  el.className = 'st' + (j.held_us == null ? ' off' : '');
}

async function refresh() {
  const s = await api('/api/state');
  const conn = document.getElementById('conn') || document.getElementById('connsp');
  STATE = s;
  if (!s) { if (conn) conn.textContent = 'связь: НЕТ'; return; }
  if (conn) conn.textContent = 'связь: ок' + (s.real ? ' (I2C)' : ' (макет)');
  for (const j of s.joints) put(j.ch, j);
}

async function act(p) { await api(p); await refresh(); }

document.getElementById('allrel').onclick = () => act('/api/release_all');

renderTabs();
refresh();
setInterval(refresh, 1500);
</script>
</body>
</html>
"""

if __name__ == '__main__':
    print('servo_pose_server: http://0.0.0.0:%d/  (%s)'
          % (PORT, 'I2C ' + DEV if BUS.real else 'МАКЕТ — I2C нет'))
    print('подстройка: %s' % TRIM_FILE)
    ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()
