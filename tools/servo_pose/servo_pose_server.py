#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Страница выставления монтажных положений серв (с удержанием).

Запуск:   python3 servo_pose_server.py [порт=8080]
Страница: http://<хост>:8080/
API:      /api/state, /api/set?unit=…, /api/release?unit=…, /api/release_all

На Pi пишет напрямую в PCA9685 (/dev/i2c-0 @0x40) — как драйвер проекта:
канал N = регистры 0x06+4N, [ON=0, OFF=тики], «отпустить» = бит full-off.
Если I2C недоступен (ПК/WSL) — режим макета: страницу можно смотреть и щёлкать.

Монтажное положение = центральный импульс 1370 мкс (плечо вертикально,
бедро +45°, колено −90°). «Выставить» держит импульс, «Отпустить» снимает.
"""
import fcntl
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080

CENTER_US = 1370.0
I2C_SLAVE = 0x0703
ADDR = 0x40
DEV = '/dev/i2c-0'
HZ = 25e6 / (4096.0 * 122.0)          # прескейл 121 -> 50.0288 Гц
PERIOD_US = 1e6 / HZ

# Юниты (метки владельца LF-LB-RF-RB). Каналы — по конфигу servos.yaml.
UNITS = {
    'lf_hip': [0], 'lf_leg': [1, 2],
    'lb_hip': [6], 'lb_leg': [7, 8],
    'rf_hip': [3], 'rf_leg': [4, 5],
    'rb_hip': [9], 'rb_leg': [10, 11],
}


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


def unit_state(unit):
    channels = []
    for ch in UNITS[unit]:
        t = BUS.read(ch)
        us = None if (t is None or t == 0) else t * PERIOD_US / 4096.0
        channels.append({'ch': ch, 'us': (round(us, 1) if us is not None else None)})
    held = [c for c in channels if c['us'] is not None]
    if not held:
        agg = 'off'
    elif len(held) == len(channels) and all(abs(c['us'] - CENTER_US) <= 15 for c in held):
        agg = 'center'
    else:
        agg = 'other'
    return {'agg': agg, 'channels': channels}


def all_state():
    return {'real': BUS.real, 'units': {u: unit_state(u) for u in UNITS}}


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
        if u.path == '/api/release_all':
            for unit in UNITS:
                for ch in UNITS[unit]:
                    BUS.off(ch)
            return self._json(all_state())
        if u.path in ('/api/set', '/api/release'):
            unit = (q.get('unit') or [''])[0]
            if unit not in UNITS:
                return self._json({'error': 'unknown unit: %r' % unit}, 400)
            for ch in UNITS[unit]:
                if u.path == '/api/set':
                    BUS.set_us(ch, CENTER_US)
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
  body { margin: 0; background: #111; color: #eee; padding: 14px; max-width: 560px; }
  h1 { font-size: 18px; margin: 0 0 10px; }
  .tabs { display: flex; gap: 8px; margin-bottom: 12px; }
  .tabs button { flex: 1; padding: 14px 0; font-size: 18px; font-weight: 700;
                 background: #222; color: #bbb; border: 1px solid #333; border-radius: 10px; }
  .tabs button.active { background: #2d4; color: #000; border-color: #2d4; }
  .card { background: #1a1a1a; border: 1px solid #333; border-radius: 12px;
          padding: 14px; margin-bottom: 12px; }
  .card h2 { margin: 0 0 4px; font-size: 16px; }
  .ch { color: #888; font-size: 12px; margin-bottom: 10px; }
  .btns { display: flex; gap: 10px; }
  .btns button { flex: 1; padding: 16px 0; font-size: 17px; font-weight: 700;
                 border: none; border-radius: 10px; }
  .set { background: #2d4; color: #000; }
  .rel { background: #444; color: #eee; }
  .status { margin-top: 8px; font-size: 14px; }
  .st-center { color: #2d4; } .st-off { color: #888; }
  .st-other { color: #fc4; } .st-err { color: #f55; }
  footer { margin-top: 14px; display: flex; gap: 10px; align-items: center; }
  footer button { padding: 12px 16px; font-size: 15px; background: #522; color: #fdd;
                  border: 1px solid #733; border-radius: 10px; }
  #conn { font-size: 12px; color: #888; margin-left: auto; }
  .hint { font-size: 12px; color: #777; margin-top: 10px; line-height: 1.45; }
</style>
</head>
<body>
<h1>Монтажные положения (удержание)</h1>
<div class="tabs" id="tabs"></div>

<div class="card">
  <h2>Плечо (hip)</h2>
  <div class="ch" id="hipch"></div>
  <div class="btns">
    <button class="set" id="hipset">Выставить</button>
    <button class="rel" id="hiprel">Отпустить</button>
  </div>
  <div class="status" id="hipst">—</div>
</div>

<div class="card">
  <h2>Лапа: бедро + колено</h2>
  <div class="ch" id="legch"></div>
  <div class="btns">
    <button class="set" id="legset">Выставить</button>
    <button class="rel" id="legrel">Отпустить</button>
  </div>
  <div class="status" id="legst">—</div>
</div>

<footer>
  <button id="allrel">Отпустить всё</button>
  <span id="conn">связь: —</span>
</footer>
<div class="hint">
  «Выставить» = импульс 1370 мкс (монтажное положение: плечо вертикально, бедро +45°,
  колено −90°), держится. «Отпустить» = выход выключен, сустав свободен.
  Работай при свободных лапах, рука на выключателе питания.
</div>

<script>
const LEGS = ['lf', 'lb', 'rf', 'rb'];
const CH = {
  lf: {hip: [0], leg: [1, 2]},
  lb: {hip: [6], leg: [7, 8]},
  rf: {hip: [3], leg: [4, 5]},
  rb: {hip: [9], leg: [10, 11]}
};
let cur = 'lf';

const tabs = document.getElementById('tabs');
for (const l of LEGS) {
  const b = document.createElement('button');
  b.textContent = l.toUpperCase();
  b.onclick = () => { cur = l; renderTabs(); refresh(); };
  tabs.appendChild(b);
}
function renderTabs() {
  [...tabs.children].forEach((b, i) => b.classList.toggle('active', LEGS[i] === cur));
  document.getElementById('hipch').textContent = 'каналы: ' + CH[cur].hip.join(', ');
  document.getElementById('legch').textContent = 'каналы: ' + CH[cur].leg.join(', ');
}

async function api(p) {
  try {
    const r = await fetch(p);
    if (!r.ok) return null;
    return await r.json();
  } catch (e) { return null; }
}

function fmt(st) {
  if (!st) return ['нет связи', 'st-err'];
  if (st.agg === 'center') return ['удерживается: центр (1370 мкс)', 'st-center'];
  if (st.agg === 'off') return ['отпущено (свободно)', 'st-off'];
  const us = st.channels.map(c => c.us === null ? '—' : Math.round(c.us) + ' мкс').join(', ');
  return ['удерживается: ' + us, 'st-other'];
}

function put(id, st) {
  const f = fmt(st);
  const el = document.getElementById(id);
  el.textContent = f[0];
  el.className = 'status ' + f[1];
}

async function refresh() {
  const s = await api('/api/state');
  const conn = document.getElementById('conn');
  if (!s) { conn.textContent = 'связь: НЕТ'; return; }
  conn.textContent = 'связь: ок' + (s.real ? ' (I2C)' : ' (макет)');
  put('hipst', s.units[cur + '_hip']);
  put('legst', s.units[cur + '_leg']);
}

async function act(unit, op) { await api('/api/' + op + '?unit=' + unit); refresh(); }
document.getElementById('hipset').onclick = () => act(cur + '_hip', 'set');
document.getElementById('hiprel').onclick = () => act(cur + '_hip', 'release');
document.getElementById('legset').onclick = () => act(cur + '_leg', 'set');
document.getElementById('legrel').onclick = () => act(cur + '_leg', 'release');
document.getElementById('allrel').onclick = async () => { await api('/api/release_all'); refresh(); };

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
    ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()
