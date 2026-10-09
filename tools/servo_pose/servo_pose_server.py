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
          /api/apply[?dry=1]         запись offset_deg в servos.yaml (кнопка страницы)
          /api/dir?ch=N&d=out|in|fwd|back|up|down
                                     проверка направления: шаг 12° с контролем тока
          /api/mock/current?val=A    (только макет) подмена тока для автотеста

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
import re
import shutil
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
TRIM_LIMIT_DEG = 30.0                      # предел подстройки, °
DIR_STEP_DEG = 12.0                        # шаг проверки направления, ° сустава
DIR_RAMP_STEPS = 10                        # ступеней движения
DIR_RAMP_DT = 0.07                         # с между ступенями (~0.7 с на шаг)
DIR_TRIP_A = 1.5                           # аварийный ток при проверке, А
DIR_HOLD_WATCH = 4                         # отсчётов после прихода (перегруз)
CLAMP_MARGIN_DEG = 5.0                     # запас до min/max на шаге проверки
INA_ADDR = 0x41                            # INA219 на шине
INA_SHUNT_OHM = 0.1                        # у модуля R100: ±320 мВ → ±3.2 А
MOCK_CURRENT = [0.15]                      # «ток» в макете, А
I2C_SLAVE = 0x0703
ADDR = 0x40
DEV = '/dev/i2c-0'
HZ = 25e6 / (4096.0 * 122.0)               # прескейл 121 -> 50.0288 Гц
PERIOD_US = 1e6 / HZ

TRIM_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'servo_pose_trim.json')
IMG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pose_img')
# Куда переносить подстройки (кнопка «Перенести в servos.yaml»).
# Приоритет: argv[2] → env SERVO_POSE_SERVOS → путь в клоне репы на Pi.
SERVOS_YAML = (sys.argv[2] if len(sys.argv) > 2
               else os.environ.get('SERVO_POSE_SERVOS')
               or '/home/sg/robot-dog/ros2_ws/src/dog_bringup/config/servos.yaml')

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
FALLBACK_BOUNDS = {'hip': (-40.0, 40.0), 'thigh': (-45.0, 135.0), 'calf': (-165.0, -15.0)}
_CFG_CACHE = {'stamp': None, 'cfg': {}}


def servo_cfg():
    """direction и min/max по каналам из servos.yaml (кэш по mtime); фолбэк — встроенный.

    Читается при каждом запросе: правка direction в файле подхватывается на лету
    (кнопка «инвертируй» = правка servos.yaml без перезапуска страницы).
    """
    base = {}
    for ch, _leg, joint, _name, direction, _ref in JOINTS:
        mn, mx = FALLBACK_BOUNDS[joint]
        base[ch] = {'dir': direction, 'min': mn, 'max': mx}
    try:
        st = os.stat(SERVOS_YAML)
        if _CFG_CACHE['stamp'] == (st.st_mtime_ns, st.st_size):
            return _CFG_CACHE['cfg']
        out = {ch: dict(v) for ch, v in base.items()}
        byname = {j[3]: j[0] for j in JOINTS}
        pat = re.compile(r'^\s*([a-z_]+_joint):\s+\{.*?direction:\s*([+-]?\d+).*?'
                         r'min_deg:\s*(-?[\d.]+),\s*max_deg:\s*(-?[\d.]+)', re.M)
        text = open(SERVOS_YAML).read()
        for m in pat.finditer(text):
            ch = byname.get(m.group(1))
            if ch is not None:
                out[ch]['dir'] = int(m.group(2))
                out[ch]['min'] = float(m.group(3))
                out[ch]['max'] = float(m.group(4))
        _CFG_CACHE['stamp'] = (st.st_mtime_ns, st.st_size)
        _CFG_CACHE['cfg'] = out
        return out
    except OSError:
        return base


def sem_sign(leg, joint, sem):
    """Знак сдвига в углах сустава: наружу — от корпуса (у правых ног к −)."""
    if joint == 'hip':
        if sem not in ('in', 'out'):
            return None
        outward = 1 if leg in ('LF', 'LB') else -1
        return outward if sem == 'out' else -outward
    if joint == 'thigh' and sem in ('fwd', 'back'):
        return -1 if sem == 'fwd' else 1
    if joint == 'calf' and sem in ('up', 'down'):
        return -1 if sem == 'up' else 1
    return None


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


def apply_offsets(dry=False):
    """offset_deg = опорный угол − trim; бэкап .bak, возвращает отчёт."""
    try:
        with open(SERVOS_YAML) as f:
            text = f.read()
    except OSError as exc:
        return {'ok': False, 'error': 'нет %s (%s)' % (SERVOS_YAML, exc)}
    want = {j[3]: j[5] - TRIM.get(j[0], 0.0) for j in JOINTS}
    pat = re.compile(r'^(\s*([a-z_]+_joint):\s+\{.*?offset_deg:)\s*(-?\d+(?:\.\d+)?)(.*)$')
    out, changed = [], []
    for line in text.splitlines():
        m = pat.match(line)
        if m and m.group(2) in want:
            new_val = '%6.1f' % want[m.group(2)]
            if m.group(3) != new_val.strip():
                changed.append([m.group(2), m.group(3), new_val.strip()])
            line = m.group(1) + new_val + m.group(4)
        out.append(line)
    if not changed or dry:
        return {'ok': True, 'dry': dry, 'path': SERVOS_YAML, 'changed': changed,
                'note': ('уже совпадает' if not changed else 'предпросмотр')}
    shutil.copy2(SERVOS_YAML, SERVOS_YAML + '.bak')
    with open(SERVOS_YAML, 'w') as f:
        f.write('\n'.join(out) + ('\n' if text.endswith('\n') else ''))
    return {'ok': True, 'dry': False, 'path': SERVOS_YAML, 'changed': changed}


def dir_test(ch, sem):
    """Шаг DIR_STEP_DEG в сторону sem с контролем тока; при перегрузе — отпускает."""
    _, leg, joint, _name, _dir0, ref = BY_CH[ch]
    sign = sem_sign(leg, joint, sem)
    if sign is None:
        return {'ok': False, 'error': 'для сустава «%s» нет направления «%s»' % (joint, sem)}
    cfg = servo_cfg()[ch]
    d = cfg['dir']
    lo = cfg['min'] - ref + CLAMP_MARGIN_DEG
    hi = cfg['max'] - ref - CLAMP_MARGIN_DEG
    base = mount_us(ch)
    t = BUS.read(ch)
    start = base if (t is None or t == 0) else t * PERIOD_US / 4096.0
    dev0 = (start - base) / (d * US_PER_DEG)
    dev1 = max(lo, min(hi, dev0 + sign * DIR_STEP_DEG))
    target = base + d * dev1 * US_PER_DEG
    samples, tripped = [], False
    for i in range(1, DIR_RAMP_STEPS + 1):
        BUS.set_us(ch, start + (target - start) * i / DIR_RAMP_STEPS)
        time.sleep(DIR_RAMP_DT)
        a = BUS.current_a()
        if a is None:
            continue
        samples.append(a)
        if len(samples) >= 2 and abs(samples[-1]) > DIR_TRIP_A and abs(samples[-2]) > DIR_TRIP_A:
            tripped = True
            break
    if not tripped:
        for _ in range(DIR_HOLD_WATCH):
            time.sleep(DIR_RAMP_DT)
            a = BUS.current_a()
            if a is None:
                continue
            samples.append(a)
            if len(samples) >= 2 and abs(samples[-1]) > DIR_TRIP_A and abs(samples[-2]) > DIR_TRIP_A:
                tripped = True
                break
    labels = {'out': 'наружу', 'in': 'внутрь', 'fwd': 'вперёд', 'back': 'назад',
              'up': 'вверх', 'down': 'вниз'}
    peak = round(max(abs(x) for x in samples), 2) if samples else None
    if tripped:
        BUS.off(ch)
        return {'ok': True, 'ch': ch, 'sem': sem, 'label': labels[sem], 'tripped': True,
                'peak_a': peak, 'moved_deg': 0.0,
                'note': 'СТОП по току — сустав отпущен'}
    limited = abs((dev0 + sign * DIR_STEP_DEG) - dev1) > 1e-6
    return {'ok': True, 'ch': ch, 'sem': sem, 'label': labels[sem], 'tripped': False,
            'peak_a': peak, 'moved_deg': round(abs(dev1 - dev0), 1),
            'dev_after_deg': round(dev1, 1), 'clamped': limited,
            'note': 'предел теста' if limited else ''}


def invert_direction(ch):
    """Меняет знак direction сустава в servos.yaml (бэкап .bak)."""
    name = BY_CH[ch][3]
    try:
        with open(SERVOS_YAML) as f:
            text = f.read()
    except OSError as exc:
        return {'ok': False, 'error': 'нет %s (%s)' % (SERVOS_YAML, exc)}
    lines = text.splitlines(keepends=True)
    for i, line in enumerate(lines):
        if not re.match(r'\s*%s:\s+\{' % re.escape(name), line):
            continue
        m = re.search(r'(direction:)\s*([+-]?\d+)(\s*,)', line)
        if not m:
            return {'ok': False, 'error': 'в строке %s нет direction' % name}
        old = int(m.group(2))
        new = -old
        lines[i] = (line[:m.start()] + m.group(1) + ' ' + ('%2d' % new) + m.group(3)
                    + line[m.end():])
        shutil.copy2(SERVOS_YAML, SERVOS_YAML + '.bak')
        with open(SERVOS_YAML, 'w') as f:
            f.write(''.join(lines))
        return {'ok': True, 'ch': ch, 'yaml': name, 'old': old, 'new': new,
                'path': SERVOS_YAML}
    return {'ok': False, 'error': 'не нашёл %s в servos.yaml' % name}


class Bus:
    """PCA9685: реальные записи, либо макет без I2C."""

    def __init__(self):
        self.real = False
        self.fd = -1
        self.fd_ina = -1
        self.mock = {}
        try:
            fd = os.open(DEV, os.O_RDWR)
            fcntl.ioctl(fd, I2C_SLAVE, ADDR)
            self.fd = fd
            self.real = True
            self._chip_init()
        except Exception as exc:                       # noqa: BLE001
            print('ВНИМАНИЕ: %s недоступен (%s) — режим макета' % (DEV, exc))
        if self.real:
            try:
                fd2 = os.open(DEV, os.O_RDWR)
                fcntl.ioctl(fd2, I2C_SLAVE, INA_ADDR)
                os.write(fd2, bytes([0x00, 0x39, 0x9F]))   # 128-усреднение, непрерывно
                self.fd_ina = fd2
            except Exception as exc:                   # noqa: BLE001
                print('ВНИМАНИЕ: INA219 (0x%02x) недоступен (%s) — без контроля тока'
                      % (INA_ADDR, exc))

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

    def current_a(self):
        """Ток шины серв с INA219 (0x41), А; в макете — подставное значение."""
        if not self.real or self.fd_ina < 0:
            return MOCK_CURRENT[0]
        try:
            os.write(self.fd_ina, bytes([0x01]))       # shunt, старший байт вперёд
            raw = os.read(self.fd_ina, 2)
            if len(raw) != 2:
                return None
            v = (raw[0] << 8) | raw[1]
            if v > 32767:
                v -= 65536
            return v * 10e-6 / INA_SHUNT_OHM
        except OSError:
            return None


BUS = Bus()


def mount_us(ch):
    """Монтажный импульс канала: центр + подстройка (в углах сустава)."""
    return CENTER_US + servo_cfg()[ch]['dir'] * TRIM.get(ch, 0.0) * US_PER_DEG


def joint_state(ch):
    _, leg, joint, yaml_name, _direction, ref = BY_CH[ch]
    t = BUS.read(ch)
    held = None if (t is None or t == 0) else round(t * PERIOD_US / 4096.0, 1)
    trim = TRIM.get(ch, 0.0)
    return {
        'ch': ch, 'leg': leg, 'joint': joint, 'yaml': yaml_name,
        'dir': servo_cfg()[ch]['dir'], 'ref_deg': ref,
        'held_us': held,
        'trim_deg': round(trim, 2),
        'mount_us': round(mount_us(ch), 1),
        'offset_deg': round(ref - trim, 2),
    }


def all_state():
    cur = BUS.current_a()
    return {'real': BUS.real, 'step_deg': STEP_DEG, 'dir_step_deg': DIR_STEP_DEG,
            'current_a': None if cur is None else round(cur, 2),
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
    protocol_version = 'HTTP/1.1'                      # keep-alive: короче задержки на Wi-Fi

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

    def _img(self, name):
        """Статические схемы из pose_img/ (svg), только простые имена."""
        if not re.fullmatch(r'[a-z0-9_]+\.svg', name or ''):
            return self._json({'error': 'bad image name'}, 404)
        try:
            with open(os.path.join(IMG_DIR, name), 'rb') as f:
                data = f.read()
        except OSError:
            return self._json({'error': 'no image: %s' % name}, 404)
        self.send_response(200)
        self.send_header('Content-Type', 'image/svg+xml')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

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
        if u.path == '/api/dir':
            chs = channel_list(q, u.path)
            if not chs or len(chs) != 1:
                return self._json({'error': 'dir needs one ch'}, 400)
            return self._json(dir_test(chs[0], (q.get('d') or [''])[0].lower()))
        if u.path == '/api/invert':
            chs = channel_list(q, u.path)
            if not chs or len(chs) != 1:
                return self._json({'error': 'invert needs one ch'}, 400)
            return self._json(invert_direction(chs[0]))
        if u.path == '/api/mock/current':
            if BUS.real:
                return self._json({'error': 'mock only'}, 404)
            try:
                MOCK_CURRENT[0] = float((q.get('val') or ['0.15'])[0])
            except ValueError:
                return self._json({'error': 'bad val'}, 400)
            return self._json({'ok': True, 'current_a': MOCK_CURRENT[0]})
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
        if u.path == '/api/apply':
            dry = (q.get('dry') or [''])[0] == '1'
            return self._json(apply_offsets(dry=dry))
        if u.path.startswith('/img/'):
            return self._img(u.path[len('/img/'):])
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
  body { margin: 0 auto; background: #111; color: #eee; padding: 14px; max-width: 1280px; }
  .layout { display: grid; grid-template-columns: minmax(0, 1fr); gap: 12px; align-items: start; }
  @media (min-width: 900px) {
    .layout { grid-template-columns: 380px minmax(0, 1fr); }
    .right { position: sticky; top: 12px; }
  }
  #box { display: grid; grid-template-columns: 1fr; gap: 10px; }
  h1 { font-size: 18px; margin: 0 0 10px; }
  .tabs { display: flex; gap: 8px; margin-bottom: 12px; }
  .tabs button { flex: 1; padding: 14px 0; font-size: 18px; font-weight: 700;
                 background: #222; color: #bbb; border: 1px solid #333; border-radius: 10px; }
  .tabs button.active { background: #2d4; color: #000; border-color: #2d4; }
  .card { background: #1a1a1a; border: 1px solid #333; border-radius: 12px;
          padding: 12px; }
  .card h2 { margin: 0 0 2px; font-size: 16px; }
  .dim { color: #888; font-size: 12px; font-weight: 400; }
  .st { font-size: 13px; color: #bbb; margin: 6px 0 10px; line-height: 1.45; }
  .st b { color: #2d4; font-weight: 600; } .st.off b { color: #888; }
  .btns { display: flex; gap: 8px; }
  .btns + .btns { margin-top: 8px; }
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
  .b-apply { padding: 12px 14px; font-size: 14px; background: #246; color: #cde;
             border: 1px solid #468; border-radius: 10px; }
  .b-dir { background: #753; color: #fed; }
  .btns button:disabled { opacity: .45; }
  .dirmsg { font-size: 12px; color: #9a8; margin-top: 6px; min-height: 15px; line-height: 1.35; }
  .dirmsg.bad { color: #f88; }
  .linkrow { display: flex; gap: 16px; align-items: center; }
  .lnk.inv { color: #eb6; font-weight: 600; }
  #applymsg { font-size: 12px; color: #9a8; margin-top: 6px; line-height: 1.4; }
  details.imgs { background: #1a1a1a; border: 1px solid #333; border-radius: 12px;
                 padding: 10px 12px; margin-bottom: 12px; font-size: 13px; }
  details.imgs summary { cursor: pointer; color: #ddd; font-weight: 600; margin-bottom: 4px; }
  details.imgs a { display: block; }
  details.imgs img { display: block; width: 100%; height: auto; background: #eef1f5;
                     border-radius: 8px; margin: 10px 0 4px; }
  details.imgs .cap { color: #888; font-size: 12px; line-height: 1.45; }
  .hint { font-size: 12px; color: #777; margin-top: 12px; line-height: 1.5; }
  .hint code { color: #9a8; }
</style>
</head>
<body>
<h1>Монтажные положения + подстройка</h1>
<div class="layout">
  <div class="left">
    <div class="tabs" id="tabs"></div>
    <div id="box"></div>
    <div class="legrow" id="legrow"></div>
    <footer>
      <button id="allrel">Отпустить всё</button>
      <button id="applybtn" class="b-apply">Перенести в servos.yaml</button>
      <a class="lnk" href="/api/export" target="_blank">экспорт offset_deg</a>
      <span id="conn">связь: —</span>
    </footer>
    <div id="applymsg"></div>
  </div>
  <div class="right">
    <details class="imgs" open>
      <summary>Монтажная поза — схема</summary>
      <a href="/img/assembly_pose.svg" target="_blank" rel="noopener"><img src="/img/assembly_pose.svg" alt="Монтажная поза и рабочие позы"></a>
      <div class="cap">Все 12 серв на 1370 мкс — плечо вертикально, бедро +45° назад, колено −90°.
      Ниже на схеме — рабочие позы с импульсами (у правой ноги зеркально).</div>
    </details>
  </div>
</div>
<div class="hint">
  «держать» = импульс <b>1370 мкс + подстройка</b> канала (монтажная поза: плечо
  вертикально, бедро +45°, колено −90°). Кнопки <b>±1°</b> — подстройка одним шагом
  в любую сторону (шаг = 1° сустава, предел ±30°). Значения помнятся в
  <code>servo_pose_trim.json</code> и переносятся в <code>offset_deg</code>
  servos.yaml скриптом <code>servo_pose_apply.py</code> — это то, что увидят ноды.<br>
  У правых ног сервы стоят зеркально (<code>direction: −1</code>): шаг применяется
  в углах сустава, поэтому импульс у них меняется в обратную сторону — так
  проверяется инверсирование.<br>
  <b>Проверка направления:</b> кнопки «внутрь/наружу» (плечо), «вперёд/назад»
  (бедро), «вверх/вниз» (колено) сдвигают сустав на 12°. Правильно так: наружу —
  от корпуса; вперёд — стопа к носу; вверх — голень складывается (стопа
  поднимается). Если наоборот — жми «инвертировать direction» у сустава
  (правит <code>direction</code> в servos.yaml, бэкап .bak — страница видит сразу),
  или скажи мне. При токе
  больше 1.5 А движение останавливается, сустав отпускается.<br>
  Во время калибровки стек ROS должен быть остановлен. Работай при свободных
  лапах, рука на выключателе питания.
</div>
<script>
const LEGS = ['LF', 'LB', 'RF', 'RB'];
let cur = 'LF';
let STATE = null;
const DIR_BTNS = {
  hip:   [['внутрь', 'in'], ['наружу', 'out']],
  thigh: [['вперёд', 'fwd'], ['назад', 'back']],
  calf:  [['вверх', 'up'], ['вниз', 'down']],
};

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
  d.innerHTML = '<h2 id="h-' + j.ch + '">' + jointLine(j) + '</h2>' +
    '<div class="st" id="st-' + j.ch + '">—</div>' +
    '<div class="btns">' +
      '<button class="b-hold">держать</button>' +
      '<button class="b-rel">снять</button>' +
    '</div>' +
    '<div class="btns">' +
      '<button class="b-step">−1°</button>' +
      '<button class="b-step">+1°</button>' +
    '</div>' +
    '<div class="btns">' +
      '<button class="b-dir"></button>' +
      '<button class="b-dir"></button>' +
    '</div>' +
    '<div class="dirmsg" id="dir-' + j.ch + '">шаг ±12° — смотри глазами (верно или инверсия)</div>' +
    '<div class="linkrow">' +
      '<button class="lnk">сброс подстройки</button>' +
      '<button class="lnk inv">инвертировать direction</button>' +
    '</div>';
  const [hold, rel] = d.querySelectorAll('.btns')[0].querySelectorAll('button');
  const [minus, plus] = d.querySelectorAll('.btns')[1].querySelectorAll('button');
  const dBtns = d.querySelectorAll('.btns')[2].querySelectorAll('button');
  const dirs = DIR_BTNS[j.joint] || [['?', 'in'], ['?', 'out']];
  dBtns[0].textContent = dirs[0][0];
  dBtns[0].dataset.d = dirs[0][1];
  dBtns[1].textContent = dirs[1][0];
  dBtns[1].dataset.d = dirs[1][1];
  hold.onclick = () => act('/api/set?ch=' + j.ch);
  rel.onclick = () => act('/api/release?ch=' + j.ch);
  minus.onclick = () => act('/api/nudge?ch=' + j.ch + '&d=-1');
  plus.onclick = () => act('/api/nudge?ch=' + j.ch + '&d=1');
  for (const b of dBtns) b.onclick = () => dirAct(j, b);
  d.querySelector('.lnk').onclick = () => act('/api/reset?ch=' + j.ch);
  d.querySelector('.lnk.inv').onclick = () => invAct(j);
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
  const txt = jointStatus(j);
  if (el.innerHTML !== txt) el.innerHTML = txt;
  el.className = 'st' + (j.held_us == null ? ' off' : '');
  const h = document.getElementById('h-' + ch);
  if (h) {
    const ht = jointLine(j);
    if (h.innerHTML !== ht) h.innerHTML = ht;
  }
}

async function refresh() {
  const s = await api('/api/state');
  const conn = document.getElementById('conn');
  STATE = s;
  if (!s) { if (conn) conn.textContent = 'связь: НЕТ'; return; }
  if (conn) conn.textContent = 'связь: ок' + (s.real ? ' (I2C)' : ' (макет)') +
    (s.current_a != null ? ' · ток ' + s.current_a.toFixed(2) + ' А' : '');
  if (!document.getElementById('box').children.length) render();
  for (const j of s.joints) put(j.ch, j);
}

async function act(p) { await api(p); await refresh(); }

async function dirAct(j, btn) {
  const box = document.getElementById('dir-' + j.ch);
  const btns = btn.parentElement.querySelectorAll('button');
  btns.forEach(b => b.disabled = true);
  box.className = 'dirmsg';
  box.textContent = '…';
  const r = await api('/api/dir?ch=' + j.ch + '&d=' + btn.dataset.d);
  btns.forEach(b => b.disabled = false);
  if (!r || !r.ok) {
    box.className = 'dirmsg bad';
    box.textContent = 'ошибка: ' + ((r && r.error) || 'запрос не прошёл');
    return;
  }
  const pk = (r.peak_a == null) ? 'нет данных тока' : 'пик ' + r.peak_a.toFixed(2) + ' А';
  if (r.tripped) {
    box.className = 'dirmsg bad';
    box.textContent = 'СТОП по току (' + pk + ') — сустав отпущен';
  } else if (r.clamped) {
    box.className = 'dirmsg bad';
    box.textContent = '«' + r.label + '»: ' + r.moved_deg.toFixed(1) + '°, предел теста (' + pk + ')';
  } else {
    box.className = 'dirmsg';
    box.textContent = '«' + r.label + '»: ' + r.moved_deg.toFixed(1) + '°, ' + pk;
  }
  await refresh();
}

async function invAct(j) {
  const box = document.getElementById('dir-' + j.ch);
  if (!confirm('Инвертировать direction сустава ' + j.leg + ' ' + j.joint +
               ' в servos.yaml? Сейчас dir=' + (j.dir > 0 ? '+1' : '−1'))) return;
  const r = await api('/api/invert?ch=' + j.ch);
  if (!r || !r.ok) {
    box.className = 'dirmsg bad';
    box.textContent = 'ошибка: ' + ((r && r.error) || 'не прошло');
    return;
  }
  box.className = 'dirmsg';
  box.textContent = 'direction инвертирован: ' + r.old + ' → ' + r.new + ' (' + r.yaml +
    '); нажми кнопку направления ещё раз — поедет зеркально';
  await refresh();
}

document.getElementById('allrel').onclick = () => act('/api/release_all');

document.getElementById('applybtn').onclick = async () => {
  const msg = document.getElementById('applymsg');
  msg.textContent = '…';
  const dry = await api('/api/apply?dry=1');
  if (!dry || !dry.ok) {
    msg.textContent = 'ошибка: ' + ((dry && dry.error) || 'запрос не прошёл');
    return;
  }
  if (!dry.changed.length) { msg.textContent = 'уже совпадает — менять нечего'; return; }
  const list = dry.changed.map(c => c[0] + ' ' + c[1] + '→' + c[2]);
  const preview = list.slice(0, 8).join('; ') + (list.length > 8 ? ' …' : '');
  if (!confirm('Записать offset_deg в servos.yaml? ' + preview)) {
    msg.textContent = 'отменено';
    return;
  }
  const r = await api('/api/apply');
  if (!r || !r.ok) {
    msg.textContent = 'ошибка: ' + ((r && r.error) || 'запись не прошла');
    return;
  }
  msg.textContent = 'записано (' + r.changed.length + '): ' + preview + '  →  ' + r.path + ' (бэкап .bak)';
};

renderTabs();
(async function loop() { await refresh(); setTimeout(loop, 1500); })();
</script>
</body>
</html>
"""

if __name__ == '__main__':
    print('servo_pose_server: http://0.0.0.0:%d/  (%s)'
          % (PORT, 'I2C ' + DEV if BUS.real else 'МАКЕТ — I2C нет'))
    print('подстройка: %s' % TRIM_FILE)
    print('servos.yaml: %s' % SERVOS_YAML)
    ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()
