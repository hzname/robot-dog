"""Measurement diagrams for docs/DEPLOYMENT.md (stage 1).  python3 make_diagrams.py"""
import math
from xml.sax.saxutils import escape

HEAD = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}" font-family="DejaVu Sans, Arial, sans-serif" font-size="13">
<defs>
 <marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#2a78d6"/></marker>
 <marker id="o" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#eb6834"/></marker>
 <marker id="k" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#12161b"/></marker>
</defs>
<rect width="100%" height="100%" fill="#ffffff"/>
"""
T, G, LEG, BLUE, ORANGE = '#12161b', '#8791a0', '#d9892b', '#2a78d6', '#eb6834'


def dim(x1, y1, x2, y2, label, lx=None, ly=None, anchor='middle'):
    s = (f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{BLUE}" stroke-width="1.6" '
         'marker-start="url(#a)" marker-end="url(#a)"/>')
    if lx is None:
        lx, ly = (x1 + x2) / 2, (y1 + y2) / 2 - 6
    return s + f'<text x="{lx:.1f}" y="{ly:.1f}" fill="{BLUE}" font-weight="bold" text-anchor="{anchor}">{label}</text>'


def axis(x, y, r=7):
    return (f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" fill="#fff" stroke="{T}" stroke-width="2"/>'
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2" fill="{T}"/>')


def text(x, y, t, col=T, size=13, anchor='start', bold=False):
    weight = ' font-weight="bold"' if bold else ''
    return (f'<text x="{x:.1f}" y="{y:.1f}" fill="{col}" font-size="{size}" text-anchor="{anchor}"'
            f'{weight}>{escape(t)}</text>')


def notes(lines, y0, size=12.5):
    return ''.join(text(20, y0 + i * 17, t, size=size) for i, t in enumerate(lines))


def save(name, w, h, body):
    open(name, 'w').write(HEAD.format(w=w, h=h) + body + '</svg>\n')


# ---------------------------------------------------------------- leg, side
def leg_side():
    w, h = 700, 500
    s = text(20, 28, 'Нога сбоку: смотрим на ЛЕВЫЙ бок робота, «вперёд» — вправо', size=15, bold=True)
    hx, hy, L = 330, 120, 150
    th, cf = math.radians(44), math.radians(-89)
    kx, ky = hx - L * math.sin(th), hy + L * math.cos(th)
    a2 = th + cf
    fx, fy = kx - L * math.sin(a2), ky + L * math.cos(a2)
    gy = fy + 10
    s += f'<rect x="230" y="{hy - 30}" width="260" height="46" rx="4" fill="#3b4452" opacity="0.9"/>'
    s += text(430, hy - 10, 'корпус', col='#fff')
    s += f'<line x1="120" y1="{gy}" x2="600" y2="{gy}" stroke="{G}" stroke-width="2"/>' + text(565, gy + 18, 'пол', col=G)
    s += f'<line x1="{hx}" y1="{hy}" x2="{kx:.1f}" y2="{ky:.1f}" stroke="{LEG}" stroke-width="12" stroke-linecap="round"/>'
    s += f'<line x1="{kx:.1f}" y1="{ky:.1f}" x2="{fx:.1f}" y2="{fy:.1f}" stroke="{LEG}" stroke-width="9" stroke-linecap="round"/>'
    s += f'<circle cx="{fx:.1f}" cy="{fy:.1f}" r="10" fill="#333"/>'
    s += axis(hx, hy, 9) + axis(kx, ky, 8)
    s += text(hx + 14, hy + 30, 'ось бедра (вал сервы thigh)')
    s += text(kx - 16, ky + 4, 'ось колена', anchor='end')
    s += text(fx + 16, fy + 4, 'точка касания')
    ox, oy = -24 * math.cos(th), -24 * math.sin(th)
    s += dim(hx + ox, hy + oy, kx + ox, ky + oy, 'thigh', lx=(hx + kx) / 2 - 38, ly=(hy + ky) / 2 - 14, anchor='end')
    ox2, oy2 = 20 * math.cos(a2), 20 * math.sin(a2)
    s += dim(kx + ox2, ky + oy2, fx + ox2, fy + 10 + oy2, 'calf', lx=(kx + fx) / 2 + 36, ly=(ky + fy) / 2 + 10, anchor='start')
    s += f'<line x1="{hx}" y1="{hy}" x2="535" y2="{hy}" stroke="{G}" stroke-dasharray="4 3"/>'
    s += dim(525, hy, 525, gy, 'stand_height', lx=533, ly=(hy + gy) / 2, anchor='start')
    s += f'<line x1="{hx}" y1="{hy}" x2="{hx}" y2="{hy + 100}" stroke="{G}" stroke-dasharray="5 4"/>'
    r = 62
    s += (f'<path d="M{hx},{hy + r} A{r},{r} 0 0,1 {hx - r * math.sin(th):.1f},{hy + r * math.cos(th):.1f}" '
          f'fill="none" stroke="{ORANGE}" stroke-width="2" marker-end="url(#o)"/>')
    s += text(hx + 8, hy + r + 22, 'угол thigh ≈ +44° в стойке', col=ORANGE, bold=True)
    ex, ey = kx - 60 * math.sin(th), ky + 60 * math.cos(th)
    s += f'<line x1="{kx:.1f}" y1="{ky:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" stroke="{G}" stroke-dasharray="5 4"/>'
    s += (f'<path d="M{kx - 42 * math.sin(th):.1f},{ky + 42 * math.cos(th):.1f} A42,42 0 0,0 '
          f'{kx - 42 * math.sin(a2):.1f},{ky + 42 * math.cos(a2):.1f}" fill="none" stroke="{ORANGE}" stroke-width="2" '
          'marker-end="url(#o)"/>')
    s += text(kx - 30, ky + 78, 'угол calf ≈ −89° в стойке', col=ORANGE, bold=True, anchor='end')
    s += f'<line x1="590" y1="62" x2="660" y2="62" stroke="{T}" stroke-width="2" marker-end="url(#k)"/>'
    s += text(625, 54, 'вперёд (x)', anchor='middle')
    s += notes(['thigh — от центра оси бедра до центра оси колена, по прямой.',
                'calf — от центра оси колена до точки касания стопы с полом.',
                'stand_height — от оси бедра до пола в стойке (задаётся в конфиге, не мерится).',
                'Нули: thigh 0 = бедро вертикально вниз, «+» = стопа уходит назад;',
                'calf 0 = нога прямая, «−» = колено согнуто назад (−90° = прямой угол).'], h - 88)
    save('measure_leg_side.svg', w, h, s)


# ---------------------------------------------------------------- leg, rear
def leg_rear():
    w, h = 700, 480
    s = text(20, 28, 'Нога сзади: смотрим на робота СЗАДИ, его левый бок — слева', size=15, bold=True)
    cx, by = 360, 120
    s += f'<rect x="{cx - 60}" y="{by - 25}" width="120" height="50" rx="4" fill="#3b4452" opacity="0.9"/>'
    s += text(cx, by + 5, 'корпус', col='#fff', anchor='middle')
    s += f'<line x1="{cx}" y1="55" x2="{cx}" y2="390" stroke="{G}" stroke-dasharray="8 4"/>'
    s += text(cx + 6, 345, 'середина корпуса', col=G)
    ax, off = cx - 90, 60
    lx = ax - off
    s += f'<line x1="{ax}" y1="{by}" x2="{lx}" y2="{by}" stroke="#555" stroke-width="10" stroke-linecap="round"/>'
    s += axis(ax, by, 9)
    s += text(ax + 14, by + 46, 'ось отведения hip', size=12)
    s += text(ax + 14, by + 61, '(вдоль корпуса, на нас)', size=12)
    s += f'<line x1="{lx}" y1="{by}" x2="{lx}" y2="340" stroke="{LEG}" stroke-width="12" stroke-linecap="round"/>'
    s += f'<circle cx="{lx}" cy="348" r="10" fill="#333"/>'
    for x in (lx, ax, cx):
        s += f'<line x1="{x}" y1="66" x2="{x}" y2="{by - 14}" stroke="{G}" stroke-dasharray="4 3"/>'
    s += dim(ax, 76, cx, 76, 'hip_y', ly=70)
    s += dim(lx, 96, ax, 96, 'hip_offset', lx=lx - 8, ly=92, anchor='end')
    s += text(lx - 14, 240, 'плоскость, в которой', size=12, anchor='end')
    s += text(lx - 14, 255, 'ходят бедро и голень', size=12, anchor='end')
    s += f'<line x1="60" y1="358" x2="640" y2="358" stroke="{G}" stroke-width="2"/>'
    s += (f'<path d="M{lx + 2},310 A 190 190 0 0 1 {lx - 45},302" fill="none" stroke="{ORANGE}" stroke-width="2" '
          'marker-end="url(#o)"/>')
    s += text(lx - 52, 292, 'угол hip «+»', col=ORANGE, bold=True, anchor='end')
    s += text(lx - 52, 307, 'стопа влево', col=ORANGE, size=12, anchor='end')
    s += f'<line x1="640" y1="110" x2="580" y2="110" stroke="{T}" stroke-width="2" marker-end="url(#k)"/>'
    s += text(610, 102, 'влево (y)', anchor='middle')
    s += notes(['hip_y — от середины корпуса до оси отведения = половина расстояния между осями',
                '        отведения левой и правой ноги.',
                'hip_offset — от оси отведения до средней плоскости бедра и голени.',
                'В модели ось отведения и ось бедра на одной высоте. Если они разнесены по высоте',
                'больше чем на 5 мм — запишите это число: модель придётся дополнить.'], h - 92)
    save('measure_leg_rear.svg', w, h, s)


# ---------------------------------------------------------------- body, top
def body_top():
    w, h = 700, 480
    s = text(20, 28, 'Корпус сверху: «вперёд» — вправо, левый бок — вверху', size=15, bold=True)
    cx, cy, hxp, hyp = 330, 210, 150, 70
    s += f'<rect x="{cx - 175}" y="{cy - 50}" width="350" height="100" rx="6" fill="#3b4452" opacity="0.15" stroke="#3b4452"/>'
    for fx, fy, name in ((1, 1, 'LF'), (1, -1, 'RF'), (-1, 1, 'LR'), (-1, -1, 'RR')):
        x, y = cx + fx * hxp, cy - fy * hyp
        s += f'<line x1="{x - 30}" y1="{y}" x2="{x + 30}" y2="{y}" stroke="#555" stroke-width="3" stroke-dasharray="6 3"/>'
        s += f'<circle cx="{x}" cy="{y}" r="6" fill="{T}"/>'
        s += text(x + (12 if fx > 0 else -12), y - fy * 16 + 4, name, bold=True, anchor='start' if fx > 0 else 'end')
    s += f'<circle cx="{cx}" cy="{cy}" r="5" fill="{ORANGE}"/>' + text(cx + 8, cy + 20, 'центр корпуса', col=ORANGE, bold=True)
    s += f'<line x1="{cx}" y1="{cy}" x2="{cx + 70}" y2="{cy}" stroke="{T}" stroke-width="2" marker-end="url(#k)"/>' + text(cx + 76, cy + 4, 'x')
    s += f'<line x1="{cx}" y1="{cy}" x2="{cx}" y2="{cy - 60}" stroke="{T}" stroke-width="2" marker-end="url(#k)"/>' + text(cx + 5, cy - 62, 'y')
    s += f'<line x1="{cx + hxp}" y1="{cy - hyp}" x2="{cx + hxp}" y2="{cy + hyp + 48}" stroke="{G}" stroke-dasharray="4 3"/>'
    s += f'<line x1="{cx}" y1="{cy}" x2="{cx}" y2="{cy + hyp + 48}" stroke="{G}" stroke-dasharray="4 3"/>'
    s += dim(cx, cy + hyp + 40, cx + hxp, cy + hyp + 40, 'hip_x', ly=cy + hyp + 34)
    s += f'<line x1="{cx + hxp}" y1="{cy - hyp}" x2="{cx + hxp + 76}" y2="{cy - hyp}" stroke="{G}" stroke-dasharray="4 3"/>'
    s += f'<line x1="{cx}" y1="{cy}" x2="{cx + hxp + 76}" y2="{cy}" stroke="{G}" stroke-dasharray="4 3"/>'
    s += dim(cx + hxp + 66, cy, cx + hxp + 66, cy - hyp, 'hip_y', lx=cx + hxp + 74, ly=cy - hyp / 2, anchor='start')
    s += notes(['● — точка, где ось бедра пересекает ось отведения (пунктир вдоль корпуса).',
                'Центр корпуса — середина между этими четырьмя точками; от него же меряются датчики.',
                'hip_x — от центра до передних (и задних) точек вдоль корпуса = половина расстояния перёд–зад.',
                'hip_y — от центра до оси отведения поперёк = половина расстояния лево–право.',
                'Датчики (sensors.* в robot.yaml): x вперёд, y влево, z вверх от центра; z = 0 на высоте оси бедра.'],
               h - 100, size=12)
    save('measure_body_top.svg', w, h, s)


# ---------------------------------------------------------------- linkage
def linkage():
    w, h = 700, 440
    s = text(20, 28, 'Тяга (сустав не на валу сервы): серва на ЦЕНТРАЛЬНОМ импульсе', size=15, bold=True)
    o1, o2, A, B = (200, 230), (460, 230), (200, 130), (460, 140)
    s += f'<line x1="{o1[0]}" y1="{o1[1]}" x2="{o2[0]}" y2="{o2[1]}" stroke="{G}" stroke-dasharray="6 4"/>'
    s += f'<line x1="{o1[0]}" y1="{o1[1]}" x2="{A[0]}" y2="{A[1]}" stroke="#555" stroke-width="9" stroke-linecap="round"/>'
    s += f'<line x1="{o2[0]}" y1="{o2[1]}" x2="{B[0]}" y2="{B[1]}" stroke="{LEG}" stroke-width="9" stroke-linecap="round"/>'
    s += f'<line x1="{A[0]}" y1="{A[1]}" x2="{B[0]}" y2="{B[1]}" stroke="{T}" stroke-width="4"/>'
    for p in (A, B):
        s += f'<circle cx="{p[0]}" cy="{p[1]}" r="6" fill="#fff" stroke="{T}" stroke-width="2"/>'
    s += axis(*o1, 10) + axis(*o2, 10)
    s += text(o1[0], o1[1] + 34, 'ось (вал) сервы', anchor='middle') + text(o2[0], o2[1] + 34, 'ось сустава', anchor='middle')
    s += dim(o1[0], o1[1] + 52, o2[0], o2[1] + 52, 'axis_distance_mm', ly=o1[1] + 46)
    s += dim(o1[0] - 26, o1[1], A[0] - 26, A[1], 'servo_arm_mm', lx=o1[0] - 34, ly=184, anchor='end')
    s += dim(o2[0] + 26, o2[1], B[0] + 26, B[1], 'joint_arm_mm', lx=o2[0] + 34, ly=188, anchor='start')
    s += dim(A[0], A[1] - 24, B[0], B[1] - 24, 'rod_mm', ly=A[1] - 32)
    s += f'<path d="M{o1[0] + 24},{o1[1]} A24,24 0 0,0 {o1[0]},{o1[1] - 24}" fill="none" stroke="{ORANGE}" stroke-width="2"/>'
    s += text(o1[0] + 30, o1[1] - 16, '90° на центре', col=ORANGE, size=12)
    s += notes(['Все длины — между центрами осей и шарниров. servo_arm_mm = 0 — серва прямо на оси сустава.',
                'joint_arm_mm = 0 — рычаг сустава такой же, как у сервы; rod_mm = 0 — тяга = axis_distance_mm.',
                'На центральном импульсе рычаг сервы ⟂ линии «ось сервы — ось сустава».',
                'Колено тянется от корпуса через бедро? Тогда coupled_to: <нога>_thigh_joint (CALIBRATION.md).'],
               h - 76, size=12)
    save('measure_linkage.svg', w, h, s)


# ---------------------------------------------------------------- IMU
def imu():
    w, h = 700, 380
    s = text(20, 28, 'IMU: что писать в axes (config/imu.yaml). Вид сверху, «вперёд» — вправо', size=15, bold=True)

    def board(x, y, sx, sy, title, spec):
        o = f'<rect x="{x - 50}" y="{y - 40}" width="100" height="80" rx="4" fill="#2f6f3e" opacity="0.85"/>'
        o += text(x, y + 64, title, bold=True, anchor='middle')
        o += (f'<text x="{x}" y="{y + 84}" fill="{BLUE}" text-anchor="middle" font-weight="bold" '
              f'font-family="DejaVu Sans Mono, monospace">axes: "{spec}"</text>')
        for (dx, dy), lab in ((sx, 'X'), (sy, 'Y')):
            o += f'<line x1="{x}" y1="{y}" x2="{x + dx}" y2="{y + dy}" stroke="#fff" stroke-width="3" marker-end="url(#k)"/>'
            o += text(x + dx * 1.3, y + dy * 1.3 + 5, lab, bold=True, anchor='middle')
        return o
    s += f'<line x1="600" y1="56" x2="670" y2="56" stroke="{T}" stroke-width="2" marker-end="url(#k)"/>' + text(635, 48, 'вперёд', anchor='middle')
    s += board(130, 150, (40, 0), (0, -30), 'X вперёд, Y влево', 'x,y,z')
    s += board(350, 150, (0, -30), (-40, 0), 'X влево, Y назад', '-y,x,z')
    s += board(570, 150, (0, 30), (40, 0), 'X вправо, Y вперёд', 'y,-x,z')
    s += notes(['Стрелки X/Y напечатаны на плате MPU6050; плата горизонтально, компонентами вверх (z вверх).',
                '"a,b,c": ось x корпуса = ось a датчика, y корпуса = b, z корпуса = c (со знаком).',
                'Проверка: нос вниз → тангаж +; левый бок вверх → крен +; поворот против часовой → ω_z +.'],
               h - 60, size=12)
    save('measure_imu_axes.svg', w, h, s)


if __name__ == '__main__':
    leg_side(), leg_rear(), body_top(), linkage(), imu()
