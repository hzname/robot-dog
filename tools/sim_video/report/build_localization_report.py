#!/usr/bin/env python3
"""Build report/localization.html: the map of the room from the lidars and
the robot finding itself in it.

  python3 tools/sim_video/report/build_localization_report.py --out report [--raw RENDER_DIR]

Scores come from data/localization.json (collect.py localization makes it
from localization_check recordings). Videos: <out>/videos/loc_{mapping,localize}.mp4;
--raw points at fresh renders of the same names (tools/sim_video/localization_video.py).
"""
import argparse
import json
import os

import common as cm
from common import P, W, table

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--out', default='report')
ap.add_argument('--raw', action='append', default=[])
args = ap.parse_args()
S = json.load(open(os.path.join(cm.DATA, 'localization.json'), encoding='utf-8'))


def video(name, title, run):
    raw = next((d for d in args.raw if os.path.exists(os.path.join(d, f'{name}.mp4'))), None)
    src, poster = cm.video(args.out, name, raw, crf=30, poster_at=0.8)
    s = S[run]
    lo, dr = s['localization'], s['dead_reckoning']
    chips = [('ok', 'ошибка p95', f"{100 * lo['p95_m']:.1f} см"), ('ok', 'курс p95', f"{lo['yaw_p95_deg']:.1f}°"),
             ('bad', 'счисление в конце', f"{100 * dr['final_m']:.0f} см"), ('skip', 'пройдено', f"{s['walked_m']:.1f} м")]
    if s['phase'] == 'localize':
        chips.insert(0, ('ok', 'нашёл себя', f"за {s['relocalized_s']:.1f} с"))
    chip_html = ''.join(f'<li class="chip {c}"><span>{k}</span><b>{v}</b></li>' for c, k, v in chips)
    return f'''<figure class="vid">
  <figcaption><h3>{title}</h3><span class="badge">×2</span></figcaption>
  <div class="player"><video data-key="{name}" src="{src}" controls playsinline muted preload="none" poster="{poster}"></video></div>
  <ul class="chips">{chip_html}</ul>
</figure>'''


def row(name, what):
    s = S[name]
    lo, dr, rl = s['localization'], s['dead_reckoning'], s.get('reloc')
    found = '—' if s['phase'] == 'mapping' else f"{s['relocalized_s']:.1f} с"
    margin = '—' if not rl else f"{rl['best']} % / {rl['second']} % ({rl['points']} точек)"
    return [what, (found, ''), (margin, ''), P(f"{100 * lo['p95_m']:.1f} см"), P(f"{lo['yaw_p95_deg']:.1f}°"),
            W(f"{100 * dr['final_m']:.0f} см")]


runs_tbl = table(['Прогон', 'Нашёл себя', 'На стенах: лучшая / вторая гипотеза', 'Ошибка p95', 'Курс p95',
                  'Счисление в конце'], [
    row('map0', 'Картирование, старт 0'),
    row('loc1', 'Локализация, старт 1'),
    row('loc2', 'Локализация, старт 2'),
    row('loc1ns', 'Старт 1 без осмотра')])
m, l1 = S['map0'], S['loc1']

body = f'''
<header>
  <div class="eyebrow">RobotDog 2.0 · Gazebo Harmonic · dog_perception/localization_node · 26.09.2026</div>
  <h1>Локализация по карте</h1>
  <p>Счисление по шагам и гироскопу уходит на полметра за круг по комнате. Верхние половины двух наклонённых лидаров видят стены и мебель; по ним робот строит 2D-карту комнаты, а потом, с любого места в ней, находит себя и идёт с ошибкой в пару сантиметров. Подробности — <a href="../docs/LOCALIZATION.md">docs/LOCALIZATION.md</a>.</p>
</header>
<div class="stats">
  <div class="stat"><div class="v">{100 * l1["localization"]["p95_m"]:.1f}<small> см</small></div><div class="k">ошибка положения (p95) по карте с другого старта</div></div>
  <div class="stat"><div class="v">{100 * l1["dead_reckoning"]["final_m"]:.0f}<small> см</small></div><div class="k">у одного счисления за тот же маршрут {l1["walked_m"]:.1f} м</div></div>
  <div class="stat"><div class="v">{l1["relocalized_s"]:.1f}<small> с</small></div><div class="k">после осмотра робот знает, где он в сохранённой карте</div></div>
  <div class="stat"><div class="v">{m["map_cells"]}</div><div class="k">клеток стен 5 см в карте комнаты 5 × 4 м</div></div>
</div>
<div class="callout"><h3>Как читать ролики</h3>
  <div class="legend">
    <div><b>Зелёное</b> — истинный путь, <b>синее</b> — где робот себя считает, <b>красный пунктир</b> — одно счисление.</div>
    <div><b>Чёрные точки</b> — стены сохранённой карты, <b>бежевое</b> — настоящая мебель и стены мира.</div>
    <div>Справа — ошибка положения во времени. Робот ведётся по истинной позе (это стенд), оценивается его собственная.</div>
  </div>
  <p class="note">Другие отчёты: <a href="index.html">ходьба и рельеф</a>, <a href="gaits.html">походки</a>, <a href="perception.html">датчики</a>.</p></div>

<section><div class="sec-head"><div class="eyebrow">первый запуск: карты нет</div>
  <h2><span class="num-badge">1</span>Построение карты</h2>
  <p>Робот встаёт, делает осмотр (корпус качается вверх-вниз и влево-вправо, лидары проходят по стенам) и идёт по кругу. Каждый скан сопоставляется с уже построенной частью карты и дописывается в неё. Счисление с гироскопом, уходящим на 0,3°/с, к концу круга ошибается на {100 * m["dead_reckoning"]["final_m"]:.0f} см и {m["dead_reckoning"]["yaw_p95_deg"]:.0f}°. Карта при этом не расползается: стены ложатся на мир в пределах пары сантиметров.</p></div>
  {video('loc_mapping', 'Построение карты комнаты', 'map0')}
</section>

<section><div class="sec-head"><div class="eyebrow">следующий запуск: карта есть, где робот — неизвестно</div>
  <h2><span class="num-badge">2</span>Привязка и локализация</h2>
  <p>Робот поставлен в другое место комнаты и развёрнут. Пока он осматривается, точки копятся в кадре счисления. Затем перебираются все свободные места карты через 10 см и все курсы через 3°, лучшие уточняются. Привязка принимается, если на стенах больше половины точек и вторая по качеству гипотеза заметно хуже. Дальше каждый скан подтягивает позу к стенам.</p></div>
  {video('loc_localize', 'Локализация с другого старта', 'loc1')}
  <div class="panel">{runs_tbl}</div>
  <div class="panel"><h3>Что видно по прогонам</h3><ul class="list">
    <li><b>Осмотр нужен для надёжности, а не для точности.</b> Без него робот тоже нашёл себя, но по {S["loc1ns"]["reloc"]["points"]} точкам вместо {S["loc1"]["reloc"]["points"]}, и вторая гипотеза набрала {S["loc1ns"]["reloc"]["second"]} % против {S["loc1ns"]["reloc"]["best"]} % (с осмотром — {S["loc1"]["reloc"]["second"]} против {S["loc1"]["reloc"]["best"]}). Порог неоднозначности — 90 % от лучшей. В комнате посимметричнее без осмотра привязка откажется или ошибётся.</li>
    <li><b>Ошибки сравниваются в одном кадре.</b> Кадр карты — там, где робот стоял в начале картирования, по его счислению. Прогон картирования подбирает жёсткое совмещение карты с миром (она оказалась повёрнута на 3°), и тот же сдвиг используется для всех стартов. Поэтому ошибка локализации — это ещё и искажение самой карты.</li>
    <li><b>Курс</b> ошибается на 2–3° p95, в среднем около 1°: откуда остаток, ещё не разобрано (кандидаты — рыскание корпуса в рыси между сканами и задержка скана относительно счисления).</li>
  </ul></div>
</section>

<section><div class="sec-head"><div class="eyebrow">чего это ещё не умеет</div><h2><span class="num-badge">3</span>Ограничения</h2></div>
  <div class="panel"><ul class="list">
    <li>Карта плоская: стены и мебель от 10 см до 2 м над полом, спроецированные на пол. Для ковра, порогов и ступеней по-прежнему карта высот охранника.</li>
    <li>Без замыкания петель: в большом доме карта будет накапливать искажение. Комната 5 × 4 м проходится с ошибкой меньше 2 см.</li>
    <li>Длинный пустой коридор задаёт только поперечное положение, вдоль него держится счисление (тест <code>CorridorKeepsThePriorAlongIt</code>).</li>
    <li>Люди и передвинутая мебель не удаляются из карты: при сопоставлении такие точки просто не попадают на стены.</li>
    <li>Глобальный поиск по комнате — около 1 с на ПК. На A53 будет в разы дольше; для большого дома нужен поиск от грубой сетки к мелкой.</li>
  </ul></div>
</section>
'''

html = (f'<!doctype html>\n<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" '
        f'content="width=device-width, initial-scale=1">\n<title>Локализация RobotDog 2.0</title>\n'
        f'<style>{cm.template_css()}\n</style></head>\n<body><main>{body}</main>\n'
        + cm.template_script() + '\n</body></html>')
out = os.path.join(args.out, 'localization.html')
open(out, 'w', encoding='utf-8').write(html)
print(f'{out}: {len(html) // 1024} kB')
