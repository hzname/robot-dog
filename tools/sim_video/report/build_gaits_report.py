#!/usr/bin/env python3
"""Build report/gaits.html: the crawl, going round, stopping, the greeting.

  python3 tools/sim_video/report/build_gaits_report.py --out report [--raw RENDER_DIR]

Scores come from data/gaits.json (collect.py gaits makes it from
perception_check recordings, listed in data/gaits_runs.json). Videos:
<out>/videos/gaits_<run>.mp4; --raw points at fresh renders named <run>.mp4
(tools/sim_video/perception_video.py, with the run's --speed).
"""
import argparse
import json
import os

import common as cm
from common import F, P, W, table

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--out', default='report')
ap.add_argument('--raw', action='append', default=[])
args = ap.parse_args()
RUNS = {r['name']: r for r in json.load(open(os.path.join(cm.DATA, 'gaits_runs.json'), encoding='utf-8'))}
SCORES = json.load(open(os.path.join(cm.DATA, 'gaits.json'), encoding='utf-8'))

GUARD_RU = {'clear': 'свободно', 'caution': 'медленно', 'step_over': 'высокий шаг', 'crawl': 'ползком',
            'stop': 'стоп', 'avoid': 'объезд'}


def video(name):
    raw = next((d for d in args.raw if os.path.exists(os.path.join(d, f'{name}.mp4'))), None)
    src, poster = cm.video(args.out, f'gaits_{name}', raw, crf=31, poster_at=0.55, raw_name=name)
    return poster, f' src="{src}"'


def chip(cls, k, v):
    return f'<li class="chip {cls}"><span>{k}</span><b>{v}</b></li>'


def chips(s):
    out = []
    g = s.get('guard') or {}
    for k in ('clear', 'caution', 'step_over', 'crawl', 'avoid', 'stop'):
        if g.get(k, 0) >= 0.005:
            out.append(chip('skip', GUARD_RU[k], f'{100 * g[k]:.0f} %'))
    c = s.get('crossing')
    if c:
        if 'top_reached' in c:
            out.append(chip('ok' if c['top_reached'] else 'bad', 'подъём', 'да' if c['top_reached'] else 'нет'))
            out.append(chip('ok' if c['down_reached'] else 'bad', 'спуск', 'да' if c['down_reached'] else 'нет'))
        else:
            out.append(chip('ok' if c['crossed'] else 'bad', 'прошёл препятствие', 'да' if c['crossed'] else 'нет'))
        if 'back_on_line' in c:
            out.append(chip('ok' if c['back_on_line'] else 'bad', 'вернулся на линию',
                            'да' if c['back_on_line'] else 'нет'))
        if 'min_gap_m' in c:
            out.append(chip('bad' if c.get('touched') else 'ok', 'зазор до блока', f"{c['min_gap_m']:.2f} м"))
    w = s.get('wall')
    if w:
        out.append(chip('bad' if w['touched'] else 'ok', 'стопы до стены', f"{w['front_feet_to_wall_m']:.2f} м"))
    gr = s.get('greet')
    if gr:
        out.append(chip('ok' if gr.get('finished') else 'bad', 'встал обратно', 'да' if gr.get('finished') else 'нет'))
        out.append(chip('ok' if gr.get('max_nose_up_deg', 0) >= 25 else 'bad', 'нос вверх',
                        f"{gr.get('max_nose_up_deg', 0):.0f}°"))
        out.append(chip('skip', 'длительность', f"{gr.get('seconds', 0):.0f} с"))
        out.append(chip('skip', 'сдвиг', f"{gr.get('drift_m', 0):.2f} м"))
    out.append(chip('bad' if s.get('fell') else 'ok', 'падение', 'да' if s.get('fell') else 'нет'))
    if not gr:
        out.append(chip('skip', 'прошёл', f"{s.get('walked_m', 0):.2f} м"))
    return '<ul class="chips">' + ''.join(out) + '</ul>'


def block(name, title=None):
    r = RUNS[name]
    s = SCORES[name]
    poster, src = video(name)
    speed = f'<span class="badge">×{r["speed"]:g}</span>' if r.get('speed', 1) != 1 else ''
    fell = '<span class="badge bad">упал</span>' if s.get('fell') else ''
    return f'''<figure class="vid">
  <figcaption><h3>{title or r['setup']}</h3>{speed}{fell}</figcaption>
  <div class="player"><video data-key="{name}"{src} controls playsinline muted preload="none" poster="{poster}"></video></div>
  {chips(s)}
</figure>'''


rules_tbl = table(['Уступ вверх / вниз', 'Что делает робот'], [
    ['до 15 мм', ('ничего: шум', '')],
    ['до 25 / 35 мм', ('рысь; высокий шаг (до 30 мм) только у ноги, перед которой уступ', '')],
    ['до 70 мм', P('ползание: одна нога в воздухе, корпус заранее над тремя другими')],
    ['выше 70 мм', W('стоп; узкое (объезд ≤ 0,6 м, верх выше 100 мм по карте) — объезд, иначе ждёт оператора')]])
result_tbl = table(['Мир', 'Итог', 'Почему'], [
    ['Ступени 30 мм', P('проходит'), ('рысь с высоким шагом и ползание на кромках', '')],
    ['Стена 80 мм', P('стоп, объезда нет'), ('стопы в ~0,2 м от стены', '')],
    ['Блок 150 мм', P('объезд и возврат на линию'), ('в сторону → мимо → назад на линию', '')],
    ['Брус 60 мм', W('1 из 3'), ('ползание: задние ноги цепляют брус, корпус садится на него', '')],
    ['Лестница 50 мм', W('подъём 3 из 3, спуск 1 из 3'), ('на спуске опорные стопы сползают к кромке, робот срывается', '')],
    ['Приветствие', P('проходит'), ('на коленях, нос вверх до 30°, ~34 с, сдвиг ~0,2 м', '')]])

body = f'''
<header>
  <div class="eyebrow">RobotDog 2.0 · Gazebo Harmonic · dog_control + dog_perception · 26.09.2026</div>
  <h1>Походки: ползание, объезд, приветствие</h1>
  <p>Рысь с высоким шагом проходит уступы до 25 мм. Всё, что выше, требует другой походки или обхода. Здесь прогоны в физической симуляции с тем же кодом, что и на роботе: ползание на трёх опорах через ступени, брус и лестницу; остановка у стены; объезд узкого блока; поза «сидит, лапы вверх». Подробности — <a href="../docs/GAITS.md">docs/GAITS.md</a>.</p>
</header>
<div class="stats">
  <div class="stat"><div class="v">~1.1<small> см/с</small></div><div class="k">скорость ползания: шаг 10 см за цикл из четырёх ног, уступы до 70 мм</div></div>
  <div class="stat"><div class="v">{SCORES["wall80"]["wall"]["front_feet_to_wall_m"]:.2f}<small> м</small></div><div class="k">стопы до стены 80 мм при остановке</div></div>
  <div class="stat"><div class="v">{SCORES["block150"]["crossing"]["min_gap_m"]:.2f}<small> м</small></div><div class="k">наименьший зазор до блока 150 мм при объезде</div></div>
  <div class="stat"><div class="v">{SCORES["greet"]["greet"]["max_nose_up_deg"]:.0f}<small>°</small></div><div class="k">нос вверх в приветствии, лапы в воздухе, опора — колени и задние стопы</div></div>
</div>
<div class="callout"><h3>Как читать ролики</h3>
  <div class="legend">
    <div><b>Внизу слева</b> — состояние охранника: свободно, медленно, высокий шаг, ползком, стоп, объезд.</div>
    <div><b>Точки</b> — лидары, <b>зелёные лучи</b> — ToF, <b>фиолетовая линия</b> — GS2. Кольцо над стопой — нога с высоким шагом.</div>
    <div><b>×N</b> в заголовке — ролик ускорен: ползание медленное.</div>
  </div>
  <p class="note">Другие отчёты: <a href="index.html">ходьба и рельеф</a>, <a href="perception.html">датчики</a>. Ролики лежат в папке videos/.</p></div>

<section><div class="sec-head"><div class="eyebrow">охранник выбирает походку по высоте уступа</div>
  <h2><span class="num-badge">1</span>Какая походка когда</h2>
  <p>Решение принимается по скачку профиля за 3–6 см пути, а не по высоте над полом. Походка меняется только на месте: скорость обнуляется, переключение — когда шаг закончен.</p></div>
  <div class="panel">{rules_tbl}</div>
</section>

<section><div class="sec-head"><div class="eyebrow">одна нога в воздухе, опоры выбираются по профилю рельефа</div>
  <h2><span class="num-badge">2</span>Ползание</h2>
  <p>Перед подъёмом ноги корпус смещается так, чтобы центр масс был внутри треугольника трёх других ног с запасом 3,5 см. Стопа ставится туда, где в окне ±4,5 см перепад меньше 8 мм. Нога идёт вверх, потом вперёд, потом вниз, на 3 см выше самой высокой точки на пути.</p></div>
  <div class="pair">{block('steps30')}{block('bar60')}</div>
  <div class="pair">{block('stairs50')}{block('stairs50b')}</div>
  <div class="panel"><h3>Где ползание пока ненадёжно</h3><ul class="list">
    <li><b>Лестница 50 мм:</b> подъём и верхняя площадка — 3 из 3, спуск — 1 из 3. На спуске опорные стопы понемногу сползают к кромке, и робот срывается со ступени (второй ролик).</li>
    <li><b>Брус 60 мм:</b> 1 из 3. Передние ноги переходят, задние цепляют брус или корпус садится на него.</li>
    <li>Обе задачи — в <code>dog_control</code>, в CI их нет. Ступени 30 мм, стена и приветствие проверяются в CI на каждый коммит.</li>
  </ul></div>
</section>

<section><div class="sec-head"><div class="eyebrow">выше 70 мм: стоп или объезд по карте высот</div>
  <h2><span class="num-badge">3</span>Стоп и объезд</h2>
  <p>Объезжается только то, что карта уверенно видит высоким: средние высоты клеток выше 100 мм. Ширину даёт передняя грань. Робот отходит вбок так, чтобы коридор 0,2 м был свободен, идёт мимо, держа смещение, и возвращается на линию, по которой шёл. Стена 80 мм едва выше предела ползания и видна картой ненадёжно: у неё робот просто стоит.</p></div>
  <div class="pair">{block('wall80')}{block('block150')}</div>
</section>

<section><div class="sec-head"><div class="eyebrow">команда greet: кнопка «Привет», клавиша 3, Y / Triangle</div>
  <h2><span class="num-badge">4</span>Приветствие</h2>
  <p>Две задние стопы — линия, а не опора, поэтому робот садится на колени задних ног: голени лежат на полу, колени и стопы образуют опору 9 см. Задние лапы по одной переставляются вперёд, корпус откидывается на 20°, передние лапы поднимаются, нос уходит до 30°, левая лапа машет два раза, потом всё в обратном порядке.</p></div>
  <div class="pair">{block('greet')}<div class="panel"><h3>На роботе</h3><ul class="list">
    <li>Колени касаются пола. Перед первым запуском проверьте, что у колена нет деталей, которые упрутся в пол; при необходимости наклейте резиновую накладку.</li>
    <li>Сначала на подставке, потом на полу со страховкой рукой (docs/DEPLOYMENT.md, этап 14).</li>
    <li>Сдвиг за приветствие ~0,2 м и разворот до 30° — от перестановки задних лап; на ковре меньше, на гладком полу больше.</li>
  </ul></div></div>
</section>

<section><div class="sec-head"><div class="eyebrow">по прогонам</div><h2><span class="num-badge">5</span>Итог</h2></div>
  <div class="panel">{result_tbl}</div>
  <div class="panel"><h3>Что нашли и исправили по ходу</h3><ul class="list">
    <li><b>Лидар видел собственную ногу</b>, поднятую над брусом, и клал её в карту как высокое препятствие. Точки у бедра и голени каждой ноги теперь выбрасываются по кинематике.</li>
    <li><b>Смена походки зависала:</b> удержание курса поворачивало ползание, и оно не вставало на месте; опоры у блока на смазанном крае карты не были «на одном уровне» в пределах 1 см. Теперь на время смены курс не держится, а в рысь можно при разбросе до 25 мм.</li>
    <li><b>Объезд стены:</b> шумные клетки 80-мм стены выглядели узким блоком. Теперь объезжается только уверенно высокое, а ширина пересчитывается по ходу.</li>
    <li><b>GS2 на ровном полу</b> давал ложный «провал» в первые полсекунды ходьбы (корпус качается на 3–4°). Подтверждение — 8 сканов вместо 3.</li>
    <li><b>Проверки в CI</b> считали настенное время и делили ROS-домен; теперь время симуляции и свой домен на шаг.</li>
  </ul></div>
</section>
'''

html = (f'<!doctype html>\n<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" '
        f'content="width=device-width, initial-scale=1">\n<title>Походки RobotDog 2.0</title>\n'
        f'<style>{cm.template_css()}\n</style></head>\n<body><main>{body}</main>\n'
        + cm.template_script() + '\n</body></html>')
out = os.path.join(args.out, 'gaits.html')
open(out, 'w', encoding='utf-8').write(html)
print(f'{out}: {len(html) / 1e3:.0f} kB')
