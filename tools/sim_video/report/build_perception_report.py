#!/usr/bin/env python3
"""Build report/perception.html: terrain sensors and the reaction to hazards.

  python3 tools/sim_video/report/build_perception_report.py --out report [--raw RENDER_DIR]

Scores come from data/perception.json (collect.py perception makes it from
perception_check recordings, listed in data/perception_runs.json). Videos:
<out>/videos/perception_<run>.mp4; --raw points at fresh renders named
<run>.mp4 (tools/sim_video/perception_video.py).
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
RUNS = {r['name']: r for r in json.load(open(os.path.join(cm.DATA, 'perception_runs.json'), encoding='utf-8'))}
SCORES = json.load(open(os.path.join(cm.DATA, 'perception.json'), encoding='utf-8'))

FEAT = {'stone_left': 'камень, левая линия', 'stone_right': 'камень, правая линия',
        'step_down': 'ступенька ↓', 'step_up': 'ступенька ↑', 'wall': 'стенка 80 мм'}


def FAMS(s):
    return (('lidar', 'лидары'), ('tof', 'ToF')) + ((('gs2', 'GS2'),) if 'gs2' in s.get('reports', {}) else ())


def scores(name):
    return SCORES[name]


def video(name):
    raw = next((d for d in args.raw if os.path.exists(os.path.join(d, f'{name}.mp4'))), None)
    src, poster = cm.video(args.out, f'perception_{name}', raw, crf=31, poster_at=0.65, raw_name=name)
    return poster, f' src="{src}"'


GUARD_RU = {'clear': 'свободно', 'caution': 'медленно', 'step_over': 'перешагивает', 'stop': 'стоп'}


def guard_chips(s):
    """Reaction runs: share of the walk in each guard state, wall gap, fall."""
    out = []
    g = s.get('guard') or {}
    for k in ('clear', 'caution', 'step_over', 'stop'):
        if g.get(k):
            cls = 'bad' if (k == 'stop' and s.get('terrain') not in ('wall',)) else 'skip'
            out.append(f'<li class="chip {cls}"><span>{GUARD_RU[k]}</span><b>{100 * g[k]:.0f} %</b></li>')
    w = s.get('wall')
    if w:
        cls = 'bad' if w['touched'] else 'ok'
        out.append(f'<li class="chip {cls}"><span>стопы до стенки</span><b>{w["front_feet_to_wall_m"]:.2f} м</b></li>')
    out.append(f'<li class="chip {"bad" if s.get("fell") else "ok"}"><span>падение</span>'
               f'<b>{"да" if s.get("fell") else "нет"}</b></li>')
    out.append(f'<li class="chip skip"><span>прошёл</span><b>{s.get("walked_m", 0):.2f} м</b></li>')
    return out


def chips(s):
    out = guard_chips(s) if 'guard' in s or s.get('terrain') == 'wall' else []
    if 'detection' in s:
        for f, v in s['detection'].items():
            if v.get('not_in_path'):
                out.append(f'<li class="chip skip"><span>{FEAT[f]}</span><b>не на пути</b></li>')
                continue
            if v.get('not_reached'):
                out.append(f'<li class="chip skip"><span>{FEAT[f]}</span><b>не дошёл</b></li>')
                continue
            for fam, lab in FAMS(s):
                x = v.get(fam)
                if x and x.get('not_reached'):
                    out.append(f'<li class="chip skip"><span>{FEAT[f]} · {lab}</span><b>не дошёл</b></li>')
                    continue
                cls = 'ok' if x else 'bad'
                val = f"{x['ahead_m']:.2f} м" if x else 'нет'
                out.append(f'<li class="chip {cls}"><span>{FEAT[f]} · {lab}</span><b>{val}</b></li>')
    r = s['reports']
    for fam, lab in FAMS(s):
        u = r[fam]['unexplained_per_m']
        cls = 'ok' if u == 0 else ('bad' if u > 2 else 'skip')
        out.append(f'<li class="chip {cls}"><span>ложные · {lab}</span><b>{u:g}/м</b></li>')
    gl = s.get('ground_lidar', {})
    out.append(f'<li class="chip skip"><span>пол по лидарам</span><b>±{gl.get("pitch_rms_deg")}°, '
               f'±{gl.get("height_rms_mm")} мм</b></li>')
    return '<ul class="chips">' + ''.join(out) + '</ul>'


def block(name, title=None):
    sub = RUNS[name]['setup']
    s = scores(name)
    poster, src = video(name)
    fell = s.get('fell', s.get('ground_feet', {}).get('roll_rms_deg', 0) > 30)
    head = title or sub
    return f'''<figure class="vid">
  <figcaption><h3>{head}</h3>{'<span class="badge bad">упал</span>' if fell else ''}</figcaption>
  <div class="player"><video data-key="{name}"{src} controls playsinline muted preload="none" poster="{poster}"></video></div>
  {chips(s)}
</figure>'''


plane_tbl = table(['Поверхность', 'Лидары: высота', 'Лидары: наклон', 'Ноги: высота', 'Ноги: наклон'], [
    ['Ровный пол', P('4 мм'), P('0.05°'), P('2 мм'), W('0.7–1.2°')],
    ['Уклон 10°', P('4 мм'), P('0.07°'), P('3 мм'), W('0.6–1.0°')],
    ['Камни 20 мм', P('6 мм'), P('0.15°'), W('3–9 мм'), F('1.2–3.7°')],
    ['Волны 20 мм', W('15 мм (гребни)'), P('0.23°'), W('11 мм'), F('4.3°')],
    ['Ступеньки', P('5–6 мм'), W('0.6–0.9°'), P('2–5 мм'), F('1.2–2.9°')]])
det_tbl = table(['Препятствие', 'Лидары: запас до стопы', 'Лидары: место', 'ToF: запас', 'ToF: датчик'], [
    ['Камень 20 мм на линии стопы', P('0.36–0.55 м'), P('±1–3 см'), P('0.10–0.28 м'), ('FL / FR', '')],
    ['Ступенька вниз 20–30 мм', P('0.56–0.73 м'), P('±7 см'), P('0.49–0.78 м'), ('FC', '')],
    ['Ступенька вверх 20–30 мм', P('0.55–0.57 м'), P('±0.5–2 см'), P('0.35–0.56 м'), ('FC', '')]])
fa_tbl = table(['Схема', 'Лидары, ложных на метр', 'ToF, ложных на метр', 'Камни 20 мм замечены ToF'], [
    ['<b>Лидары крестом + ToF, порог 15 мм</b>', P('0'), P('0'), P('3 из 4')],
    ['Лидары крестом + ToF, порог 20 мм', P('0'), P('0'), W('ступенька 20 мм на пороге')],
    ['Только ToF, опора на ноги, порог 30 мм', ('—', ''), W('3.8'), F('0 из 2')]])
map_tbl = table(['Поверхность', 'Ошибка ячеек', 'Ячеек точнее 10 мм'], [
    ['Ровный пол', P('7 мм'), P('89–91 %')], ['Уклон 10°', P('8 мм'), P('85 %')], ['Ступеньки', P('8 мм'), P('84–87 %')],
    ['Камни 20 мм', W('10 мм'), W('77–81 %')], ['Волны 20 мм', W('12 мм'), W('70 %')]])
gs2_tbl = table(['Препятствие / режим', 'GS2: запас до стопы', 'GS2: способ', 'Лидары: запас', 'ToF: запас'], [
    ['Камень 20 мм на линии стопы', W('0.07–0.13 м'), ('линия / плоскость', ''), P('0.48–0.58 м'), W('0.09–0.35 м, 1 пропуск')],
    ['Ступенька вниз 20–30 мм', W('0.14–0.15 м'), ('линия', ''), P('0.59–0.75 м'), P('0.17–0.75 м')],
    ['Ступенька вверх 20 мм', W('0.10–0.12 м'), ('плоскость', ''), P('0.52–0.57 м'), P('0.39–0.61 м')],
    ['Ложные, ровный пол', P('0 (с порогом 15 мм было 5.3 /м)'), ('только «плоскость»', ''), P('0'), P('0')],
    ['Ложные, уклон 10°', P('0.9 /м'), ('', ''), P('0'), F('11 /м')],
    ['Ложные, всё на опоре по ногам', P('0'), ('', ''), F('107 за прогон'), F('62 за прогон')]])
cpu_tbl = table(['Вариант узла', 'В симуляции (Xeon, 1200–1300 сообщ./с)', 'Частоты робота, ПК', 'Banana Pi, оценка (A53, × 4–6)'], [
    ['Python (было)', F('72 % ядра'), ('24 % ядра', ''), F('1–1.5 ядра из 4')],
    ['<b>C++ (сейчас)</b>', P('19 % ядра'), P('≈ 7 % ядра'), P('≈ 0.3–0.4 ядра')],
    ['C++, 4 ToF одним сообщением', ('—', ''), ('≈ 5.5 % ядра', ''), P('≈ 0.2–0.3 ядра')]])
guard_tbl = table(['Сценарий', 'С реакцией', 'Без реакции'], [
    ['Ровный пол', P('100 % «свободно»'), ('—', '')],
    ['Стенка 80 мм', P('стоп: стопы в 0.22 м'), F('упёрся в стенку')],
    ['Ступеньки 20 мм', P('2.6 м, без падения'), P('2.8 м, без падения')],
    ['Ступеньки 30 мм', P('2.4 м, без падения'), ('—', '')],
    ['Уклон 10°', W('0.5 м за 16 с (осторожно), без падения'), ('—', '')],
    ['Камни 20 мм', P('наклон ≤ 7°, без падения'), ('—', '')]])

body = f'''
<header>
  <div class="eyebrow">RobotDog 2.0 · Gazebo Harmonic · dog_perception · 25.09.2026</div>
  <h1>Лидары «крестом», VL53L1X, GS2 и реакция на препятствия</h1>
  <p>Два однолучевых лидара наклонены вниз-вбок крест-накрест (α = 30°, β = 40°), четыре VL53L1X смотрят вниз-вперёд и назад. Все прогоны сделаны в физической симуляции с тем же кодом управления, что и на роботе. Точки лидаров и лучи ToF в роликах — это то, что датчики реально измерили в симуляции.</p>
</header>
<div class="stats">
  <div class="stat"><div class="v">0.05<small>°</small></div><div class="k">точность наклона пола по лидарам (по ногам 0.7–1.2°)</div></div>
  <div class="stat"><div class="v">0.36–0.73<small> м</small></div><div class="k">запас до стопы, с которым лидары замечают камни 20 мм и ступеньки</div></div>
  <div class="stat"><div class="v">0<small> /м</small></div><div class="k">ложных срабатываний на ровном полу (лидары + ToF, порог 15 мм)</div></div>
  <div class="stat"><div class="v">0.3–0.4<small> ядра</small></div><div class="k">Banana Pi для узла на C++ (оценка по замеру; на Python 1–1.5 ядра)</div></div>
  <div class="stat"><div class="v">0.22<small> м</small></div><div class="k">стопы до стенки 80 мм при остановке; без реакции робот в неё упирается</div></div>
</div>
<div class="callout"><h3>Как читать ролики</h3>
  <div class="legend">
    <div><b>Точки</b> — оба лидара: серые — пол, оранжевые — выше пола, синие — ниже. На ровном полу видна буква X.</div>
    <div><b>Зелёные лучи</b> — VL53L1X до точки попадания; красный — датчик сработал.</div>
    <div><b>Справа</b> — что и с каким запасом обнаружено, остатки ToF (измерено − ожидалось), тангаж по лидарам и ногам против истины, сообщения о препятствиях.</div>
  </div>
  <p class="note">Ролики лежат рядом, в папке videos/; основной отчёт — <a href="index.html">index.html</a>.</p></div>

<section><div class="sec-head"><div class="eyebrow">мир steps: помост, камни 20 мм на линиях стоп, ступенька вниз и вверх</div>
  <h2><span class="num-badge">1</span>Камни и ступеньки</h2>
  <p>Главный тест: заметит ли робот препятствие до того, как на него наступит. «Запас» — расстояние от передней стопы до края в момент первого сообщения; при 0.12 м/с это около 8 с на метр.</p></div>
  <div class="pair">{block('steps30_t15', 'Ступеньки 30 мм · порог 15 мм')}{block('steps20_t15b', 'Ступеньки 20 мм · порог 15 мм')}</div>
  <div class="pair">{block('steps30_auto', 'Ступеньки 30 мм · порог 20 мм')}{block('steps20_auto', 'Ступеньки 20 мм · порог 20 мм')}</div>
  <div class="pair">{block('steps30_feet', 'Ступеньки 30 мм · только ToF и ноги')}{block('steps20_t15', 'Ступеньки 20 мм · порог 15 мм · робот споткнулся')}</div>
  <div class="panel"><h3>Итог по обнаружению</h3>{det_tbl}
    <p class="note">Лидары нашли все препятствия на пути во всех прогонах с порогом 15 мм. С порогом 20 мм ступенька ровно в 20 мм не ловится. «Не на пути» — камень, мимо которого робот прошёл сбоку после разворота от первого камня. В последнем ролике робот споткнулся о камень и упал: это ограничение походки, а не датчиков, и оба камня были замечены заранее.</p></div>
</section>

<section><div class="sec-head"><div class="eyebrow">ложные срабатывания и точность пола</div>
  <h2><span class="num-badge">2</span>Ровный пол</h2>
  <p>На ровном полу не должно быть ни одного сообщения. Сравниваются три настройки, в том числе ToF без опоры на лидары.</p></div>
  <div class="pair">{block('flat_t15', 'Лидары + ToF, порог 15 мм')}{block('flat_feet', 'Только ToF и ноги, порог 30 мм')}</div>
  <div class="pair">{block('flat_auto', 'Лидары + ToF, порог 20 мм')}<div class="panel"><h3>Ложные срабатывания</h3>{fa_tbl}
    <p class="note">Без лидаров опорная плоскость берётся по ногам и ошибается на 1–3°. Для FL/FR это 5–15 мм, для FC — 23 мм на градус. Порог приходится поднимать до 30 мм, и камни 20 мм пропускаются.</p></div></div>
</section>

<section><div class="sec-head"><div class="eyebrow">уклон 10°, камни и волны 20 мм</div>
  <h2><span class="num-badge">3</span>Уклон и неровности</h2>
  <p>На уклоне плоскость по лидарам повторяет пандус, и сообщения появляются только у начала подъёма. На камнях и волнах сообщения — это настоящие неровности.</p></div>
  <div class="pair">{block('slope10_t15', 'Уклон 10° · порог 15 мм')}{block('slope10_auto', 'Уклон 10° · порог 20 мм')}</div>
  <div class="pair">{block('rough20_t15', 'Камни 20 мм · порог 15 мм')}{block('rough20_auto', 'Камни 20 мм · порог 20 мм')}</div>
  <div class="pair">{block('waves20_auto', 'Волны 20 мм · порог 20 мм')}<div class="panel"><h3>Карта высот (3 × 3 м, ячейка 2 см)</h3>{map_tbl}</div></div>
</section>

<section><div class="sec-head"><div class="eyebrow">YDLIDAR GS2: линейный лазер + 2 камеры, веер 100°, 25–300 мм, 28 Гц</div>
  <h2><span class="num-badge">4</span>GS2 на передней грани</h2>
  <p>GS2 стоит в центре передней грани (x = 0.115 м, 0.15 м над полом), наклонён на 40° вниз. Его веер режет пол поперечной линией в 0.29 м от центра корпуса — это 0.14 м перед передними стопами — шириной ±0.19 м, то есть через обе линии стоп. В роликах это фиолетовая линия. Препятствие ищется двумя способами: <b>«линия»</b> — точки выходят из прямой, проведённой по самому профилю (опора не нужна, наклон корпуса не мешает); <b>«плоскость»</b> — весь коридор выше или ниже плоскости ног (так видна ступенька во всю ширину). Опора для GS2 — ноги: до линии всего 0.14 м, и 1° ошибки даёт там 2.5 мм.</p></div>
  <div class="pair">{block('steps20_gs2', 'GS2 · ступеньки 20 мм')}{block('steps30_gs2', 'GS2 · ступеньки 30 мм')}</div>
  <div class="pair">{block('steps20_gs2_feet', 'GS2 · ступеньки 20 мм · всё на опоре по ногам')}{block('flat_gs2', 'GS2 · ровный пол')}</div>
  <div class="pair">{block('slope10_gs2', 'GS2 · уклон 10°')}{block('rough20_gs2', 'GS2 · камни 20 мм')}</div>
  <div class="panel"><h3>GS2 против лидаров и ToF</h3>{gs2_tbl}
    <p class="note">В прогоне 30 мм робот остановился у ступеньки вверх раньше, чем до неё дошла линия GS2 — «не дошёл». Ложные на ровном полу — один эпизод 0.2 с способом «плоскость», когда плоскость ног ошиблась на 17 мм; способ «линия» не дал ни одного ложного во всех прогонах. Поэтому на роботе сообщение «линия» — стоп или перешагнуть, «плоскость» — только сбавить ход.</p></div>
  <div class="panel"><h3>Вывод по GS2</h3><ul class="list">
    <li><b>Заменить лидары «крестом» не может:</b> видит на 0.07–0.15 м вперёд стопы (около шага), лидары — на 0.4–0.75 м. Для остановки перед ступенькой на 0.12 м/с этого хватает (~1 с), для выбора пути — нет.</li>
    <li><b>Хорошо заменяет передние ToF (FL, FR, FC).</b> Одним датчиком закрывает всю ширину, а не три точки, и способом «линия» не зависит от опорной плоскости: на опоре только по ногам (как без лидаров) у GS2 0 ложных при пороге 12 мм, а ToF в том же режиме требовали 30 мм и пропускали камни.</li>
    <li><b>Вычисления:</b> 1.1–1.3 мс на скан в Python, ≈ 2500 точек/с, около 3 % ядра ПК при 25 Гц; на Banana Pi это ~0.1–0.2 ядра на Python и доли процента на C++.</li>
    <li><b>Рекомендация:</b> лидары «крестом» + GS2 спереди + один VL53L1X сзади (RC) для движения назад.</li>
  </ul></div>
</section>

<section><div class="sec-head"><div class="eyebrow">guard: медленнее, выше шаг у нужной ноги, стоп · узел на C++</div>
  <h2><span class="num-badge">5</span>Реакция на препятствия</h2>
  <p>Восприятие теперь не только сообщает о препятствиях, но и командует походке: предел скорости вперёд и высоту шага каждой ноги. Стоп — кромка выше 40 мм (по лидарам). Перешагнуть — кромка до 30 мм: высоко идёт только нога, на линии которой лежит препятствие. Медленно — всё остальное подтверждённое, в том числе с неизвестной высотой. В роликах внизу слева — состояние реакции, кольцо над стопой — нога с высоким шагом.</p></div>
  <div class="pair">{block('wall80_guard', 'Стенка 80 мм · с реакцией')}{block('wall80_noguard', 'Стенка 80 мм · без реакции')}</div>
  <div class="pair">{block('steps20_guard', 'Ступеньки 20 мм · с реакцией')}{block('steps20_noguard', 'Ступеньки 20 мм · без реакции')}</div>
  <div class="pair">{block('flat_guard', 'Ровный пол · с реакцией')}{block('steps30_guard', 'Ступеньки 30 мм · с реакцией')}</div>
  <div class="pair">{block('slope10_guard', 'Уклон 10° · с реакцией')}{block('rough20_guard', 'Камни 20 мм · с реакцией')}</div>
  <div class="panel"><h3>Итог</h3>{guard_tbl}
    <p class="note">Первые прогоны реакции ошибались: замедлялись 65–80 % времени на ровном полу, останавливались посреди поля (GS2 и ToF теряли пол при кивке корпуса), дважды опрокинули робота высоким шагом 40–50 мм и один раз дали ему уйти в стенку (сообщения «перешагнуть» вытеснили «стоп» из памяти). Всё это исправлено, подробности — docs/PERCEPTION.md и docs/REVIEW.md.</p></div>
</section>

<section><div class="sec-head"><div class="eyebrow">все прогоны</div>
  <h2><span class="num-badge">6</span>Точность плоскости пола</h2></div>
  <div class="panel">{plane_tbl}<p class="note">Лидары дают наклон пола в 10–20 раз точнее ног, ноги — высоту. Поэтому опорная плоскость объединённая: наклон по лидарам, высота сверяется по ногам (±15 мм), иначе берутся ноги.</p></div>
</section>

<section><div class="sec-head"><div class="eyebrow">замер в симуляции, пересчёт на частоты робота</div>
  <h2><span class="num-badge">7</span>Вычислительная нагрузка</h2>
  <p>Узел переписан на C++: пара сканов лидаров 0.62 мс вместо 1.4, скан GS2 0.16 мс вместо 1.18, приём сообщения 0.11 мс вместо 0.38. Частоты робота: суставы и IMU по 100 Гц, одометрия 25 Гц, ToF 4 × 50 Гц, лидары 2 × 10 Гц, GS2 28 Гц.</p></div>
  <div class="panel">{cpu_tbl}<p class="note">Пересчёт на A53 — оценка (одно ядро A53 в 4–6 раз медленнее ядра ПК); на самой плате не мерилось. Узел на C++ займёт около трети ядра — это можно держать рядом с походкой. Тяжёлое — не эта обработка, а 3D-карта большой площади и планирование шагов; их лучше считать на ноутбуке.</p></div>
</section>

<section><div class="sec-head"><div class="eyebrow">по ходу симуляции</div><h2><span class="num-badge">8</span>Что нашли и исправили</h2></div>
  <div class="panel"><ul class="list">
    <li><b>Сканы двух лидаров снимаются в разные моменты</b> (до 0.1 с), а корпус на рыси кивает около 10°/с. Без доворота по IMU плоскость ошибалась на 1°, с ним — 0.05°. На настоящем лидаре так же придётся доворачивать каждую точку оборота.</li>
    <li><b>Автокалибровка ToF при старте</b> запоминала пандус перед роботом как поправку (FC 130 мм), и весь путь становился «ямой». Теперь поправки снимаются один раз на ровном полу и хранятся в robot.yaml.</li>
    <li><b>Многопоточная линейная алгебра numpy</b> занимала все ядра ожиданием. Теперь ограничена одним потоком: на Banana Pi это отняло бы ядра у походки.</li>
    <li><b>Каждый замер сравнивается с положением корпуса в свой момент</b> (история IMU), а не с последним известным.</li>
    <li><b>GS2 на объединённой опоре не видел ступеньку:</b> плоскость лидаров к моменту подхода уже захватывала пол за ступенькой. Для GS2 опора — только ноги (линия в 0.14 м от стоп), после этого ступеньки 20 мм найдены обе.</li>
  </ul></div>
  <div class="panel"><h3>Ограничения модели</h3><ul class="list">
    <li>Лидар в Gazebo снимает оборот мгновенно, реальный — за 100 мс.</li>
    <li>VL53L1X смоделирован медианой 25 лучей. Чёрные и блестящие поверхности и солнце не учтены.</li>
    <li>Для карты высот взята точная одометрия симуляции. На роботе будет оценка, и карта со временем поплывёт.</li>
    <li>Углы установки лидаров и поправки ToF на роботе нужно калибровать: 1° ошибки угла даёт около 1 см высоты на 0.6 м.</li>
  </ul></div>
</section>
'''


html = (f'<!doctype html>\n<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" '
        f'content="width=device-width, initial-scale=1">\n<title>Восприятие RobotDog 2.0</title>\n'
        f'<style>{cm.template_css()}\ntd.r- {{ color: var(--ink-2); }}\n</style></head>\n<body><main>{body}</main>\n'
        + cm.template_script() + '\n</body></html>')
out = os.path.join(args.out, 'perception.html')
open(out, 'w', encoding='utf-8').write(html)
print(f'{out}: {len(html) / 1e3:.0f} kB')
