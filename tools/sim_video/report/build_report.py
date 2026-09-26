#!/usr/bin/env python3
"""Build report/index.html: the full RobotDog 2.0 test cycle.

  python3 tools/sim_video/report/build_report.py --out report [--raw RENDER_DIR ...]

Numbers come from data/walk.json and data/tests.json (collect.py makes them
from simulation recordings and colcon test results). Videos are taken from
<out>/videos; --raw points at folders with fresh renders (tools/sim_video/
render.py, calib_video.py), which are then re-encoded into <out>/videos.
"""
import argparse
import json
import os
import statistics

import common as cm
from common import F, P, W, X, table

NAMES = {'stand': 'Встаёт', 'forward': 'Вперёд', 'backward': 'Назад', 'left': 'Влево', 'right': 'Вправо',
         'turn_ccw': 'Разворот ↺', 'turn_cw': 'Разворот ↻',
         'arc_left': 'Дуга влево', 'arc_right': 'Дуга вправо', 'lie': 'Ложится'}
SLOPE = {'forward': 'Подъём', 'backward': 'Спуск', 'left': 'Влево поперёк', 'right': 'Вправо поперёк'}
PKG_RU = {'dog_control': 'кинематика, походка, режимы', 'dog_hardware': 'сервы, датчик тока, IMU',
          'dog_teleop': 'геймпад, клавиатура', 'dog_web': 'веб-пульт, калибровка по сети',
          'dog_bringup': 'весь стек целиком', 'dog_description': 'модель робота (URDF)',
          'dog_gazebo': 'миры и проверки для симуляции', 'dog_perception': 'лидары, ToF, GS2, реакция'}


def chips(kind, res):
    out = []
    for r in res:
        name = (SLOPE if kind == 'slope' else {}).get(r['name'], NAMES.get(r['name'], r['name']))
        if r.get('fallen'):
            cls, val = 'skip', 'пропущен'
        elif r.get('tilt_deg', 0) > 60:
            cls, val = 'bad', 'упал'
        elif 'ratio' in r:
            cls, val = ('ok' if r['ok'] else 'bad'), f'{100 * r["ratio"]:.0f}%'
        else:
            cls, val = ('ok' if r['ok'] else 'bad'), ('да' if r['ok'] else 'нет')
        out.append(f'<li class="chip {cls}"><span>{name}</span><b>{val}</b></li>')
    ok = sum(r['ok'] for r in res)
    return f'<ul class="chips">{"".join(out)}</ul>', ok, len(res)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='report', help='report folder (index.html, videos/, posters/)')
    ap.add_argument('--raw', action='append', default=[], help='folder with fresh raw renders (repeatable)')
    args = ap.parse_args()
    walk = json.load(open(os.path.join(cm.DATA, 'walk.json'), encoding='utf-8'))
    tests = json.load(open(os.path.join(cm.DATA, 'tests.json'), encoding='utf-8'))

    def raw_dir(name):
        return next((d for d in args.raw if os.path.exists(os.path.join(d, f'{name}.mp4'))), None)

    def vid_block(key, name, title, rec=None, note='', crf=32):
        src, poster = cm.video(args.out, name, raw_dir(name), crf)
        head = foot = ''
        if rec:
            ch, ok, n = chips(walk[rec]['terrain'], walk[rec]['results'])
            badge = 'ok' if ok == n else ('bad' if ok <= n // 2 else 'warn')
            head, foot = f'<span class="badge {badge}">{ok}/{n}</span>', ch
        return f'''<figure class="vid">
  <figcaption><h3>{title}</h3>{head}</figcaption>
  <div class="player"><video data-key="{key}" src="{src}" controls playsinline muted preload="none" poster="{poster}"></video></div>
  {foot}
  {f'<p class="note">{note}</p>' if note else ''}
</figure>'''

    # tests
    cnt = tests['counts']
    rows = [[f'<span class="mono">{pkg}</span><div class="sub">{PKG_RU.get(pkg, "")}</div>',
             P(f'{cnt["jazzy"].get(pkg, 0)} ✔'), P(f'{cnt["lyrical"].get(pkg, 0)} ✔')]
            for pkg in sorted(set(cnt['jazzy']) | set(cnt['lyrical']))]
    tests_tbl = table(['Пакет', 'Jazzy', 'Lyrical'], rows)

    # calibration demo (seed 1): joint, default zero, true zero, found zero [deg]
    cal = [('lf_hip', 0.0, 0.21, 0.60), ('rf_hip', 0.0, 5.90, 6.79), ('lr_hip', 0.0, -3.06, -3.12),
           ('rr_hip', 0.0, -5.34, -5.78), ('lf_thigh', 45, 38.59, 38.65), ('lf_calf', -90, -93.39, -93.41),
           ('lr_thigh', 45, 41.46, 41.25), ('lr_calf', -90, -96.59, -96.03), ('rf_thigh', 45, 45.89, 46.38),
           ('rf_calf', -90, -85.44, -85.56), ('rr_thigh', 45, 49.51, 49.91), ('rr_calf', -90, -90.27, -90.58)]
    cal_rows = []
    for j, default, true, found in cal:
        rev = ' <span class="tag">серва стояла наоборот, направление исправлено</span>' if j == 'rr_thigh' else ''
        cal_rows.append([f'<span class="mono">{j}</span>{rev}', W(f'{true - default:+.1f}°'), P(f'{found - true:+.1f}°')])
    cal_tbl = table(['Сустав', 'Ошибка нуля до', 'После калибровки'], cal_rows)

    # sweeps: summarised by hand from docs/TERRAIN.md
    slope_tbl = table(['Уклон', 'Подъём', 'Влево', 'Вправо', 'Разв. ↺', 'Разв. ↻', 'Спуск', 'Наклон'], [
        ['0° ×2', P('97–110%'), P('75–84%'), P('72–92%'), P('86–102%'), P('81–103%'), P('53–70%'), P('9°')],
        ['8°', P('97%'), P('79%'), P('74%'), P('103%'), P('56%'), P('65%'), P('13°')],
        ['10° ×4', P('85–106%'), P('66–74%'), P('76–84%'), P('78–100%'), P('67–82%'), P('71–80%'), P('14°')],
        ['12° ×2', P('77–91%'), P('91–120%'), P('46–50%'), P('89–93%'), P('82–88%'), P('54–56%'), P('13°')],
        ['15° ×2', P('81–82%'), W('24–102%'), W('сполз / 121%'), P('40–106%'), P('79–106%'), W('12–65%'), W('19°')],
        ['18°', P('86%'), F('37%'), F('накл. 21°'), F('4%'), P('86%'), F('16%'), F('23°')],
        ['20°', X, None, None, None, None, None, None]])
    off_tbl = table(['Уклон', 'Подъём', 'Влево', 'Вправо', 'Разв. ↺', 'Разв. ↻', 'Спуск'], [
        ['6°', P('да'), P('да'), P('да'), P('да'), P('да'), P('да')],
        ['8°', P('да'), P('да'), P('да'), P('да'), P('да'), X],
        ['10° ×2', W('72% / упал'), F('32%'), X, None, None, None],
        ['12°', P('64%'), X, None, None, None, None],
        ['15° ×2', X, None, None, None, None, None]])
    waves_tbl = table(['Высота', 'Вперёд', 'Назад', 'Влево', 'Вправо', 'Разв. ↺', 'Разв. ↻', 'Наклон'], [
        ['10 мм ×3', P('97%'), W('17–36%'), P('60%'), P('66%'), P('80%'), P('83%'), P('10°')],
        ['20 мм ×2', P('58–59%'), F('5–13%'), W('25–49%'), P('51–76%'), W('13–29%'), W('20–41%'), P('13°')],
        ['30 мм', P('43%'), F('2%'), P('76%'), P('56%'), W('30%'), W('31%'), W('18°')],
        ['40 мм', F('накл. 27°'), F('1%'), P('45%'), F('накл. 21°'), F('2%'), F('2%'), F('27°')]])
    rough_tbl = table(['Высота', 'Вперёд', 'Назад', 'Влево', 'Вправо', 'Разв. ↺', 'Разв. ↻', 'Наклон'], [
        ['10 мм ×3', P('70%'), W('36%'), P('43%'), P('47%'), P('48%'), P('41%'), P('9°')],
        ['20 мм ×2', P('58–64%'), W('8–28%'), W('23–55%'), W('37–38%'), P('44–50%'), P('51–55%'), P('14°')],
        ['30 мм', P('58%'), W('34%'), W('34%'), P('45%'), W('32%'), F('17%'), W('17°')],
        ['40 мм', P('62%'), F('14%'), W('35%'), W('26%'), P('46%'), W('27%'), W('16°')]])
    lim_tbl = table(['Манёвр', 'Уклон с IMU', 'Уклон без IMU', 'Волны', 'Камни'], [
        ['Вперёд, подъём', P('≤ 15°'), W('≤ 6°'), P('≤ 30 мм'), P('≤ 30 мм')],
        ['Назад, спуск', P('≤ 15°'), W('≤ 6°'), W('≤ 10 мм, ползком'), W('≤ 20 мм, медленно')],
        ['Вбок поперёк склона', P('≤ 12°'), W('≤ 6°'), P('≤ 10 мм'), P('≤ 20 мм')],
        ['Разворот на месте', P('≤ 15°'), W('≤ 6°'), P('≤ 10 мм'), P('≤ 20 мм')],
        ['Любые манёвры', P('≤ 10°'), W('≤ 6°'), P('≤ 10 мм'), P('≤ 10 мм')],
        ['<b>Реальный робот: начать с</b>', ('7°', ''), ('4°', ''), ('7 мм', ''), ('7 мм', '')]])

    blocks = {
        'cal': vid_block('cal', 'calibration', 'Автокалибровка: 4 ракурса × 2 прохода, 240 измерений',
                         note='Слева кадр виртуальной камеры: зелёные рамки — найденные метки ArUco, линии — '
                              'бедро, колено, стопа. Под ним робот на подставке и положение камеры. Справа '
                              'текущий шаг, точки «импульс → угол» и подобранная модель сервы, внизу ошибка нуля '
                              'каждого сустава до и после.'),
        'flat': vid_block('flat', 'flat', 'Ровный пол', 'on/flat_0.json'),
        's10on': vid_block('s10on', 'slope10_on', 'Уклон 10° · IMU включена', 'on/slope_10.json'),
        's10off': vid_block('s10off', 'slope10_off', 'Уклон 10° · без IMU', 'off/slope_10.json'),
        's15on': vid_block('s15on', 'slope15_on', 'Уклон 15° · IMU включена', 'on/slope_15.json'),
        's15off': vid_block('s15off', 'slope15_off', 'Уклон 15° · без IMU', 'off/slope_15.json'),
        'w20on': vid_block('w20on', 'waves20_on', 'Волны 20 мм · IMU включена', 'on/waves_20.json'),
        'w20off': vid_block('w20off', 'waves20_off', 'Волны 20 мм · без IMU', 'off/waves_20.json'),
        'r20on': vid_block('r20on', 'rough20_on', 'Камни до 20 мм · IMU включена', 'on/rough_20.json'),
        'r20off': vid_block('r20off', 'rough20_off', 'Камни до 20 мм · без IMU', 'off/rough_20.json'),
        'arcflat': vid_block('arcflat', 'arc_flat', 'Дуга · ровный пол', 'arc/rec/flat_0.json'),
        'arcwaves': vid_block('arcwaves', 'arc_waves', 'Дуга · волны 10 мм', 'arc/rec/waves_10.json'),
        'arcrough': vid_block('arcrough', 'arc_rough', 'Дуга · камни 10 мм', 'arc/rec/rough_10.json'),
    }
    arc_rows = []
    for key, lab in [('arc/rec/flat_0.json', 'Ровный пол'), ('arc/rec/waves_10.json', 'Волны 10 мм'),
                     ('arc/rec/rough_10.json', 'Камни 10 мм')]:
        res = {r['name']: r for r in walk[key]['results']}
        row = [lab]
        for m in ('arc_left', 'arc_right'):
            r = res[m]
            row.append((P if r['ok'] else F)(f"{100 * r['ratio']:.0f}% · {r['dyaw_deg']:+.0f}°"))
        row.append(P(f"{max(res[m]['tilt_deg'] for m in ('arc_left', 'arc_right')):.0f}°"))
        arc_rows.append(row)
    arc_tbl = table(['Покрытие', 'Дуга влево', 'Дуга вправо', 'Наклон корпуса'], arc_rows)
    file_ = {'flat': 'flat_0', 'slope': 'slope_10', 'waves': 'waves_20', 'rough': 'rough_20'}
    title = {'flat': 'Ровный пол', 'slope': 'Уклон 10°', 'waves': 'Волны 20 мм', 'rough': 'Камни до 20 мм'}
    heading = [f'on_flat_{i}' for i in (1, 2, 3)] + [f'off_flat_{i}' for i in (1, 2, 3)] + \
        ['on_slope', 'on_slope_2', 'on_slope_3', 'off_slope', 'on_waves', 'off_waves', 'on_rough', 'off_rough']
    for name in heading:
        kind = name.split('_')[1]
        run = name.split('_')[2] if name.count('_') == 2 else ''
        lab = 'удержание ВКЛ' if name.startswith('on') else 'удержание ВЫКЛ'
        blocks['h' + name] = vid_block('h' + name, name, f'{title[kind]} · {lab}' + (f' · прогон {run}' if run else ''),
                                       f'rec/{name}/{file_[kind]}.json', crf=34)

    def head_stats(names):
        res = [r for n in names for r in walk[f'rec/{n}/{file_[n.split("_")[1]]}.json']['results']]
        st = [abs(r['dyaw_deg']) for r in res
              if r['name'] in ('forward', 'backward', 'left', 'right') and 'dyaw_deg' in r and not r.get('fallen')]
        tu = [100 * r['ratio'] for r in res if r['name'].startswith('turn') and 'ratio' in r]
        return statistics.mean(st), max(st), min(tu), max(tu)

    hrows = []
    for kind, on, off in [('flat', [f'on_flat_{i}' for i in (1, 2, 3)], [f'off_flat_{i}' for i in (1, 2, 3)]),
                          ('slope', ['on_slope', 'on_slope_2', 'on_slope_3'], ['off_slope']),
                          ('waves', ['on_waves'], ['off_waves']), ('rough', ['on_rough'], ['off_rough'])]:
        a, b = head_stats(off), head_stats(on)
        hrows.append([f'{title[kind]} <span class="sub">({len(on)} / {len(off)} прогона)</span>',
                      W(f'{a[0]:.0f}° / {a[1]:.0f}°'), P(f'{b[0]:.0f}° / {b[1]:.0f}°'),
                      W(f'{a[2]:.0f}–{a[3]:.0f}%'), P(f'{b[2]:.0f}–{b[3]:.0f}%')])
    head_tbl = table(['Покрытие', 'Увод на прямой без удержания (средн. / макс.)', 'С удержанием',
                      'Развороты без удержания', 'С удержанием'], hrows)

    n_perception = sum(1 for f in os.listdir(os.path.join(args.out, 'videos')) if f.startswith('perception_'))
    html = open(cm.TEMPLATE, encoding='utf-8').read()
    for k, v in blocks.items():
        html = html.replace(f'<!--V:{k}-->', v)
    for k, v in {'TESTS': tests_tbl, 'CAL': cal_tbl, 'SLOPE': slope_tbl, 'OFF': off_tbl, 'WAVES': waves_tbl,
                 'ROUGH': rough_tbl, 'LIM': lim_tbl, 'ARC': arc_tbl, 'NCASES': str(tests['cases']),
                 'NTESTS': str(tests.get('colcon_tests', '—')), 'HEADTBL': head_tbl,
                 'PERCEPTION_N': str(n_perception), 'VIDEODATA': '',
                 'VIDEONOTE': 'Ролики лежат рядом, в папке videos/. Откройте этот файл из клонированного '
                              'репозитория (или через GitHub Pages) в Chrome, Edge или Safari.'}.items():
        html = html.replace(f'<!--{k}-->', v)
    out = os.path.join(args.out, 'index.html')
    open(out, 'w', encoding='utf-8').write(html)
    print(f'{out}: {len(html) / 1e3:.0f} kB, {len(blocks)} videos')


if __name__ == '__main__':
    main()
