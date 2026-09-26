"""Video of a localization run (dog_gazebo localization_check --trace): the
room from above with the map's walls, the true path, the robot's own
estimate (localization/pose) and dead reckoning, and their errors over time.

  python3 tools/sim_video/localization_video.py run.json map out.mp4 --title "..." [--speed 2]

`map` is the map path without extension (<map>.walls and <map>.truth.json:
the map frame's place in the mapping run's world, written by the check).
Everything is drawn in the mapping run's world frame.
"""

import argparse
import json
import math
import os
import sys

import imageio_ffmpeg
import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

HERE = os.path.dirname(__file__)
sys.path.insert(0, HERE)
import render as rv  # noqa: E402
from dog_gazebo import terrain  # noqa: E402  (path added by render)

TRUE, LOC, DR, WALL, FURN = '#1baf7a', '#2a78d6', '#d03b3b', '#12161b', '#c8b8a6'
STATUS_RU = {'tracking': 'следит по карте', 'relocalizing': 'ищет себя в карте', 'lost': 'потерялся'}
PHASE_RU = {'stand': 'встаёт', 'survey': 'осмотр', 'relocalize': 'привязка', 'route': 'маршрут', 'end': 'стоп'}


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), -a[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('recording')
    ap.add_argument('map')
    ap.add_argument('out')
    ap.add_argument('--title', default='')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--speed', type=float, default=2.0)
    args = ap.parse_args()

    rec = json.load(open(args.recording))
    truth_cfg = json.load(open(args.map + '.truth.json'))
    off = tuple(truth_cfg['map_to_world'])  # map frame -> mapping run's world
    walls = np.loadtxt(args.map + '.walls').reshape(-1, 2)
    c, s = math.cos(off[2]), math.sin(off[2])
    walls = walls @ np.array([[c, s], [-s, c]]) + np.array(off[:2])
    world = truth_cfg.get('world', 'room')
    boxes = terrain.obstacles(world, 0, truth_cfg.get('seed', 0))
    # submap origins and loop closures of the map (house maps have a graph)
    graph_nodes, graph_loops = [], []
    if os.path.exists(args.map + '.graph'):
        lines = open(args.map + '.graph').read().split('\n')
        n = int(lines[0].split()[1])
        graph_nodes = [compose(off, tuple(float(v) for v in ln.split()[1:4])) for ln in lines[1:1 + n]]
        for ln in lines[2 + n:]:
            f = ln.split()
            if len(f) == 8 and f[7] == '1':
                graph_loops.append((int(f[0]), int(f[1])))
    F = [e for e in rec['trace'] if e['phase'] != 'start']
    ts = np.array([e['t'] for e in F])
    t0 = ts[0]
    truth = np.array([e['truth'] for e in F])
    loc = np.array([compose(off, e['loc']) if 'loc' in e else (np.nan,) * 3 for e in F])
    dr_first = next((e for e in rec['trace'] if 'dr' in e), None)  # as localization_check scores it
    dr_off = compose(tuple(dr_first['truth']), inverse(tuple(dr_first['dr'])))  # starts where the robot stood
    dr = np.array([compose(dr_off, e['dr']) if 'dr' in e else (np.nan,) * 3 for e in F])
    e_loc = np.hypot(*(loc[:, :2] - truth[:, :2]).T)
    e_dr = np.hypot(*(dr[:, :2] - truth[:, :2]).T)
    sc = rec['scores']

    W, H = 1280, 720
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor=rv.BG)
    ax = fig.add_axes([0.02, 0.04, 0.58, 0.86], facecolor=rv.BG)
    ae = fig.add_axes([0.66, 0.10, 0.31, 0.30], facecolor=rv.BG)
    fig.text(0.02, 0.955, args.title, fontsize=17, color=rv.INK, weight='bold')
    info = fig.text(0.66, 0.86, '', fontsize=12.5, color=rv.INK, va='top', linespacing=1.6)
    for b in boxes:
        sx, sy = b['size'][:2]
        cb, sb = math.cos(b['yaw']), math.sin(b['yaw'])
        pts = [(b['x'] + cb * u - sb * v, b['y'] + sb * u + cb * v)
               for u, v in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]
        wall = b['name'].startswith('wall')
        ax.add_patch(Polygon(pts, closed=True, fc='#e4e0da' if wall else FURN,
                             ec='none', alpha=0.9 if wall else 0.55, zorder=1))
    ax.scatter(walls[:, 0], walls[:, 1], s=3, c=WALL, lw=0, zorder=2, label='стены карты')
    if graph_nodes:
        gn = np.array(graph_nodes)
        ax.plot(gn[:, 0], gn[:, 1], 'o', ms=5, mfc='none', mec='#8a5cd0', mew=1.2, zorder=6, label='подкарты')
        for i, j in graph_loops:
            ax.plot([gn[i, 0], gn[j, 0]], [gn[i, 1], gn[j, 1]], color='#8a5cd0', lw=1.4, ls=':', zorder=6)
    xs = [b['x'] for b in boxes]
    ys = [b['y'] for b in boxes]
    ax.set_xlim(min(xs) - 0.4, max(xs) + 0.4)
    ax.set_ylim(min(ys) - 0.4, max(ys) + 0.4)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)
    l_true, = ax.plot([], [], color=TRUE, lw=2.4, zorder=3, label='истина')
    l_dr, = ax.plot([], [], color=DR, lw=1.8, ls='--', zorder=3, label='счисление')
    l_loc, = ax.plot([], [], color=LOC, lw=1.8, zorder=4, label='локализация')
    robot = Polygon(np.zeros((3, 2)), closed=True, fc=TRUE, ec=rv.INK, lw=0.8, zorder=5)
    ghost = Polygon(np.zeros((3, 2)), closed=True, fc=LOC, ec='none', alpha=0.8, zorder=6)
    ax.add_patch(robot)
    ax.add_patch(ghost)
    ax.legend(loc='upper left', bbox_to_anchor=(0.0, 1.06), ncol=5, frameon=False, fontsize=11)

    ae.set_xlim(0, ts[-1] - t0)
    ae.set_ylim(0, max(0.3, float(np.nanmax(e_dr)) * 1.1 if np.isfinite(e_dr).any() else 0.3))
    ae.set_title('ошибка положения, м', fontsize=11, color=rv.INK2, loc='left')
    ae.set_xlabel('с', fontsize=10, color=rv.MUTED)
    ae.grid(color=rv.GRID, lw=0.6)
    for sp in ae.spines.values():
        sp.set_color(rv.GRID)
    ae.tick_params(colors=rv.MUTED, labelsize=9)
    le_loc, = ae.plot([], [], color=LOC, lw=1.6)
    le_dr, = ae.plot([], [], color=DR, lw=1.4, ls='--')
    cursor = ae.axvline(0, color=rv.MUTED, lw=0.8)

    def tri(p, size=0.16):
        cp, sp_ = math.cos(p[2]), math.sin(p[2])
        pts = ((size, 0), (-0.6 * size, 0.5 * size), (-0.6 * size, -0.5 * size))
        return [(p[0] + cp * u - sp_ * v, p[1] + sp_ * u + cp * v) for u, v in pts]

    writer = imageio_ffmpeg.write_frames(args.out, (W, H), fps=args.fps, quality=7, macro_block_size=8,
                                         output_params=['-profile:v', 'main', '-movflags', '+faststart'])
    writer.send(None)
    lo = sc.get('localization') or {}
    for t in np.arange(ts[0], ts[-1], args.speed / args.fps):
        k = int(np.searchsorted(ts, t, side='right')) - 1
        e = F[k]
        l_true.set_data(truth[:k + 1, 0], truth[:k + 1, 1])
        l_dr.set_data(dr[:k + 1, 0], dr[:k + 1, 1])
        l_loc.set_data(loc[:k + 1, 0], loc[:k + 1, 1])
        robot.set_xy(tri(truth[k]))
        ghost.set_visible(bool(np.isfinite(loc[k, 0])))
        if np.isfinite(loc[k, 0]):
            ghost.set_xy(tri(loc[k], 0.09))
        le_loc.set_data(ts[:k + 1] - t0, e_loc[:k + 1])
        le_dr.set_data(ts[:k + 1] - t0, e_dr[:k + 1])
        cursor.set_xdata([t - t0, t - t0])
        st = e.get('status')
        lines = [f"t = {t - t0:5.1f} с   ×{args.speed:g}",
                 f"этап: {PHASE_RU.get(e['phase'], e['phase'])}",
                 f"карта: {STATUS_RU.get(st, st or '—')}"
                 + (f" · {100 * e['inliers']:.0f} % точек на стенах" if st == 'tracking' and e.get('inliers') else ''),
                 f"ошибка: локализация {e_loc[k] * 100:.1f} см" if np.isfinite(e_loc[k]) else 'ошибка: локализация —',
                 f"            счисление {e_dr[k] * 100:.0f} см" if np.isfinite(e_dr[k]) else '',
                 '',
                 f"итог по маршруту: p95 {100 * lo.get('p95_m', float('nan')):.1f} см, "
                 f"курс p95 {lo.get('yaw_p95_deg', float('nan')):.1f}°"]
        if sc.get('relocalized_s') is not None and sc['phase'] == 'localize':
            lines.append(f"нашёл себя через {sc['relocalized_s']:.1f} с после осмотра")
        info.set_text('\n'.join(lines))
        fig.canvas.draw()
        writer.send(np.ascontiguousarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3]))
    last = np.ascontiguousarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3])
    for _ in range(2 * args.fps):
        writer.send(last)
    writer.close()
    print('wrote', args.out)


if __name__ == '__main__':
    main()
