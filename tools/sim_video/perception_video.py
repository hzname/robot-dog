"""Video of a perception run (dog_gazebo perception_check --trace): the robot
with the scan lines of the two crossed lidars and the VL53L1X beams on the
terrain, what was detected and when, the ToF residuals and the ground
attitude from the lidars and from the legs against the truth.

  python3 tools/sim_video/perception_video.py run.json out.mp4 --title "..."
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

HERE = os.path.dirname(__file__)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', 'ros2_ws', 'src', 'dog_perception'))
import render as rv  # noqa: E402
from dog_gazebo import terrain  # noqa: E402  (path added by render)
from dog_perception import core  # noqa: E402

UP, DOWN, FLOOR, BEAM, ALERT = '#eb6834', '#2a78d6', '#8f9aa8', '#1baf7a', '#d03b3b'
TOF = ('fl', 'fr', 'fc', 'rc')
TOF_RU = {'fl': 'FL', 'fr': 'FR', 'fc': 'FC', 'rc': 'RC'}
SENSORS_OLD = {  # mounts of recordings made before they were stored in the recording
    'x_lidar': True, 'x_lidar_x': 0.10, 'x_lidar_y': 0.04, 'x_lidar_z': 0.05,
    'x_lidar_tilt_deg': 30.0, 'x_lidar_yaw_deg': 40.0,
    'tof': True, 'tof_names': list(TOF), 'tof_x': [0.115, 0.115, 0.115, -0.115],
    'tof_y': [0.045, -0.045, 0.0, 0.0], 'tof_z': [-0.012, -0.012, 0.0, -0.012],
    'tof_pitch_deg': [40.0, 40.0, 20.0, 40.0], 'tof_yaw_deg': [23.0, -23.0, 0.0, 180.0],
    'gs2': True, 'gs2_x': 0.115, 'gs2_y': 0.0, 'gs2_z': -0.012, 'gs2_pitch_deg': 40.0,
}
GS2 = '#7a3fc4'
FEATURES_RU = {'stone_left': 'камень 20 мм, левая нога', 'stone_right': 'камень 20 мм, правая нога',
               'step_down': 'ступенька вниз', 'step_up': 'ступенька вверх'}


def nearest(ts, t):
    i = int(np.clip(np.searchsorted(ts, t), 0, len(ts) - 1))
    if i > 0 and t - ts[i - 1] < ts[i] - t:
        i -= 1
    return i


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('recording')
    ap.add_argument('out')
    ap.add_argument('--title', default='')
    ap.add_argument('--subtitle', default='')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--tof-threshold', default='0.02,0.02,0.05,0.02')
    ap.add_argument('--start', type=float, default=0.0)
    ap.add_argument('--end', type=float, default=1e9)
    ap.add_argument('--speed', type=float, default=1.0, help='play the run this many times faster (long crawls)')
    args = ap.parse_args()
    if args.speed != 1.0:
        args.subtitle = (args.subtitle + ' · ' if args.subtitle else '') + f'ускорено ×{args.speed:g}'
    thr = [float(v) for v in args.tof_threshold.split(',')]

    rec = json.load(open(args.recording))
    kind, level = rec['terrain'], rec['level']
    F = [f for f in rec['trace'] if f.get('j') and f['phase'] in ('stand', 'walk', 'stop')]
    t0 = next(f['t'] for f in F if f['phase'] == 'walk') - 1.0
    F = [f for f in F if f['t'] >= t0]
    ts = np.array([f['t'] for f in F])
    obs = terrain.obstacles(kind, level)
    stones = [o for o in obs if o['shape'] == 'box']
    hfield = [o for o in obs if o['shape'] != 'box']
    mounts = core.mounts_from_params(rec.get('sensors') or SENSORS_OLD)
    scans = {n: [s for s in rec['scans'] if s['name'] == n] for n in ('lidar_left', 'lidar_right', 'gs2')}
    has_gs2 = bool(scans['gs2'])
    fams = (('lidar', 'лидары'), ('tof', 'ToF')) + ((('gs2', 'GS2'),) if has_gs2 else ())
    scan_t = {n: np.array([s['t'] for s in v]) for n, v in scans.items()}
    tof = np.array([[np.nan if v is None else v for v in r] for r in rec['tof']], float) \
        if rec['tof'] else np.zeros((0, 5))
    ground = rec['ground']
    hazards = [h for h in rec['hazards'] if h['phase'] == 'walk']
    guard = rec.get('guard') or []  # [t, state, max_vx, [step per leg], phase] on every change
    guard_t = np.array([g[0] for g in guard]) if guard else np.zeros(0)
    scores = rec['scores']

    # first detection time of each known hazard per sensor family (steps world)
    firsts = {}
    if kind == 'steps':
        (xl, _), (xr, _) = terrain.STONES
        feats = [('stone_left', xl, 'up', {'left'}, {'tof_fl'}),
                 ('stone_right', xr, 'up', {'right'}, {'tof_fr'}),
                 ('step_down', terrain.STEP_DOWN_X, 'down', {'left', 'right', 'centre'}, {'tof_fl', 'tof_fr', 'tof_fc'}),
                 ('step_up', terrain.STEP_UP_X, 'up', {'left', 'right', 'centre'}, {'tof_fl', 'tof_fr', 'tof_fc'})]
        for name, edge, k, corr, tofs in feats:
            for fam, _ in fams:
                best = None
                for h in hazards:
                    if h['kind'] != k:
                        continue
                    ok = (h['source'] == fam and h.get('corridor') in corr) if fam in ('lidar', 'gs2') \
                        else h['source'] in tofs
                    ahead = edge - (h['robot'][0] + 0.15)
                    if ok and -0.05 < ahead < 1.2 and (best is None or h['t_rx'] < best[0]):
                        best = (h['t_rx'], ahead)
                firsts[(name, fam)] = best
        feat_order = [f[0] for f in feats]
    else:
        feat_order = []

    W, H = 1280, 720
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor=rv.BG)
    ax = fig.add_axes([-0.05, -0.06, 0.66, 0.98], projection='3d', facecolor=rv.BG, computed_zorder=False)
    a_tof = fig.add_axes([0.62, 0.40, 0.35, 0.14], facecolor=rv.BG)
    a_pitch = fig.add_axes([0.62, 0.20, 0.35, 0.12], facecolor=rv.BG)
    a_hz = fig.add_axes([0.62, 0.05, 0.35, 0.08], facecolor=rv.BG)
    writer = imageio_ffmpeg.write_frames(args.out, (W, H), fps=args.fps, quality=7, macro_block_size=8,
                                         output_params=['-profile:v', 'main', '-movflags', '+faststart'])
    writer.send(None)
    r = 0.42
    t_out = np.arange(max(ts[0], t0 + args.start), min(ts[-1], t0 + args.end), args.speed / args.fps)
    gt = np.array([g['t'] for g in ground]) if ground else np.zeros(0)
    for n_out, t in enumerate(t_out):
        f = F[nearest(ts, t)]
        R, p = rv.rotq(f['q']), np.array([f['x'], f['y'], f['z']])
        yaw = math.atan2(R[1, 0], R[0, 0])
        cx, cy = f['x'] + 0.22 * math.cos(yaw), f['y'] + 0.22 * math.sin(yaw)  # look ahead
        cz = terrain.surface(kind, level, cx)[0]
        ax.cla()
        ax.set_facecolor(rv.BG)
        ax.set_axis_off()
        ax.set_xlim(cx - r, cx + r)
        ax.set_ylim(cy - r, cy + r)
        ax.set_zlim(cz - 0.03, cz + 0.30)
        ax.set_box_aspect((1, 1, 0.33 / (2 * r)))
        ax.view_init(elev=30, azim=math.degrees(yaw) - 60)
        rv.draw_ground(ax, kind, level, hfield, stones, cx, cy, r)
        # lidar points of the latest scan of each lidar, coloured by height over the floor plane
        body_pts, world_pts = [], []
        for name in ('lidar_left', 'lidar_right'):
            if not len(scan_t[name]):
                continue
            k = int(np.searchsorted(scan_t[name], t)) - 1
            if k < 0 or t - scan_t[name][k] > 0.15:
                continue
            s = scans[name][k]
            rr = [math.inf if v is None else v for v in s['r']]
            bp = core.scan_to_body(mounts[name], rr, s['a0'], s['da'])
            fs = F[nearest(ts, s['t'])]
            Rs, ps = rv.rotq(fs['q']), np.array([fs['x'], fs['y'], fs['z']])
            body_pts.append(bp @ (R.T @ Rs).T)
            world_pts.append(bp @ Rs.T + ps)
        if body_pts:
            bp, wp = np.vstack(body_pts), np.vstack(world_pts)
            keep = ~((np.abs(bp[:, 0]) < 0.22) & (np.abs(bp[:, 1]) < 0.17))
            bp, wp = bp[keep], wp[keep]
            near = bp[(np.hypot(bp[:, 0], bp[:, 1]) < 1.2) & (bp[:, 2] < -0.05)]
            fit = core.robust_plane(near)
            res = bp @ fit[0][0] - fit[0][1] if fit else np.zeros(len(bp))
            vis = (np.abs(wp[:, 0] - cx) < r) & (np.abs(wp[:, 1] - cy) < r) & (res < 0.25)
            col = np.where(res > 0.02, UP, np.where(res < -0.02, DOWN, FLOOR))
            ax.scatter(wp[vis, 0], wp[vis, 1], wp[vis, 2] + 0.002, s=5, c=col[vis], depthshade=False, zorder=3)
        # GS2 line: the latest scan, coloured by the offset from its own straight-line fit
        if has_gs2:
            k = int(np.searchsorted(scan_t['gs2'], t)) - 1
            if k >= 0 and t - scan_t['gs2'][k] < 0.1:
                s = scans['gs2'][k]
                rr = [math.inf if v is None else v for v in s['r']]
                bp = core.scan_to_body(mounts['gs2'], rr, s['a0'], s['da'], 0.025, 0.30)
                fs = F[nearest(ts, s['t'])]
                Rs, ps = rv.rotq(fs['q']), np.array([fs['x'], fs['y'], fs['z']])
                a = Rs @ mounts['gs2'].p + ps
                if len(bp) > 5:
                    wp = bp @ Rs.T + ps
                    res = bp[:, 2] - np.median(bp[:, 2])
                    col = np.where(res > 0.012, UP, np.where(res < -0.012, DOWN, GS2))
                    for q in (wp[0], wp[-1]):
                        ax.plot([a[0], q[0]], [a[1], q[1]], [a[2], q[2]], color=GS2, lw=0.6, alpha=0.5, zorder=5)
                    ax.scatter(wp[:, 0], wp[:, 1], wp[:, 2] + 0.003, s=9, c=col, depthshade=False, zorder=6)
        # ToF beams
        for k, n in enumerate(TOF):
            sel = tof[(tof[:, 1] == k) & (tof[:, 0] <= t) & (tof[:, 0] > t - 0.1)] if len(tof) else []
            m = mounts[f'tof_{n}']
            a = R @ m.p + p
            u = R @ m.beam
            if len(sel):
                rng, res_k = sel[-1, 2], sel[-1, 4]
                bad = np.isfinite(res_k) and abs(res_k) > thr[k] or not np.isfinite(rng)
                L = rng if np.isfinite(rng) else 0.6
            else:
                bad, L = False, 0.0
            if L > 0:
                b = a + L * u
                ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], color=ALERT if bad else BEAM,
                        lw=2.2 if bad else 1.4, zorder=6, ls='-' if np.isfinite(rng) else ':')
                ax.scatter([b[0]], [b[1]], [b[2]], s=18, c=ALERT if bad else BEAM, depthshade=False, zorder=6)
        rv.draw_robot(ax, f, {leg: False for leg, _, _ in rv.LEGS})
        # hazard guard: state and the legs that swing higher
        guard_line = None
        if len(guard_t):
            gk = int(np.searchsorted(guard_t, t, side='right')) - 1
            g = guard[gk] if gk >= 0 else None
            if g:
                state, vmax, steps = g[1], g[2], g[3]
                col = {'stop': ALERT, 'step_over': UP, 'caution': '#c9a227', 'crawl': '#7b5cc4',
                       'avoid': '#2f8fb5'}.get(state, rv.GOOD)
                label = {'clear': 'путь свободен', 'caution': 'препятствие впереди — медленно',
                         'step_over': 'перешагивает', 'crawl': 'ступень: медленно, на трёх опорах',
                         'stop': 'СТОП: препятствие', 'avoid': 'обходит препятствие'}.get(state, state)
                legs = ', '.join(f'{n} {1e3 * h:.0f} мм' for n, h in zip(('ЛП', 'ПП', 'ЛЗ', 'ПЗ'), steps) if h)
                speed = '' if vmax is None else f' · до {vmax:.2f} м/с'
                guard_line = (f'Реакция: {label}{speed}' + (f' · высокий шаг: {legs}' if legs else ''), col)
                for (leg, fr, sd), h in zip(rv.LEGS, steps):
                    if h:  # a ring over the foot of a leg that steps high
                        ft = rv.leg_points(f, leg, fr, sd)[3]
                        ax.scatter([ft[0]], [ft[1]], [ft[2] + 0.02], s=140, facecolors='none', edgecolors=UP,
                                   linewidths=2.2, depthshade=False, zorder=8)

        # ToF residual chart
        lo = t - 4.0
        a_tof.cla()
        a_tof.set_facecolor(rv.BG)
        for k, (n, col) in enumerate(zip(('fl', 'fr', 'fc'), (rv.C1, rv.C2, rv.C3))):
            sel = tof[(tof[:, 1] == k) & (tof[:, 0] > lo) & (tof[:, 0] <= t)] if len(tof) else np.zeros((0, 5))
            a_tof.plot(sel[:, 0] - t0, 1e3 * sel[:, 4], color=col, lw=1.4, label=TOF_RU[n])
        a_tof.axhspan(1e3 * thr[0], 200, color='#fbe3e3', lw=0)
        a_tof.axhspan(-200, -1e3 * thr[0], color='#e3ecfb', lw=0)
        a_tof.set_ylim(-60, 60)
        a_tof.set_xlim(lo - t0, t - t0 + 0.05)
        a_tof.legend(loc='lower left', fontsize=8, frameon=False, ncol=3)
        a_tof.text(0, 1.08, f'ToF: измерено − ожидалось, мм (порог FL/FR ±{1e3 * thr[0]:.0f})', transform=a_tof.transAxes,
                   fontweight='bold')
        # attitude chart: pitch true vs lidar vs legs
        a_pitch.cla()
        a_pitch.set_facecolor(rv.BG)
        if len(gt):
            for src, col, lab in (('lidar', rv.C1, 'по лидарам'), ('feet', rv.C2, 'по ногам')):
                g = [x for x in ground if x['src'] == src and lo < x['t'] <= t]
                if g:
                    a_pitch.plot([x['t'] - t0 for x in g], [math.degrees(x['est'][1]) for x in g], color=col, lw=1.5,
                                 label=lab)
            g = [x for x in ground if x['src'] == 'lidar' and lo < x['t'] <= t]
            if g:
                a_pitch.plot([x['t'] - t0 for x in g], [math.degrees(x['true'][1]) for x in g], color=rv.INK, lw=1.0,
                             ls=(0, (2, 2)), label='истина')
        a_pitch.set_xlim(lo - t0, t - t0 + 0.05)
        a_pitch.set_ylim(-6, 6)
        a_pitch.legend(loc='lower left', fontsize=8, frameon=False, ncol=3)
        a_pitch.text(0, 1.1, 'Тангаж корпуса относительно пола, °', transform=a_pitch.transAxes, fontweight='bold')
        # hazard ticks
        a_hz.cla()
        a_hz.set_facecolor(rv.BG)
        rows = [('лидары ↑', 'lidar', 'up'), ('лидары ↓', 'lidar', 'down'), ('ToF ↑', 'tof', 'up'),
                ('ToF ↓', 'tof', 'down')] + ([('GS2 ↑', 'gs2', 'up'), ('GS2 ↓', 'gs2', 'down')] if has_gs2 else [])
        for k, (_, src, kd) in enumerate(rows):
            tt = [h['t_rx'] - t0 for h in hazards if h['source'].startswith(src) and h['kind'] == kd and lo < h['t_rx'] <= t]
            a_hz.scatter(tt, [k] * len(tt), s=10, c=UP if kd == 'up' else DOWN, marker='|')
        a_hz.set_yticks(range(len(rows)))
        a_hz.set_yticklabels([x[0] for x in rows], fontsize=6 if has_gs2 else 7)
        a_hz.set_ylim(len(rows) - 0.4, -0.6)
        a_hz.set_xlim(lo - t0, t - t0 + 0.05)
        a_hz.text(0, 1.15, 'Сообщения о препятствиях', transform=a_hz.transAxes, fontweight='bold')
        for a in (a_tof, a_pitch, a_hz):
            a.grid(color=rv.GRID, lw=0.6)
            a.tick_params(labelsize=7, length=0)
            for sp in a.spines.values():
                sp.set_visible(False)

        for tx in list(fig.texts):
            tx.remove()
        if guard_line:
            fig.text(0.02, 0.12, guard_line[0], fontsize=11, fontweight='bold', color=guard_line[1])
        fig.text(0.02, 0.945, args.title, fontsize=17, fontweight='bold')
        fig.text(0.02, 0.905, args.subtitle, fontsize=10.5, color=rv.INK2)
        fig.text(0.02, 0.07, '● пол   ', color=FLOOR, fontsize=10)
        fig.text(0.075, 0.07, '● выше пола', color=UP, fontsize=10)
        fig.text(0.16, 0.07, '● ниже пола', color=DOWN, fontsize=10)
        fig.text(0.245, 0.07, '— луч ToF (красный = срабатывание)', color=BEAM, fontsize=10)
        fig.text(0.02, 0.04, 'точки — два лидара «крестом» (α 30°, β 40°), лучи — VL53L1X FL, FR, FC, RC'
                 + (', фиолетовая линия — GS2' if has_gs2 else ''), fontsize=9, color=rv.MUTED)
        fig.text(0.56, 0.86, f't = {t - t0:5.1f} с', fontsize=11, color=rv.MUTED, ha='right', family='DejaVu Sans Mono')
        fig.text(0.62, 0.945, 'Что обнаружено' if feat_order else 'Итог прогона', fontsize=11, fontweight='bold')
        if feat_order:
            for k, name in enumerate(feat_order):
                y = 0.905 - k * 0.05
                parts = []
                for fam, lab in fams:
                    b = firsts.get((name, fam))
                    parts.append(f'{lab} {b[1]:.2f} м' if b and b[0] <= t else f'{lab} —')
                seen = any(firsts.get((name, fam)) and firsts[(name, fam)][0] <= t for fam, _ in fams)
                fig.text(0.62, y, ('✔ ' if seen else '· ') + FEATURES_RU[name], fontsize=9.5,
                         color=rv.GOOD if seen else rv.MUTED)
                fig.text(0.62, y - 0.022, '   до стопы: ' + ', '.join(parts), fontsize=8.5, color=rv.INK2)
        else:
            gl, gf = scores.get('ground_lidar', {}), scores.get('ground_feet', {})
            lines = [f"пол по лидарам: высота ±{gl.get('height_rms_mm', '—')} мм, наклон ±{gl.get('pitch_rms_deg', '—')}°",
                     f"пол по ногам: высота ±{gf.get('height_rms_mm', '—')} мм, наклон ±{gf.get('pitch_rms_deg', '—')}°",
                     f"карта высот: ±{scores.get('map', {}).get('rms_mm', '—')} мм",
                     f"сообщений до сих пор: лидары {sum(1 for h in hazards if h['source'] == 'lidar' and h['t_rx'] <= t)},"
                     f" ToF {sum(1 for h in hazards if h['source'].startswith('tof') and h['t_rx'] <= t)}"
                     + (f", GS2 {sum(1 for h in hazards if h['source'] == 'gs2' and h['t_rx'] <= t)}" if has_gs2 else '')]
            for k, ln in enumerate(lines):
                fig.text(0.62, 0.905 - k * 0.04, ln, fontsize=9.5, color=rv.INK2)
        fig.canvas.draw()
        writer.send(np.ascontiguousarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3]))
        if n_out % 300 == 0:
            print(f'{args.out}: {n_out}/{len(t_out)}', flush=True)
    last = np.ascontiguousarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3])
    for _ in range(2 * args.fps):
        writer.send(last)
    writer.close()


if __name__ == '__main__':
    main()
