"""Render a recorded simulation run (walk_check --trace ... --record, or
terrain_sweep --record-dir) into an MP4: the robot on its terrain in 3D, the
maneuver checklist with the verdicts, body tilt, IMU attitude and foot contacts.

  python3 tools/sim_video/render.py rec/on/slope_10.json out.mp4 --title "Уклон 10°, IMU вкл."

Needs numpy, matplotlib, imageio-ffmpeg and ros2_ws/src/dog_gazebo on the path
(for the terrain geometry); ROS itself is not needed.
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
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'ros2_ws', 'src', 'dog_gazebo'))
from dog_gazebo import terrain  # noqa: E402

L1, L2, L3, HX, HY = 0.055, 0.105, 0.105, 0.09, 0.06
BODY = (0.23, 0.10, 0.05)
LEGS = [('lf', 1, 1), ('rf', 1, -1), ('lr', -1, 1), ('rr', -1, -1)]
BG, INK, INK2, MUTED, GRID = '#f7f8fa', '#12161b', '#48515e', '#8791a0', '#dde2e8'
C1, C2, C3 = '#2a78d6', '#eb6834', '#1baf7a'
GOOD, BAD = '#0ca30c', '#d03b3b'
PAIR = {'lf': C1, 'rr': C1, 'rf': C2, 'lr': C2}
NAMES = {
    'stand': 'Встаёт', 'forward': 'Вперёд', 'backward': 'Назад', 'left': 'Шаг влево',
    'right': 'Шаг вправо', 'turn_ccw': 'Разворот ↺', 'turn_cw': 'Разворот ↻', 'lie': 'Ложится'}
SLOPE_NAMES = {'forward': 'Подъём', 'backward': 'Спуск (задним ходом)',
               'left': 'Влево поперёк склона', 'right': 'Вправо поперёк склона'}
WIN = 4.0  # seconds shown in the charts

plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'text.color': INK,
                     'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                     'xtick.color': MUTED, 'ytick.color': MUTED})


def rotq(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def leg_points(f, leg, fr, sd):
    R, p = rotq(f['q']), np.array([f['x'], f['y'], f['z']])
    q0, q1, q2 = (f['j'].get(f'{leg}_{n}_joint', 0.0) for n in ('hip', 'thigh', 'calf'))
    hip = np.array([fr * HX, sd * HY, 0.0])
    A = Rx(q0)
    th = hip + A @ np.array([0, sd * L1, 0])
    B = A @ Ry(q1)
    kn = th + B @ np.array([0, 0, -L2])
    ft = kn + B @ Ry(q2) @ np.array([0, 0, -L3])
    return np.array([p + R @ v for v in (hip, th, kn, ft)])


def body_faces(f):
    R, p = rotq(f['q']), np.array([f['x'], f['y'], f['z']])
    bl, bw, bh = BODY
    v = [p + R @ np.array([sx * bl / 2, sy * bw / 2, sz * bh / 2])
         for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    faces = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
    return [[v[i] for i in fc] for fc in faces]


def stone_faces(o):
    sx, sy, sz = o['size']
    c, s = math.cos(o['yaw']), math.sin(o['yaw'])
    pts = []
    for zz in (0.0, sz):
        for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            du, dv = u * sx / 2, v * sy / 2
            pts.append((o['x'] + c * du - s * dv, o['y'] + s * du + c * dv, zz))
    idx = [[4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    return [[pts[i] for i in fc] for fc in idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('recording')
    ap.add_argument('out')
    ap.add_argument('--title', default='')
    ap.add_argument('--subtitle', default='')
    ap.add_argument('--size', default='1280x720')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--start', type=float, default=0.0, help='[s] (for previews)')
    ap.add_argument('--end', type=float, default=1e9, help='[s] (for previews)')
    args = ap.parse_args()

    rec = json.load(open(args.recording))
    kind, level = rec['terrain'], rec['level']
    F = [f for f in rec['trace'] if 'q' in f and f.get('j')]
    t0 = F[0]['t']
    for f in F:
        f['t'] -= t0
    obs = terrain.obstacles(kind, level)
    stones = [o for o in obs if o['shape'] == 'box']
    hfield = [o for o in obs if o['shape'] != 'box']
    results = {r['name']: r for r in rec['results']}

    # maneuver order (as checked) and end times: a verdict shows once its phase is over
    order = [r['name'] for r in rec['results']]
    ends = {}
    for a, b in zip(F, F[1:] + [None]):
        if b is None or b['phase'] != a['phase']:
            ends[a['phase']] = a['t']
    for name in order:  # maneuvers skipped after a fall: shown at the end
        ends.setdefault(name, F[-1]['t'])

    ts = np.array([f['t'] for f in F])
    tilt = []
    for f in F:
        up = rotq(f['q'])[:, 2]
        n = np.array(terrain.surface(kind, level, f['x'])[1])
        tilt.append(math.degrees(math.acos(max(-1.0, min(1.0, float(up @ n))))))
    tilt = np.array(tilt)
    imu = np.array([f.get('imu', [np.nan, np.nan]) for f in F])
    feet = {leg: np.array([leg_points(f, leg, fr, sd)[3] for f in F]) for leg, fr, sd in LEGS}
    air = {leg: np.array([p[2] - terrain.height(kind, level, p[0], p[1], obs) > 0.015 for p in feet[leg]])
           for leg, _, _ in LEGS}
    trail = np.array([[f['x'], f['y']] for f in F])
    # heading for the camera, smoothed (a fallen robot spins: keep the last upright heading)
    yaw = np.array([f['yaw'] for f in F], float)
    for k in range(1, len(yaw)):
        if tilt[k] > 60:
            yaw[k] = yaw[k - 1]
    yaw_s = np.convolve(np.pad(yaw, 45, mode='edge'), np.ones(91) / 91, mode='valid')

    W, H = (int(v) for v in args.size.split('x'))
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor=BG)
    ax = fig.add_axes([-0.04, -0.06, 0.66, 0.98], projection='3d', facecolor=BG, computed_zorder=False)
    a_tilt = fig.add_axes([0.62, 0.43, 0.35, 0.13], facecolor=BG)
    a_imu = fig.add_axes([0.62, 0.23, 0.35, 0.13], facecolor=BG)
    a_gait = fig.add_axes([0.62, 0.06, 0.35, 0.1], facecolor=BG)
    writer = imageio_ffmpeg.write_frames(args.out, (W, H), fps=args.fps, quality=7, macro_block_size=8,
                                         output_params=['-profile:v', 'main',
                                                        '-movflags', '+faststart'])
    writer.send(None)
    r = 0.26
    # the recording is sampled in wall time; resample to a steady frame rate
    t_out = np.arange(max(0.0, args.start), min(ts[-1], args.end), 1.0 / args.fps)
    for n_out, i in enumerate(np.clip(np.searchsorted(ts, t_out), 0, len(F) - 1)):
        f = F[i]
        ax.cla()
        ax.set_facecolor(BG)
        ax.set_axis_off()
        cx, cy = f['x'], f['y']
        cz = terrain.surface(kind, level, cx)[0]
        ax.set_xlim(cx - r, cx + r)
        ax.set_ylim(cy - r, cy + r)
        ax.set_zlim(cz - 0.03, cz + 0.26)
        ax.set_box_aspect((1, 1, 0.29 / (2 * r)))
        # the camera follows the robot's heading: always from the front-right quarter
        yaw_cam = yaw_s[i] if yaw_s is not None else 0.0
        ax.view_init(elev=24, azim=yaw_cam - 55 + 10 * math.sin(f['t'] / 10))
        # ground: grid lines following the surface (ramp, waves)
        xs = np.arange(math.floor((cx - r) / 0.04) * 0.04, cx + r, 0.04)
        ys = np.arange(math.floor((cy - r) / 0.04) * 0.04, cy + r, 0.04)
        fine_x = np.linspace(cx - r, cx + r, 90)
        fine_y = np.linspace(cy - r, cy + r, 90)
        for gy in ys:
            ax.plot(fine_x, np.full_like(fine_x, gy),
                    [terrain.height(kind, level, x, gy, hfield) for x in fine_x], color=GRID, lw=0.8, zorder=1)
        for gx in xs:
            ax.plot(np.full_like(fine_y, gx), fine_y,
                    [terrain.height(kind, level, gx, y, hfield) for y in fine_y], color=GRID, lw=0.8, zorder=1)
        if hfield:  # waves: shade the crests
            for o in hfield:
                if o['shape'] == 'cyl_y' and abs(o['x'] - cx) < r:
                    ax.plot([o['x']] * 2, [cy - r, cy + r], [o['z'] + o['r']] * 2, color='#9aa6b8', lw=2.2, zorder=2)
                elif o['shape'] == 'cyl_x' and abs(o['y'] - cy) < r and abs(o['x'] - cx) < r:
                    ax.plot([o['x'] - o['length'] / 2, o['x'] + o['length'] / 2], [o['y']] * 2,
                            [o['z'] + o['r']] * 2, color='#9aa6b8', lw=2.2, zorder=2)
        near = [o for o in stones if abs(o['x'] - cx) < r - 0.03 and abs(o['y'] - cy) < r - 0.03]
        if near:
            ax.add_collection3d(Poly3DCollection([fc for o in near for fc in stone_faces(o)],
                                                 facecolor='#b3aa9c', edgecolor='#7d7468', lw=0.5, zorder=2))
        tr = trail[:i + 1]
        m = (np.abs(tr[:, 0] - cx) < r) & (np.abs(tr[:, 1] - cy) < r)
        ax.plot(tr[m, 0], tr[m, 1], [terrain.surface(kind, level, x)[0] + 0.002 for x in tr[m, 0]],
                color=MUTED, lw=1.4, ls=(0, (2, 2)), zorder=3)
        # robot
        # the camera looks from the right side: left legs behind the body
        for leg, fr, sd in LEGS:
            if sd > 0:
                pts = leg_points(f, leg, fr, sd)
                ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color=PAIR[leg], lw=4, zorder=4, solid_capstyle='round')
        ax.add_collection3d(Poly3DCollection(body_faces(f), facecolor='#3b4452', edgecolor=INK,
                                             lw=0.8, alpha=0.95, zorder=5))
        for leg, fr, sd in LEGS:
            pts = leg_points(f, leg, fr, sd)
            if sd < 0:
                ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color=PAIR[leg], lw=4, zorder=6, solid_capstyle='round')
            down = not air[leg][i]
            ax.scatter([pts[3, 0]], [pts[3, 1]], [pts[3, 2]], s=60, c=PAIR[leg] if down else BG,
                       edgecolors=PAIR[leg], linewidths=2, depthshade=False, zorder=7 if sd < 0 else 4)

        lo = f['t'] - WIN
        sel = (ts >= lo) & (ts <= f['t'])
        # tilt chart
        a_tilt.cla()
        a_tilt.set_facecolor(BG)
        a_tilt.axhspan(20, 90, color='#fbe3e3', lw=0)
        a_tilt.plot(ts[sel], tilt[sel], color=C1, lw=2)
        a_tilt.set_xlim(lo, f['t'] + 0.05)
        a_tilt.set_ylim(0, max(30, float(np.nanmax(tilt[sel])) + 3) if sel.any() else 30)
        a_tilt.text(0, 1.08, 'Наклон корпуса к поверхности, °', transform=a_tilt.transAxes, fontweight='bold')
        a_tilt.text(1, 1.08, f'{tilt[i]:5.1f}°', transform=a_tilt.transAxes, ha='right', family='DejaVu Sans Mono')
        a_tilt.text(0.99, 0.93, 'предел 20°', transform=a_tilt.transAxes, ha='right', va='top', color=BAD, fontsize=8)
        # IMU chart
        a_imu.cla()
        a_imu.set_facecolor(BG)
        a_imu.plot(ts[sel], imu[sel, 1], color=C2, lw=2, label='тангаж')
        a_imu.plot(ts[sel], imu[sel, 0], color=C3, lw=1.6, label='крен')
        a_imu.set_xlim(lo, f['t'] + 0.05)
        a_imu.set_ylim(-25, 25)
        a_imu.text(0, 1.08, 'IMU: тангаж / крен относительно горизонта, °', transform=a_imu.transAxes,
                   fontweight='bold')
        a_imu.legend(loc='lower left', fontsize=8, frameon=False, ncol=2)
        # contacts
        a_gait.cla()
        a_gait.set_facecolor(BG)
        for row, (leg, _, _) in enumerate(LEGS):
            a_gait.fill_between(ts[sel], row - 0.35, row + 0.35, where=air[leg][sel], color=PAIR[leg],
                                step='mid', lw=0)
            a_gait.plot([lo, f['t']], [row, row], color=GRID, lw=1, zorder=0)
        a_gait.set_xlim(lo, f['t'] + 0.05)
        a_gait.set_ylim(3.6, -0.6)
        a_gait.set_yticks(range(4))
        a_gait.set_yticklabels(['LF', 'RF', 'LR', 'RR'], fontsize=8)
        a_gait.text(0, 1.12, 'Нога в воздухе', transform=a_gait.transAxes, fontweight='bold')
        for a in (a_tilt, a_imu, a_gait):
            a.grid(color=GRID, lw=0.6)
            a.tick_params(labelsize=8, length=0)
            for s in a.spines.values():
                s.set_visible(False)
        a_gait.set_xlabel('время, с', fontsize=8)

        for t in list(fig.texts):
            t.remove()
        names = dict(NAMES, **(SLOPE_NAMES if kind == 'slope' else {}))
        fig.text(0.02, 0.945, args.title, fontsize=17, fontweight='bold', color=INK)
        fig.text(0.02, 0.905, args.subtitle, fontsize=10.5, color=INK2)
        cur = names.get(f['phase'], '')
        fig.text(0.02, 0.855, cur, fontsize=14, color=C1, fontweight='bold')
        fig.text(0.56, 0.855, f't = {f["t"]:5.1f} с', fontsize=11, color=MUTED, ha='right',
                 family='DejaVu Sans Mono')
        # checklist
        fig.text(0.62, 0.945, 'Манёвры', fontsize=11, fontweight='bold')
        for k, ph in enumerate(order):
            col, row = k % 2, k // 2
            x, y = 0.62 + col * 0.18, 0.905 - row * 0.042
            done = f['t'] >= ends.get(ph, 1e9) - 0.05 and ph in results
            if done:
                res = results[ph]
                mark, color = ('✔', GOOD) if res['ok'] else ('✘', BAD)
                extra = ''
                if 'ratio' in res:
                    extra = f' {100 * res["ratio"]:.0f}%'
                    if not ph.startswith('turn') and 'dyaw_deg' in res:
                        extra += f' · курс {res["dyaw_deg"]:+.0f}°'
                if res.get('fallen'):
                    extra = ' пропущен'
                elif res.get('tilt_deg', 0) > 60:
                    extra = ' упал'
            else:
                mark, color, extra = ('▶', C1, '') if ph == f['phase'] else ('·', MUTED, '')
            fig.text(x, y, f'{mark} {names.get(ph, ph)}{extra}', fontsize=9.5, color=color)
        fig.canvas.draw()
        writer.send(np.ascontiguousarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3]))
        if n_out % 300 == 0:
            print(f'{args.out}: {n_out}/{len(t_out)}', flush=True)
    # hold the last frame for 2 s
    last = np.ascontiguousarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3])
    for _ in range(2 * args.fps):
        writer.send(last)
    writer.close()


if __name__ == '__main__':
    main()
