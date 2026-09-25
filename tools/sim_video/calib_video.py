"""Video of the camera auto-calibration on the virtual robot (tools/autocal demo).

  python3 tools/sim_video/calib_video.py out.mp4 [--seed 1]

The virtual robot has hidden calibration errors (offsets off by up to 9 deg,
one servo mounted reversed, different pulse scales). The video shows what the
calibration does: the virtual camera image with the detected markers, the
robot on its stand, the sweep of the current joint with the fitted line, and
the zero error of every joint before and after.
"""

import argparse
import math
import os
import sys

import cv2
import imageio_ffmpeg
import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'autocal'))
from robotdog_autocal import __main__ as cli  # noqa: E402
from robotdog_autocal import geometry as geo  # noqa: E402
from robotdog_autocal import procedure, sim, vision  # noqa: E402

BG, INK, INK2, MUTED, GRID = '#f7f8fa', '#12161b', '#48515e', '#8791a0', '#dde2e8'
C1, C2, C3 = '#2a78d6', '#eb6834', '#1baf7a'
GOOD, BAD, WARN = '#0ca30c', '#d03b3b', '#b07800'
PAIR = {'lf': C1, 'rr': C1, 'rf': C2, 'lr': C2}
VIEW_RU = {'front': 'спереди', 'rear': 'сзади', 'left': 'слева', 'right': 'справа'}
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'text.color': INK,
                     'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                     'xtick.color': MUTED, 'ytick.color': MUTED})


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def record(seed):
    """Run the demo calibration and capture every measurement."""
    true = sim.hidden_errors(seed)
    robot = sim.FakeRobot(true)
    events, ctx = [], {'pass': 1, 'joint': None, 'phase': 'sweep', 'points': {}}
    fits = {}  # joint -> list of FitResult per pass

    orig_measure = procedure.Calibrator.measure
    orig_sweep = procedure.Calibrator.sweep
    orig_verify = procedure.Calibrator.verify
    orig_cal_view = procedure.Calibrator.calibrate_view

    def measure(self, view):
        img = self.camera.render()
        self.camera.frames = lambda n, _img=img: [_img] * n  # the frame shown is the frame measured
        ang = orig_measure(self, view)
        del self.camera.frames
        det = self.tracker.detect(img)
        if ctx['phase'] == 'sweep' and ctx['joint']:
            j = ctx['joint']
            ctx['points'].setdefault(j, []).append((robot.status()['pulses'][j], ang[j]))
        events.append({'view': view, 'pass': ctx['pass'], 'phase': ctx['phase'], 'joint': ctx['joint'],
                       'delta': ctx.get('delta'), 'img': cv2.resize(img, (640, 360)),
                       'det': {i: c / 2.0 for i, (_, c) in det.items()},
                       'true': dict(robot.true_angles()), 'cmd': dict(self.pose),
                       'measured': dict(ang), 'fits': {j: list(v) for j, v in fits.items()},
                       'points': {j: list(p) for j, p in ctx['points'].items()}})
        return ang

    def sweep(self, view, joint, deltas=procedure.SWEEP):
        ctx['joint'], ctx['phase'] = joint, 'sweep'
        ctx['points'][joint] = []
        out = []
        base = self.base()
        c = self.cal[joint]
        for d in deltas:  # same as the original, with the delta exposed to the video
            ctx['delta'] = d
            target = min(max(base[joint] + d, c.min_deg + 1.0), c.max_deg - 1.0)
            pose = dict(base)
            pose[joint] = target
            self.command(pose)
            ang = self.measure(view)
            q = ang[joint]
            out.append((self.robot.status()['pulses'][joint], q))
        return [p for p, _ in out], [q for _, q in out]

    def calibrate_view(self, view):
        res = orig_cal_view(self, view)
        for j, r in res.items():
            fits.setdefault(j, []).append(r)
        ctx['joint'] = None
        return res

    def verify(self, view, deltas=(-15.0, 0.0, 15.0)):
        ctx['phase'], ctx['joint'], ctx['delta'] = 'verify', None, None
        out = orig_verify(self, view, deltas)
        ctx['phase'] = 'sweep'
        if view == procedure.ORDER[-1]:
            ctx['pass'] += 1
        return out

    procedure.Calibrator.measure = measure
    procedure.Calibrator.sweep = sweep
    procedure.Calibrator.verify = verify
    procedure.Calibrator.calibrate_view = calibrate_view
    try:
        cams = {}

        def cam_for_view(view):
            cams[view] = sim.VirtualCamera(robot, view, seed=seed)
            return cams[view]
        cli._calibrate(robot, cam_for_view, lambda c: vision.MarkerTracker(c.intr, 22.0),
                       procedure.ORDER, 2, True, False, 0.0, lambda *_: None)
    finally:
        procedure.Calibrator.measure = orig_measure
        procedure.Calibrator.sweep = orig_sweep
        procedure.Calibrator.verify = orig_verify
        procedure.Calibrator.calibrate_view = orig_cal_view
    before = {j: sim.default_cal(j) for j in sim.JOINTS}
    return events, true, before, fits, robot.geometry


def camera_panel(ev):
    img = cv2.cvtColor(ev['img'], cv2.COLOR_GRAY2RGB)
    for i, c in ev['det'].items():
        pts = c.astype(np.int32)
        cv2.polylines(img, [pts], True, (12, 163, 12), 2, cv2.LINE_AA)
        cx, cy = pts.mean(axis=0).astype(int)
        cv2.putText(img, str(i), (cx - 8, cy - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (208, 59, 59), 1, cv2.LINE_AA)
    # leg lines through the marker centres (side views: thigh - knee - foot)
    v = geo.VIEWS[ev['view']]
    for leg, ids in v['legs'].items():
        cs = [ev['det'][i].mean(axis=0) for i in ids if i in ev['det']]
        col = (42, 120, 214) if PAIR[leg] == C1 else (235, 104, 52)
        for a, b in zip(cs, cs[1:]):
            cv2.line(img, tuple(int(x) for x in a), tuple(int(x) for x in b), col, 2, cv2.LINE_AA)
    return img


def draw_robot(ax, q, geometry, view, t):
    ax.cla()
    ax.set_facecolor(BG)
    ax.set_axis_off()
    r = 0.18
    ax.set_xlim(-r, r)
    ax.set_ylim(-r, r)
    ax.set_zlim(-0.25, 0.05)
    ax.set_box_aspect((1, 1, 0.30 / (2 * r)))
    ax.view_init(elev=18, azim=-58 + 12 * math.sin(t / 6))
    bl, bw, bh = 0.23, 0.10, 0.05
    v = [np.array([sx * bl / 2, sy * bw / 2, sz * bh / 2]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    faces = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
    # stand
    ax.plot([0, 0], [0, 0], [-bh / 2, -0.25], color='#9aa6b8', lw=6, zorder=1)
    ax.plot([-0.08, 0.08], [0, 0], [-0.25, -0.25], color='#9aa6b8', lw=6, zorder=1)
    for leg in ('lf', 'lr'):
        pts = np.array(geo.leg_points(geometry, leg, q[f'{leg}_hip_joint'], q[f'{leg}_thigh_joint'],
                                      q[f'{leg}_calf_joint']))
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color=PAIR[leg], lw=4, zorder=2, solid_capstyle='round')
    ax.add_collection3d(Poly3DCollection([[v[i] for i in fc] for fc in faces], facecolor='#3b4452',
                                         edgecolor=INK, lw=0.8, alpha=0.95, zorder=3))
    for leg in ('rf', 'rr'):
        pts = np.array(geo.leg_points(geometry, leg, q[f'{leg}_hip_joint'], q[f'{leg}_thigh_joint'],
                                      q[f'{leg}_calf_joint']))
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color=PAIR[leg], lw=4, zorder=4, solid_capstyle='round')
    # camera position for this view
    R, pos = sim._CAMERAS[view]
    d = pos / np.linalg.norm(pos) * 0.17
    ax.scatter([d[0]], [d[1]], [d[2]], s=160, marker='s', c=INK, zorder=5)
    ax.plot([d[0], d[0] * 0.55], [d[1], d[1] * 0.55], [d[2], d[2] * 0.55], color=INK, lw=1.2, ls=':', zorder=5)
    ax.text(d[0], d[1], d[2] + 0.035, 'камера', fontsize=9, ha='center', zorder=6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--first', type=int, default=0, help='preview: start at measurement N')
    ap.add_argument('--count', type=int, default=0, help='preview: only N measurements')
    args = ap.parse_args()

    events, true, before, fits, geometry = record(args.seed)
    print(f'{len(events)} measurements', flush=True)
    joints = sim.JOINTS
    err_before = {j: wrap(true[j].offset_deg - before[j].offset_deg) for j in joints}

    W, H = 1280, 720
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor=BG)
    a_cam = fig.add_axes([0.02, 0.42, 0.46, 0.46])
    a_3d = fig.add_axes([0.02, -0.04, 0.46, 0.45], projection='3d', computed_zorder=False)
    a_fit = fig.add_axes([0.56, 0.47, 0.41, 0.28], facecolor=BG)
    writer = imageio_ffmpeg.write_frames(args.out, (W, H), fps=args.fps, quality=7, macro_block_size=8,
                                         output_params=['-profile:v', 'main', '-movflags', '+faststart'])
    writer.send(None)

    per_meas = 7  # frames per measurement: 4 moving + 3 holding
    t = 0.0
    prev_q = events[0]['true']
    last_k = len(events) if not args.count else args.first + args.count
    for k, ev in enumerate(events):
        if k < args.first or k >= last_k:
            prev_q = ev['true']
            continue
        cam = camera_panel(ev)
        for sub in range(per_meas):
            s = min(1.0, (sub + 1) / 4)
            q = {j: prev_q[j] + (ev['true'][j] - prev_q[j]) * s for j in joints}
            t += 1 / args.fps
            a_cam.cla()
            a_cam.imshow(cam)
            a_cam.set_axis_off()
            a_cam.set_title(f'Виртуальная камера {VIEW_RU[ev["view"]]}: найдено меток {len(ev["det"])}',
                            fontsize=10, loc='left', color=INK2)
            draw_robot(a_3d, q, geometry, ev['view'], t)

            # fit chart for the current (or last) joint
            j = ev['joint'] or (list(ev['points'])[-1] if ev['points'] else None)
            a_fit.cla()
            a_fit.set_facecolor(BG)
            if j:
                pts = ev['points'].get(j, [])
                if pts:
                    p, m = np.array(pts).T
                    a_fit.scatter(p, m, s=36, color=C1, zorder=3, label='измерено камерой')
                done = [f for f in ev['fits'].get(j, [])]
                if done and not ev['joint']:
                    r = done[-1]
                    c = procedure.ServoCal(**vars(before[j]))
                    c.direction, c.offset_deg, c.range_deg = r.direction, r.offset_deg, r.range_deg
                    xs = np.linspace(min(p) - 20, max(p) + 20, 20) if pts else np.linspace(1100, 1600, 20)
                    a_fit.plot(xs, [c.pulse_to_joint(x) for x in xs], color=C2, lw=2, label='подобранная модель')
                a_fit.set_xlabel('импульс сервы, мкс', fontsize=9)
                a_fit.set_ylabel('угол сустава, °', fontsize=9)
                a_fit.legend(loc='upper left', fontsize=8, frameon=False)
                a_fit.set_title(f'{j}: импульс → угол', fontsize=10, loc='left', fontweight='bold')
            a_fit.grid(color=GRID, lw=0.6)
            a_fit.tick_params(labelsize=8, length=0)
            for sp in a_fit.spines.values():
                sp.set_visible(False)

            for tx in list(fig.texts):
                tx.remove()
            fig.text(0.02, 0.955, 'Автокалибровка по камере · виртуальный робот', fontsize=17,
                     fontweight='bold')
            fig.text(0.02, 0.918, 'Скрытые ошибки: нули сдвинуты до ±9°, одна серва стоит наоборот, '
                     'разный масштаб импульса', fontsize=10, color=INK2)
            if ev['phase'] == 'verify':
                step = 'проверка после записи калибровки: ±15°'
            else:
                step = f'{ev["joint"]}, шаг {ev["delta"]:+.0f}°' if ev['joint'] else ''
            fig.text(0.56, 0.87, f'Проход {ev["pass"]}/2 · вид {VIEW_RU[ev["view"]]}', fontsize=13,
                     color=C1, fontweight='bold')
            fig.text(0.56, 0.835, step, fontsize=11, color=INK)
            fig.text(0.56, 0.795, f'измерение {k + 1} из {len(events)}', fontsize=9, color=MUTED)

            # table: zero error before -> after
            fig.text(0.56, 0.405, 'Ошибка нуля, ° (было → стало)', fontsize=10.5, fontweight='bold')
            for n, jn in enumerate(joints):
                col, row = n // 6, n % 6
                x, y = 0.56 + col * 0.21, 0.37 - row * 0.052
                fl = ev['fits'].get(jn)
                e0 = err_before[jn]
                rev = true[jn].direction != before[jn].direction
                txt = f'{jn.replace("_joint", ""):9s} {e0:+5.1f}'
                if fl:
                    e1 = wrap(fl[-1].offset_deg - true[jn].offset_deg)
                    ok_dir = fl[-1].direction == true[jn].direction
                    color = GOOD if abs(e1) < 2 and ok_dir else WARN
                    txt += f' → {e1:+4.1f}'
                    if rev:
                        txt += ' ↺ исправлено' if ok_dir else ''
                else:
                    color = INK2
                    if rev:
                        txt += '  серва наоборот'
                fig.text(x, y, txt, fontsize=9.5, family='DejaVu Sans Mono', color=color)
            fig.canvas.draw()
            writer.send(np.ascontiguousarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3]))
        prev_q = ev['true']
        if k % 40 == 0:
            print(f'{k}/{len(events)}', flush=True)
    last = np.ascontiguousarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3])
    for _ in range(3 * args.fps):
        writer.send(last)
    writer.close()


if __name__ == '__main__':
    main()
