"""robotdog-autocal: camera auto-calibration of the RobotDog 2.0 servos.

  python -m robotdog_autocal markers  --marker-mm 25            # print this sheet
  python -m robotdog_autocal camera-calib --camera 0            # optional, better accuracy
  python -m robotdog_autocal preview  --camera 0 --view left    # check marker placement
  python -m robotdog_autocal run      --robot 192.168.1.50 --camera 0 --view all --apply
  python -m robotdog_autocal demo                                # virtual robot, no hardware
"""

import argparse
import sys
import time

from . import geometry as geo
from . import markers, procedure, sim, vision


def _tracker(args, size):
    intr = vision.Intrinsics.load(args.intrinsics) if args.intrinsics \
        else vision.Intrinsics.approximate(*size, args.hfov)
    return vision.MarkerTracker(intr, args.marker_mm)


def cmd_markers(args):
    img = markers.sheet(args.marker_mm)
    vision.cv2.imwrite(args.out, img)
    print(f'wrote {args.out} (A4, 300 dpi). Print at 100 % scale and check the square is '
          f'{args.marker_mm:g} mm.\nWhere each marker goes:\n{markers.views_help()}')


def cmd_camera_calib(args):
    cv2 = vision.cv2
    cam = vision.Camera(args.camera)
    shots = []
    print('Show a printed chessboard (9x6 inner corners) from different angles.\n'
          'SPACE = take photo, ENTER = compute (need >= 10), ESC = cancel')
    while True:
        img = cam.frames(1)[0]
        view = img.copy()
        cv2.putText(view, f'photos: {len(shots)}', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.imshow('camera calibration', view)
        k = cv2.waitKey(1) & 0xFF
        if k == 32:
            shots.append(img)
        elif k == 13 and len(shots) >= 10:
            break
        elif k == 27:
            return 1
    intr, rms = vision.calibrate_from_chessboard(shots, square_mm=args.square_mm)
    intr.save(args.out, rms)
    print(f'wrote {args.out}, reprojection error {rms:.2f} px')
    return 0


def cmd_preview(args):
    cv2 = vision.cv2
    cam = vision.Camera(args.camera)
    tracker = _tracker(args, cam.size())
    print('ESC to quit. Every marker of the view must be outlined in green.')
    while True:
        img = cam.frames(1)[0]
        found = tracker.detect(img)
        for i, (xyz, pts) in found.items():
            cv2.polylines(img, [pts.astype(int)], True, (0, 255, 0), 2)
            cv2.putText(img, f'{i}', tuple(pts[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        pos = {i: xyz for i, (xyz, _) in found.items()}
        angles = geo.measure(args.view, pos, sim.GEOMETRY)
        need = [i for ids in geo.VIEWS[args.view]['legs'].values() for i in ids]
        missing = [i for i in need if i not in pos]
        y = 30
        for text in ([f'missing: {missing}'] if missing else []) + [f'{j}: {a:+.1f} deg' for j, a in angles.items()]:
            cv2.putText(img, text, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if missing else (0, 160, 0), 2)
            y += 28
        cv2.imshow(f'preview {args.view}', img)
        if cv2.waitKey(1) & 0xFF == 27:
            return 0


def _print_results(results, cal_before):
    print(f'\n  {"joint":16s} {"direction":>10s} {"offset, deg":>18s} {"servo travel":>14s} {"fit":>7s}')
    for j, r in results.items():
        b = cal_before[j]
        print(f'  {j:16s} {b.direction:+3d} → {r.direction:+d}   {b.offset_deg:7.1f} → {r.offset_deg:7.1f}'
              f'   {b.range_deg:5.1f} → {r.range_deg:5.1f}  {r.rms_deg:5.2f}°')


def _calibrate(robot, cam_for_view, tracker_for_view, views, passes, apply, find_limits, settle, log):
    results_all, cal = {}, None
    for p in range(1, passes + 1):
        for view in views:
            cam = cam_for_view(view)
            if cam is None:
                continue
            cal = procedure.Calibrator(robot, cam, tracker_for_view(cam), settle_s=settle, log=log)
            before = {j: procedure.ServoCal(**vars(c)) for j, c in cal.cal.items()}
            log(f'\n== pass {p}/{passes}, view {view}')
            res = cal.calibrate_view(view)
            _print_results(res, before)
            if apply or passes > 1:
                cal.apply(res)
                errors = cal.verify(view)
                log('  check after applying: ' + ', '.join(f'{j} {e:.1f}°' for j, e in errors.items()))
            if find_limits and p == passes:
                for j in cal.joints_in_view(view):
                    lo, hi = cal.find_limits(view, j)
                    log(f'  {j}: free travel {lo:.1f} .. {hi:.1f}° (3° margin from the stop)')
            results_all.update(res)
    return results_all, cal


def cmd_run(args):
    from .client import RobotClient
    robot = RobotClient(args.robot, args.port)
    views = procedure.ORDER if args.view == 'all' else (args.view,)
    cam = vision.Camera(args.camera)
    print('Robot must be on a stand with the legs hanging free, locomotion passive.\n'
          'Ctrl-C at any time switches the servos off (E-STOP).')

    def cam_for_view(view):
        if args.view == 'all':
            ids = [i for ids in geo.VIEWS[view]['legs'].values() for i in ids]
            input(f'\nPoint the camera at the robot from the {view.upper()} (markers {ids}), '
                  f'whole legs in frame, then press Enter... ')
        return cam

    try:
        _, cal = _calibrate(robot, cam_for_view, lambda c: _tracker(args, c.size()), views,
                            args.passes, args.apply, args.find_limits, args.settle, print)
    except KeyboardInterrupt:
        robot.estop(True)
        print('\ninterrupted: servos switched off (release the E-STOP on the web page)')
        return 1
    except procedure.MissingMarkers as exc:
        print(f'\n{exc}\nFix the camera position (try `preview`) and run again.')
        return 2
    lines = [procedure.yaml_line(j, c) for j, c in cal.cal.items()]
    with open(args.out, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print(f'\nwrote {args.out} (lines for servos.yaml)')
    if args.servos_yaml:
        done = procedure.update_servos_yaml(args.servos_yaml, cal.cal)
        print(f'updated {len(done)} joints in {args.servos_yaml}')
    if not args.apply:
        print('dry run: nothing was changed on the robot (add --apply)')
    robot.close()
    return 0


def cmd_demo(args):
    true = sim.hidden_errors(args.seed)
    robot = sim.FakeRobot(true)
    cams = {}

    def cam_for_view(view):
        cams[view] = sim.VirtualCamera(robot, view, seed=args.seed)
        return cams[view]

    print('Virtual robot with hidden calibration errors; the virtual camera renders its markers.')
    t0 = time.time()
    res, cal = _calibrate(robot, cam_for_view, lambda c: vision.MarkerTracker(c.intr, 22.0),
                          procedure.ORDER, 2, True, False, 0.0, print)
    worst = 0.0
    print(f'\n  {"joint":16s} {"found offset":>13s} {"true offset":>12s} {"direction":>10s}')
    for j, r in res.items():
        t = true[j]
        worst = max(worst, abs(r.offset_deg - t.offset_deg))
        print(f'  {j:16s} {r.offset_deg:13.2f} {t.offset_deg:12.2f}   {r.direction:+d} / {t.direction:+d}')
    print(f'\nworst offset error {worst:.2f}°, {time.time() - t0:.1f} s')
    return 0 if worst < 2.0 else 1


def main(argv=None):
    ap = argparse.ArgumentParser(prog='robotdog_autocal', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--marker-mm', type=float, default=25.0, help='printed marker size (black square)')
    common.add_argument('--intrinsics', help='camera.json from camera-calib (else an estimate)')
    common.add_argument('--hfov', type=float, default=65.0, help='camera horizontal FOV if not calibrated')
    common.add_argument('--camera', default='0', help='camera index or URL')

    p = sub.add_parser('markers', parents=[common], help='make the printable marker sheet')
    p.add_argument('--out', default='robotdog_markers.png')
    p = sub.add_parser('camera-calib', parents=[common], help='calibrate the laptop camera (chessboard)')
    p.add_argument('--square-mm', type=float, default=25.0)
    p.add_argument('--out', default='camera.json')
    p = sub.add_parser('preview', parents=[common], help='live view of detected markers and angles')
    p.add_argument('--view', choices=list(geo.VIEWS), default='left')
    p = sub.add_parser('run', parents=[common], help='calibrate the robot')
    p.add_argument('--robot', required=True, help='robot address (its web page host)')
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--view', choices=list(geo.VIEWS) + ['all'], default='all')
    p.add_argument('--passes', type=int, default=2)
    p.add_argument('--apply', action='store_true', help='write the result to the robot (live)')
    p.add_argument('--find-limits', action='store_true', help='also find mechanical stops (slow)')
    p.add_argument('--settle', type=float, default=0.8, help='wait after each move [s]')
    p.add_argument('--out', default='servos_calibrated.yaml')
    p.add_argument('--servos-yaml', help='update this servos.yaml in place')
    p = sub.add_parser('demo', help='run on a virtual robot (no hardware)')
    p.add_argument('--seed', type=int, default=1)

    args = ap.parse_args(argv)
    return {'markers': cmd_markers, 'camera-calib': cmd_camera_calib, 'preview': cmd_preview,
            'run': cmd_run, 'demo': cmd_demo}[args.cmd](args) or 0


if __name__ == '__main__':
    sys.exit(main())
