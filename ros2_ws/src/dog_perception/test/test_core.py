import math

import numpy as np

from dog_perception import core

SENSORS = {
    'x_lidar': True, 'x_lidar_x': 0.10, 'x_lidar_y': 0.04, 'x_lidar_z': 0.062,
    'x_lidar_tilt_deg': 30.0, 'x_lidar_yaw_deg': 40.0,
    'tof': True, 'tof_names': ['fl', 'fr', 'fc', 'rc'], 'tof_x': [0.115, 0.115, 0.115, -0.115],
    'tof_y': [0.045, -0.045, 0.0, 0.0], 'tof_z': [0.0, 0.0, 0.012, 0.0],
    'tof_pitch_deg': [40.0, 40.0, 20.0, 40.0], 'tof_yaw_deg': [23.0, -23.0, 0.0, 180.0],
    'gs2': True, 'gs2_x': 0.115, 'gs2_y': 0.0, 'gs2_z': 0.0, 'gs2_pitch_deg': 40.0,
}
GEOM = {'hip_offset': 0.055, 'thigh': 0.105, 'calf': 0.105, 'hip_x': 0.09, 'hip_y': 0.06}
H = 0.150  # body centre (hip axes) above the floor = stand_height
FLOOR = (np.array([0.0, 0.0, 1.0]), -H)


def test_tof_beams_hit_where_designed():
    m = core.mounts_from_params(SENSORS)
    # FL: 40 deg down, 23 deg out, from 0.15 m: 233 mm to the floor, spot on the left foot line
    p, u = m['tof_fl'].p, m['tof_fl'].beam
    t = core.ray_to_plane(p, u, FLOOR)
    assert abs(t - 0.15 / math.sin(math.radians(40))) < 1e-9
    spot = p + t * u
    assert abs(spot[1] - 0.115) < 0.005 and abs(spot[0] - 0.28) < 0.01
    assert core.ray_to_plane(m['tof_rc'].p, m['tof_rc'].beam, FLOOR) < math.inf
    assert (m['tof_rc'].p + core.ray_to_plane(m['tof_rc'].p, m['tof_rc'].beam, FLOOR) * m['tof_rc'].beam)[0] < -0.2


def test_x_lidars_draw_an_x_on_the_floor():
    m = core.mounts_from_params(SENSORS)
    lines = {}
    for name in ('lidar_left', 'lidar_right'):
        a = np.linspace(-math.pi, math.pi, 721)
        # ideal scan of the floor: range to the floor along each beam
        R, p = m[name].R, m[name].p
        rng = [core.ray_to_plane(p, R @ np.array([math.cos(x), math.sin(x), 0.0]), FLOOR) for x in a]
        pts = core.scan_to_body(m[name], rng, a[0], a[1] - a[0])
        assert np.allclose(pts[:, 2], -H, atol=1e-9)
        lines[name] = pts
    # the right lidar dips to the left: its line is nearer on the left side
    r = lines['lidar_right']
    near_left = r[(np.abs(r[:, 1] - 0.115) < 0.01)][:, 0].min()
    near_right = r[(np.abs(r[:, 1] + 0.115) < 0.01)][:, 0].min()
    assert near_left < near_right
    # the two lines cross ahead on the centre line
    both = np.vstack([lines['lidar_left'], lines['lidar_right']])
    fit = core.robust_plane(both)
    (n, c), _, rms = fit
    assert rms < 1e-6 and abs(c + H) < 1e-6
    assert all(abs(v) < 1e-6 for v in core.roll_pitch_of_normal(n))


def test_plane_attitude_signs():
    th = math.radians(10)
    # body pitched nose down by 10 deg over a level floor: floor normal in the body frame
    n = core.rot_rpy(0, th, 0).T @ np.array([0, 0, 1.0])
    roll, pitch = core.roll_pitch_of_normal(n)
    assert abs(pitch - th) < 1e-9 and abs(roll) < 1e-9
    n = core.rot_rpy(th, 0, 0).T @ np.array([0, 0, 1.0])
    roll, pitch = core.roll_pitch_of_normal(n)
    assert abs(roll - th) < 1e-9


def test_robust_plane_ignores_a_stone():
    rng = np.random.default_rng(0)
    xy = rng.uniform(-0.5, 0.5, (400, 2))
    z = np.full(400, -H) + rng.normal(0, 0.003, 400)
    z[:30] += 0.04  # a stone
    fit = core.robust_plane(np.column_stack([xy, z]))
    (n, c), mask, rms = fit
    assert abs(c + H) < 0.002 and not mask[:30].any()


def test_feet_plane_from_stance_and_imu_carry():
    # standing pose: feet straight under the thigh axes at 0.15 m
    L2 = GEOM['thigh']
    calf = -math.acos((0.15 ** 2 - 2 * L2 ** 2) / (2 * L2 ** 2))
    thigh = math.atan2(-L2 * math.sin(calf), L2 + L2 * math.cos(calf))
    q = {f'{leg}_{j}': v for leg, _, _ in core.LEGS for j, v in
         (('hip_joint', 0.0), ('thigh_joint', thigh), ('calf_joint', calf))}
    feet = core.feet_body(GEOM, q)
    assert np.allclose(feet[:, 2], -0.150, atol=1e-6)
    fp = core.FeetPlane()
    assert fp.update_feet(feet, np.eye(3))
    # a swing foot 2 cm up: not a four-leg support phase, plane kept
    lifted = feet.copy()
    lifted[0, 2] += 0.02
    assert not fp.update_feet(lifted, np.eye(3))
    # body pitched 5 deg since: the plane is carried by the IMU rotation
    R = core.rot_rpy(0, math.radians(5), 0)
    n, c = fp.current(R)
    assert abs(core.roll_pitch_of_normal(n)[1] - math.radians(5)) < 1e-9


def test_lidar_hazards_and_elevation_map():
    pts = np.array([[0.5, 0.12, -H + 0.02]] * 3 + [[0.6, -0.12, -H - 0.03]] * 3 + [[0.5, 0.0, -H]] * 5)
    hz = core.lidar_hazards(pts, FLOOR)
    assert ('left', 'up') in [(h[0], h[1]) for h in hz]
    assert ('right', 'down') in [(h[0], h[1]) for h in hz]
    assert not [h for h in hz if h[0] == 'centre']
    em = core.ElevationMap(size=1.0, res=0.02)
    em.recenter([0.0, 0.0])
    em.insert(np.array([[0.101, 0.101, 0.05], [0.101, 0.101, 0.07]]))
    mean = em.mean()
    assert abs(np.nanmax(mean) - 0.06) < 1e-12 and abs(np.nanmax(em.max) - 0.07) < 1e-12
    em.recenter([0.3, 0.0])  # move 0.3 m: the cell moves with the world
    X, Y = em.centers()
    k = np.nanargmax(em.mean())
    assert abs(X.flat[k] - 0.11) < 0.011 and abs(Y.flat[k] - 0.11) < 0.011


def test_tof_detector_calibrates_and_debounces():
    m = core.mounts_from_params(SENSORS)['tof_fl']
    det = core.TofDetector(m, thr=0.015, baseline_n=5)
    exp = core.ray_to_plane(m.p, m.beam, FLOOR)
    for _ in range(5):
        det.calibrate(exp + 0.004, FLOOR)  # sensor reads 4 mm long
    assert abs(det.offset - 0.004) < 1e-9
    assert det.check(exp + 0.004, FLOOR)[0] is None
    assert det.check(exp - 0.03, FLOOR)[0] is None     # first reading: not yet
    assert det.check(exp - 0.03, FLOOR)[0] == 'up'     # second: stone
    assert det.check(math.inf, FLOOR)[0] is None
    assert det.check(math.inf, FLOOR)[0] == 'down'     # no floor: edge or hole


def _gs2_scan(mount, floor_z, rmax=0.30, n=160, fov=100.0):
    """Ideal GS2 scan of a floor given as z(x, y) (ray march)."""
    a = np.linspace(-math.radians(fov / 2), math.radians(fov / 2), n)
    rng = []
    for x in a:
        u = mount.R @ np.array([math.cos(x), math.sin(x), 0.0])
        r = math.inf
        for t in np.arange(0.02, rmax, 0.0005):
            q = mount.p + t * u
            if q[2] <= floor_z(q[0], q[1]):
                r = t
                break
        rng.append(r)
    return core.scan_to_body(mount, rng, a[0], a[1] - a[0], 0.025, rmax)


def test_gs2_line_and_hazards():
    m = core.mounts_from_params(SENSORS)['gs2']
    flat = _gs2_scan(m, lambda x, y: -H)
    # the line lies across the floor 0.29 m ahead, both foot lines covered
    assert np.allclose(flat[:, 2], -H, atol=1e-3)
    assert abs(np.median(flat[:, 0]) - 0.294) < 0.01
    assert abs(core.gs2_line_x(SENSORS, H) - 0.2938) < 0.001
    assert flat[:, 1].min() < -0.17 and flat[:, 1].max() > 0.17
    assert core.gs2_hazards(flat, FLOOR, True) == []
    # a 20 mm stone on the left foot line: seen by the line fit, reference-free
    stone = _gs2_scan(m, lambda x, y: -H + (0.02 if 0.09 < y < 0.15 and x < 0.35 else 0.0))
    hz = core.gs2_hazards(stone, None, False)
    assert [(c, k, how) for c, k, _, _, _, how in hz] == [('left', 'up', 'line')]
    # a 20 mm step up across the path: only the reference plane sees it
    step = _gs2_scan(m, lambda x, y: -H + (0.02 if x > 0.25 else 0.0))
    kinds = {(c, k, how) for c, k, _, _, _, how in core.gs2_hazards(step, FLOOR, True)}
    assert kinds == {(c, 'up', 'plane') for c in core.CORRIDORS}
    # a 50 mm step down: the centre of the line is out of range
    down = _gs2_scan(m, lambda x, y: -H - (0.05 if x > 0.25 else 0.0))
    assert ('centre', 'down') in {(c, k) for c, k, *_ in core.gs2_hazards(down, FLOOR, True)}


def test_lidar_edge_height_tells_wall_from_ramp():
    x = np.linspace(0.25, 1.0, 300)
    ramp = np.c_[x, np.full_like(x, 0.12), -H + np.maximum(0.0, x - 0.5) * math.tan(math.radians(10))]
    wall = np.c_[x, np.full_like(x, 0.12), -H + np.where(x > 0.5, 0.10, 0.0)]
    (_, _, x0, h, jump), = core.lidar_hazards(ramp, FLOOR)
    assert 0.55 < x0 < 0.6 and h > 0.04 and jump < 0.015  # high above the plane, but no edge
    (_, _, x0, _, jump), = core.lidar_hazards(wall, FLOOR)
    assert abs(x0 - 0.5) < 0.01 and jump > 0.09


def test_hazard_guard():
    g = core.HazardGuard()
    assert g.command(0.0, (0.0, 0.0), 0.0)[2] == 'clear'
    assert g.verdict('up', 0.02) == 'step' and g.verdict('up', 0.10) == 'stop'
    assert g.verdict('down', -0.03) == 'step' and g.verdict('down', math.nan, deep=True) == 'step'
    assert core.HazardGuard(deep_stop=True).verdict('down', math.nan, deep=True) == 'stop'
    assert g.verdict('up', math.nan) == 'step' and g.verdict('down', math.nan) == 'step'
    assert g.verdict('up', 0.05) == 'crawl' and g.verdict('down', -0.05) == 'crawl'
    assert core.HazardGuard(crawl=False).verdict('up', 0.05) == 'stop'

    def add(t, xy, verdict, lift, n):
        for k in range(n):
            g.add(t, (xy[0] + 0.01 * k, xy[1]), verdict, lift)

    # one stray report does nothing
    add(0.0, (0.8, 0.115), 'step', 0.02, 1)
    assert g.command(0.0, (0.0, 0.0), 0.0)[2] == 'clear'
    # a confirmed stone on the left foot line 0.8 m ahead: caution, then only the left legs lift
    add(0.0, (0.8, 0.115), 'step', 0.02, 2)
    vx, step, state, _ = g.command(0.0, (0.0, 0.0), 0.0)
    assert state == 'caution' and vx == g.slow_vx and all(math.isnan(h) for h in step)
    vx, step, state, _ = g.command(1.0, (0.6, 0.0), 0.0)  # 0.2 m ahead: left front foot
    assert state == 'step_over' and vx == g.near_vx
    assert abs(step[0] - 0.03) < 1e-9 and all(math.isnan(h) for h in step[1:])
    vx, step, state, _ = g.command(2.0, (0.72, 0.0), 0.0)  # front foot nearly over, rear not yet
    assert math.isfinite(step[0]) and math.isnan(step[2])
    vx, step, state, _ = g.command(3.0, (0.95, 0.0), 0.0)  # left rear foot
    assert state == 'step_over' and abs(step[2] - 0.03) < 1e-9 and math.isnan(step[0]) and math.isnan(step[3])
    assert g.command(4.0, (1.1, 0.0), 0.0)[2] == 'clear'
    # unknown height (ToF, GS2 plane): slow down, no high swing
    add(4.0, (1.5, 0.0), 'step', 0.0, 3)
    vx, step, state, _ = g.command(4.0, (1.3, 0.0), 0.0)
    assert state == 'caution' and all(math.isnan(h) for h in step)
    # a wall: stop 0.3 m before it after 3 reports; turning away clears the path
    add(5.0, (2.0, 0.0), 'stop', 0.1, 2)
    assert g.command(5.0, (1.75, 0.0), 0.0)[2] != 'stop'
    add(5.0, (2.0, 0.03), 'stop', 0.1, 1)
    vx, _, state, d = g.command(5.0, (1.75, 0.0), 0.0)
    assert (vx, state) == (0.0, 'stop') and abs(d - 0.25) < 0.01  # cell mean
    assert g.command(5.5, (1.75, 0.0), math.radians(90))[2] == 'clear'
    # and it is forgotten after `memory` seconds
    assert g.command(5.0 + g.memory + 1, (1.75, 0.0), 0.0)[2] == 'clear'
    # right at the wall the lidars call it 'step' (no edge in view): the stop holds
    g = core.HazardGuard()
    add(0.0, (2.0, 0.0), 'stop', 0.1, 3)
    for k in range(2000):  # 60 reports/s for half a minute
        g.add(0.01 * k, (2.0, 0.0), 'step', 0.0)
    assert g.command(20.0, (1.75, 0.0), 0.0)[2] == 'stop'
    assert len(g.cells) <= 2  # memory grows with the area, not with the reports


def test_hazard_guard_crawl_window_matches_the_cpp_guard():
    # same as Core.GuardChoosesTheCrawlForStepsAndBars in test_core.cpp
    g = core.HazardGuard()
    for k in range(3):
        g.add(0, (0.6 + 0.01 * k, 0.12), 'crawl', 0.05)
    assert g.command(0, (0.0, 0.0), 0.0)[2] != 'crawl'   # 0.6 m ahead: not yet
    vx, _, state, _ = g.command(1, (0.2, 0.0), 0.0)       # 0.4 m: the crawl
    assert state == 'crawl' and math.isinf(vx)
    assert g.command(2, (0.85, 0.0), 0.0)[2] == 'crawl'  # under the body
    assert g.command(3, (0.95, 0.0), 0.0)[2] != 'crawl'  # rear feet past it
    s = core.HazardGuard()
    for _ in range(3):
        s.add(0, (0.8, 0.0), 'crawl', 0.05)
    assert s.command(0, (0.5, 0.0), 0.0)[2] == 'crawl'
    for _ in range(3):
        s.add(1, (1.1, 0.0), 'step', 0.02)
    assert s.command(2, (1.0, 0.0), 0.0)[2] == 'crawl'   # the next riser keeps it
    assert s.command(3, (1.45, 0.0), 0.0)[2] != 'crawl'
    assert s.command(4, (0.8, 0.0), 0.0)[2] != 'crawl'   # a 'step' alone never starts it
