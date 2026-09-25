import math

import numpy as np

from dog_perception import core

SENSORS = {
    'x_lidar': True, 'x_lidar_x': 0.10, 'x_lidar_y': 0.04, 'x_lidar_z': 0.05,
    'x_lidar_tilt_deg': 30.0, 'x_lidar_yaw_deg': 40.0,
    'tof': True, 'tof_names': ['fl', 'fr', 'fc', 'rc'], 'tof_x': [0.115, 0.115, 0.115, -0.115],
    'tof_y': [0.045, -0.045, 0.0, 0.0], 'tof_z': [-0.012, -0.012, 0.0, -0.012],
    'tof_pitch_deg': [40.0, 40.0, 20.0, 40.0], 'tof_yaw_deg': [23.0, -23.0, 0.0, 180.0],
}
GEOM = {'hip_offset': 0.055, 'thigh': 0.105, 'calf': 0.105, 'hip_x': 0.09, 'hip_y': 0.06}
H = 0.162  # body centre above the floor
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
    assert np.allclose(feet[:, 2], -0.162, atol=1e-6)
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
