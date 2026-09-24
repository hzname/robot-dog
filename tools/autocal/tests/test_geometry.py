import itertools

import numpy as np
import pytest

from robotdog_autocal import geometry as geo
from robotdog_autocal.sim import GEOMETRY, _CAMERAS, _tilt


def camera_positions(view, q, tilt=4.0):
    """Marker positions in a tilted camera frame, from joint angles."""
    R, t = _CAMERAS[view]
    R = _tilt(R, tilt)
    pos = {}
    for leg, ids in geo.VIEWS[view]['legs'].items():
        hip, thigh, knee, foot = geo.leg_points(GEOMETRY, leg, *q[leg])
        pts = (thigh, knee, foot) if geo.VIEWS[view]['kind'] == 'side' else (hip, foot)
        for i, p in zip(ids, pts):
            pos[i] = R @ (p - t) * 1000.0  # mm, like the tracker
    return pos


@pytest.mark.parametrize('view', ['left', 'right'])
@pytest.mark.parametrize('tilt', [0.0, 6.0])
def test_side_views_recover_thigh_and_calf(view, tilt):
    # Exact even with a tilted camera: body y comes from the legs' planes.
    for th, ca in itertools.product((10.0, 30.0, 55.0), (-120.0, -90.0, -45.0)):
        q = {l: (0.0, th + i * 3, ca - i * 2) for i, l in enumerate(geo.LEGS)}
        got = geo.measure(view, camera_positions(view, q, tilt))
        for leg in geo.VIEWS[view]['legs']:
            i = geo.LEGS.index(leg)
            assert got[f'{leg}_thigh_joint'] == pytest.approx(th + i * 3, abs=1e-6)
            assert got[f'{leg}_calf_joint'] == pytest.approx(ca - i * 2, abs=1e-6)


@pytest.mark.parametrize('view', ['front', 'rear'])
@pytest.mark.parametrize('tilt,tol', [(0.0, 1e-6), (4.0, 0.5)])
def test_end_views_recover_hip(view, tilt, tol):
    # A tilted camera leaves a second-order error (~0.4 deg at 15 deg, 4 deg tilt).
    for hip in (-15.0, -4.0, 0.0, 7.0, 15.0):
        q = {l: (hip * (1 if i % 2 else -1), 30.0, -90.0) for i, l in enumerate(geo.LEGS)}
        legs = {l: (30.0, -90.0) for l in geo.LEGS}
        got = geo.measure(view, camera_positions(view, q, tilt), GEOMETRY, legs)
        for leg in geo.VIEWS[view]['legs']:
            assert got[f'{leg}_hip_joint'] == pytest.approx(q[leg][0], abs=tol)


def test_missing_reference_marker_gives_nothing():
    q = {l: (0.0, 30.0, -90.0) for l in geo.LEGS}
    pos = camera_positions('left', q)
    del pos[0]
    assert geo.measure('left', pos) == {}
