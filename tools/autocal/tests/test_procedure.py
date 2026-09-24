"""End to end on the virtual robot: hidden calibration errors -> camera ->
fit -> applied calibration puts every joint where it is commanded."""

import copy

import pytest

from robotdog_autocal import procedure, sim, vision
from robotdog_autocal.servo_model import Linkage

pytest.importorskip('cv2')


def run(robot, views=procedure.ORDER, passes=2, frames=2):
    results = {}
    for _ in range(passes):
        for view in views:
            cam = sim.VirtualCamera(robot, view)
            cal = procedure.Calibrator(robot, cam, vision.MarkerTracker(cam.intr, 22.0),
                                       settle_s=0, frames=frames, log=lambda *a: None)
            res = cal.calibrate_view(view)
            cal.apply(res)
            results.update(res)
    return results, cal


@pytest.mark.parametrize('seed', [1, 2])
def test_full_calibration_recovers_hidden_errors(seed):
    true = sim.hidden_errors(seed)
    robot = sim.FakeRobot(true)
    results, cal = run(robot)
    for j, r in results.items():
        assert r.direction == true[j].direction, j
        assert abs(r.offset_deg - true[j].offset_deg) < 1.5, (j, r.offset_deg, true[j].offset_deg)
        assert r.rms_deg < 1.0, j
    # Commanded poses now land where they should.
    for view in procedure.ORDER:
        cam = sim.VirtualCamera(robot, view)
        c = procedure.Calibrator(robot, cam, vision.MarkerTracker(cam.intr, 22.0), settle_s=0,
                                 frames=2, log=lambda *a: None)
        for j, e in c.verify(view).items():
            assert e < 2.5, (j, e)


def test_rod_driven_and_body_coupled_knee():
    true = sim.hidden_errors(3)
    link = Linkage(servo_arm_mm=15, joint_arm_mm=20, rod_mm=95, axis_distance_mm=95)
    for cal in (true['lf_calf_joint'],):
        cal.linkage = copy.deepcopy(link)
        cal.coupled_to, cal.coupling = 'lf_thigh_joint', 1.0
        cal.offset_deg = -52.0  # knee + thigh at the servo centre
    believed = {j: sim.default_cal(j) for j in sim.JOINTS}
    b = believed['lf_calf_joint']
    b.linkage = copy.deepcopy(link)
    b.coupled_to, b.coupling, b.offset_deg = 'lf_thigh_joint', 1.0, -60.0
    robot = sim.FakeRobot(true, believed)
    results, _ = run(robot)
    r = results['lf_calf_joint']
    assert r.linkage
    assert r.offset_deg == pytest.approx(-52.0, abs=1.5)


def test_find_limits_stops_at_the_mechanical_stop():
    true = sim.hidden_errors(4)
    robot = sim.FakeRobot(true, stops={'lf_calf_joint': (-135.0, -35.0)})
    run(robot, views=('front', 'rear', 'left'), passes=1)
    cam = sim.VirtualCamera(robot, 'left')
    cal = procedure.Calibrator(robot, cam, vision.MarkerTracker(cam.intr, 22.0), settle_s=0,
                               frames=2, log=lambda *a: None)
    cal.cal['lf_calf_joint'].min_deg, cal.cal['lf_calf_joint'].max_deg = -110.0, -70.0
    robot.set_params({'lf_calf_joint.min_deg': -110.0, 'lf_calf_joint.max_deg': -70.0})
    lo, hi = cal.find_limits('left', 'lf_calf_joint')
    assert -135.0 < lo < -128.0, lo   # stop at -135, 3 deg margin, 3 deg steps
    assert -42.0 < hi < -35.0, hi
    assert robot.believed['lf_calf_joint'].min_deg == pytest.approx(lo)


def test_yaml_update(tmp_path):
    p = tmp_path / 'servos.yaml'
    p.write_text('x: 1\n    lf_hip_joint:   {channel: 0, direction:  1, offset_deg:   0.0}\n# end\n')
    cal = {'lf_hip_joint': sim.default_cal('lf_hip_joint')}
    cal['lf_hip_joint'].offset_deg = 3.4
    assert procedure.update_servos_yaml(str(p), cal) == ['lf_hip_joint']
    text = p.read_text()
    assert 'offset_deg:    3.4' in text and text.startswith('x: 1') and text.endswith('# end\n')
