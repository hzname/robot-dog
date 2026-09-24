"""Calibration procedure: sweep each joint, measure it with the camera, fit.

Works with the real robot (client.RobotClient + vision.Camera) and with the
virtual one (sim.FakeRobot + sim.VirtualCamera) - both expose the same calls.
"""

import time

from . import geometry as geo
from .fit import FitResult, angle_error, fit_joint, find_limit
from .servo_model import ServoCal

BASE_POSE = {'hip': 0.0, 'thigh': 30.0, 'calf': -90.0}  # legs hanging, knees at a right angle
SWEEP = (-20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0)
# Hips first: an uncalibrated hip tilts the leg plane and shifts the thigh-axis
# markers the side views use as the body's x axis. A second pass refines.
ORDER = ('front', 'rear', 'left', 'right')


class MissingMarkers(RuntimeError):
    pass


class Calibrator:
    def __init__(self, robot, camera, tracker, settle_s=0.8, frames=6, log=print):
        self.robot, self.camera, self.tracker = robot, camera, tracker
        self.settle_s, self.frames, self.log = settle_s, frames, log
        info = robot.info()
        self.joints = info['joints']
        self.geometry = info.get('geometry') or {}
        self.cal = {j: ServoCal.from_params(p) for j, p in info['calibration'].items()}
        self.pose = {j: BASE_POSE[j.split('_')[1]] for j in self.joints}

    # ------------------------------------------------------------ primitives
    def command(self, pose):
        self.pose = dict(pose)
        self.robot.pose(self.pose)
        if hasattr(self.robot, 'settle'):
            self.robot.settle()
        if self.settle_s:
            time.sleep(self.settle_s)

    def measure(self, view):
        pos = self.tracker.positions(self.camera.frames(self.frames))
        ids = [i for ids in geo.VIEWS[view]['legs'].values() for i in ids]
        missing = [i for i in ids if i not in pos]
        if missing:
            names = ', '.join(f'{i} ({geo.MARKER_NAMES[i]})' for i in missing)
            raise MissingMarkers(f'{view}: camera does not see markers {names}')
        legs = {l: (self.pose[f'{l}_thigh_joint'], self.pose[f'{l}_calf_joint']) for l in geo.LEGS}
        return geo.measure(view, pos, self.geometry, legs)

    def joints_in_view(self, view):
        v = geo.VIEWS[view]
        kinds = ('thigh', 'calf') if v['kind'] == 'side' else ('hip',)
        return [f'{leg}_{k}_joint' for leg in v['legs'] for k in kinds]

    def base(self):
        return {j: BASE_POSE[j.split('_')[1]] for j in self.joints}

    # ------------------------------------------------------------ calibration
    def sweep(self, view, joint, deltas=SWEEP):
        c = self.cal[joint]
        base = self.base()
        pulses, measured = [], []
        for d in deltas:
            target = min(max(base[joint] + d, c.min_deg + 1.0), c.max_deg - 1.0)
            pose = dict(base)
            pose[joint] = target
            self.command(pose)
            ang = self.measure(view)
            q = ang[joint]
            if c.coupled_to:
                q += c.coupling * ang.get(c.coupled_to, self.pose[c.coupled_to])
            pulses.append(self.robot.status()['pulses'][joint])
            measured.append(q)
        return pulses, measured

    def calibrate_view(self, view):
        results = {}
        for joint in self.joints_in_view(view):
            self.log(f'  {joint}: sweep ±{max(SWEEP):.0f}°')
            pulses, measured = self.sweep(view, joint)
            res = fit_joint(joint, self.cal[joint], pulses, measured)
            results[joint] = res
            self.log(f'  {joint}: direction {self.cal[joint].direction:+d} -> {res.direction:+d}, '
                     f'offset {self.cal[joint].offset_deg:.1f} -> {res.offset_deg:.1f}°, '
                     f'fit error {res.rms_deg:.2f}°')
        self.command(self.base())
        return results

    def apply(self, results):
        params = {}
        for r in results.values():
            params.update(r.params())
        ok, reason = self.robot.set_params(params)
        if not ok:
            raise RuntimeError(f'robot refused the calibration: {reason}')
        for r in results.values():
            c = self.cal[r.joint]
            c.direction, c.offset_deg = r.direction, r.offset_deg
            if not r.linkage:
                c.range_deg = r.range_deg
        self.command(self.base())

    def verify(self, view, deltas=(-15.0, 0.0, 15.0)):
        """Max |commanded - measured| per joint after calibration [deg]."""
        errors = {j: 0.0 for j in self.joints_in_view(view)}
        for d in deltas:
            pose = self.base()
            for j in errors:
                c = self.cal[j]
                pose[j] = min(max(pose[j] + d, c.min_deg + 1.0), c.max_deg - 1.0)
            self.command(pose)
            ang = self.measure(view)
            for j in errors:
                errors[j] = max(errors[j], angle_error(ang[j], pose[j]))
        self.command(self.base())
        return errors

    def find_limits(self, view, joint, step_deg=3.0, margin_deg=3.0, max_steps=15, stall_amps=0.8):
        """Walk a calibrated joint outwards until it stops following (mechanical
        stop or collision) or the servo current jumps; returns new (min, max)."""
        c = self.cal[joint]
        original = (c.min_deg, c.max_deg)
        half = 0.5 * c.range_deg
        servo_lo, servo_hi = c.offset_deg - half, c.offset_deg + half
        wide = {f'{joint}.min_deg': round(min(servo_lo, servo_hi), 1),
                f'{joint}.max_deg': round(max(servo_lo, servo_hi), 1)}
        self.robot.set_params(wide)
        c.min_deg, c.max_deg = wide[f'{joint}.min_deg'], wide[f'{joint}.max_deg']
        found = []
        try:
            for sign, start in ((-1, original[0]), (1, original[1])):
                base = self.base()
                commanded, measured, amps = [], [], []
                for k in range(max_steps + 1):
                    target = start + sign * k * step_deg
                    if not (c.min_deg <= target <= c.max_deg):
                        break
                    pose = dict(base)
                    pose[joint] = target
                    self.command(pose)
                    commanded.append(target)
                    measured.append(self.measure(view)[joint])
                    power = self.robot.status().get('power')
                    amps.append(power['current'] if power else None)
                    if len(measured) > 1 and angle_error(measured[-1] - measured[0], target - start) > 3.0:
                        break
                    if amps[0] is not None and amps[-1] is not None and amps[-1] - amps[0] > stall_amps:
                        break
                last = find_limit(commanded, measured, 1)
                if amps and amps[0] is not None:
                    for i, a in enumerate(amps):
                        if a is not None and a - amps[0] > stall_amps:
                            last = min(last, i - 1)
                            break
                last = max(last, 0)
                found.append(commanded[last] - sign * margin_deg)
                self.command(self.base())
        finally:
            new = (min(found[0], original[1] - 5.0), max(found[1], original[0] + 5.0)) if len(found) == 2 \
                else original
            self.robot.set_params({f'{joint}.min_deg': round(new[0], 1), f'{joint}.max_deg': round(new[1], 1)})
            c.min_deg, c.max_deg = new
            self.command(self.base())
        return new


def yaml_line(joint, c: ServoCal):
    """One line in the servos.yaml format."""
    parts = [f'channel: {c.channel}', f'direction: {c.direction:2d}', f'offset_deg: {c.offset_deg:6.1f}',
             f'pulse_min_us: {c.pulse_min_us:.1f}', f'pulse_max_us: {c.pulse_max_us:.1f}',
             f'range_deg: {c.range_deg:.1f}', f'min_deg: {c.min_deg:.1f}', f'max_deg: {c.max_deg:.1f}',
             f'servo_arm_mm: {c.linkage.servo_arm_mm:.1f}']
    if not c.linkage.direct:
        parts += [f'joint_arm_mm: {c.linkage.joint_arm_mm:.1f}', f'rod_mm: {c.linkage.rod_mm:.1f}',
                  f'axis_distance_mm: {c.linkage.axis_distance_mm:.1f}']
    if c.coupled_to:
        parts += [f'coupled_to: {c.coupled_to}', f'coupling: {c.coupling:.2f}']
    return f'    {joint + ":":16s}{{' + ', '.join(parts) + '}'


def update_servos_yaml(path, cal: dict):
    """Rewrite the joint lines of servos.yaml in place; everything else is kept."""
    with open(path) as fh:
        lines = fh.read().split('\n')
    done = set()
    for i, line in enumerate(lines):
        stripped = line.strip()
        for j, c in cal.items():
            if stripped.startswith(j + ':') and '{' in stripped:
                lines[i] = yaml_line(j, c)
                done.add(j)
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines))
    return sorted(done)


__all__ = ['Calibrator', 'MissingMarkers', 'FitResult', 'yaml_line', 'update_servos_yaml']
