"""Servo pulse <-> joint angle model, identical to dog_hardware/servo_driver.cpp.

Kept in sync by tests/test_servo_model.py, which checks reference values
produced by the C++ implementation.
"""

import math
from dataclasses import dataclass, field, fields


def _wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


@dataclass
class Linkage:
    servo_arm_mm: float = 0.0
    joint_arm_mm: float = 0.0
    rod_mm: float = 0.0
    axis_distance_mm: float = 0.0

    @property
    def direct(self):
        return self.servo_arm_mm <= 0.0

    def _abcd(self):
        a = self.servo_arm_mm
        b = self.joint_arm_mm if self.joint_arm_mm > 0 else a
        c = self.rod_mm if self.rod_mm > 0 else self.axis_distance_mm
        return a, b, c, self.axis_distance_mm

    def _lever(self, theta, branch):
        a, b, c, d = self._abcd()
        ax, ay = a * math.cos(theta), a * math.sin(theta)
        dx, dy = d - ax, -ay
        r = math.hypot(dx, dy)
        if r < 1e-9:
            return math.nan
        ratio = (c * c - r * r - b * b) / (2.0 * b) / r
        if ratio < -1.0 or ratio > 1.0:
            return math.nan
        return math.atan2(dy, dx) + branch * math.acos(ratio)

    def _branch(self):
        t = math.pi / 2
        p, m = self._lever(t, 1), self._lever(t, -1)
        if math.isnan(m):
            return 1
        if math.isnan(p):
            return -1
        dp = abs(_wrap_deg(math.degrees(p - t)))
        dm = abs(_wrap_deg(math.degrees(m - t)))
        return 1 if dp <= dm else -1

    def joint_delta(self, servo_deg):
        """Joint rotation [deg] for a servo rotation [deg] from its centre."""
        if self.direct:
            return servo_deg
        br = self._branch()
        p0 = self._lever(math.pi / 2, br)
        p = self._lever(math.pi / 2 + math.radians(servo_deg), br)
        if math.isnan(p0) or math.isnan(p):
            return math.nan
        return _wrap_deg(math.degrees(p - p0))

    def usable_servo_range(self, half):
        if self.direct:
            return -half, half
        step = 0.5
        g0 = self.joint_delta(0.0)
        gp = self.joint_delta(step)
        if math.isnan(g0) or math.isnan(gp) or gp == g0:
            return 0.0, 0.0
        sense = 1 if gp > g0 else -1

        def walk(direction):
            s, prev = 0.0, g0
            while abs(s) + step <= half + 1e-9:
                nxt = s + direction * step
                g = self.joint_delta(nxt)
                if math.isnan(g) or (g - prev) * sense * direction <= 0.0:
                    break
                s, prev = nxt, g
            return s

        return walk(-1), walk(1)


@dataclass
class ServoCal:
    channel: int = 0
    direction: int = 1
    offset_deg: float = 0.0
    pulse_min_us: float = 520.0
    pulse_max_us: float = 2220.0
    range_deg: float = 180.0
    min_deg: float = -180.0
    max_deg: float = 180.0
    linkage: Linkage = field(default_factory=Linkage)
    coupled_to: str = ''
    coupling: float = 0.0

    @property
    def center_us(self):
        return 0.5 * (self.pulse_min_us + self.pulse_max_us)

    @property
    def us_per_deg(self):
        return (self.pulse_max_us - self.pulse_min_us) / self.range_deg

    @classmethod
    def from_params(cls, p: dict):
        """From the flat parameter dict the robot reports (cal_info)."""
        link = Linkage(**{f.name: float(p.get(f.name) or 0.0) for f in fields(Linkage)})
        return cls(channel=int(p.get('channel', 0)), direction=int(p.get('direction', 1)),
                   offset_deg=float(p.get('offset_deg', 0.0)),
                   pulse_min_us=float(p.get('pulse_min_us', 520.0)),
                   pulse_max_us=float(p.get('pulse_max_us', 2220.0)),
                   range_deg=float(p.get('range_deg', 180.0)),
                   min_deg=float(p.get('min_deg', -180.0)), max_deg=float(p.get('max_deg', 180.0)),
                   linkage=link, coupled_to=str(p.get('coupled_to') or ''),
                   coupling=float(p.get('coupling') or 0.0))

    def servo_for_delta(self, delta):
        half = 0.5 * self.range_deg
        if self.linkage.direct:
            return max(-half, min(half, delta))
        lo, hi = self.linkage.usable_servo_range(half)
        glo, ghi = self.linkage.joint_delta(lo), self.linkage.joint_delta(hi)
        inc = ghi > glo
        if (inc and delta <= glo) or (not inc and delta >= glo):
            return lo
        if (inc and delta >= ghi) or (not inc and delta <= ghi):
            return hi
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if (self.linkage.joint_delta(mid) < delta) == inc:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def joint_to_pulse(self, joint_deg, parent_deg=0.0):
        q = max(self.min_deg, min(self.max_deg, joint_deg))
        parent = parent_deg if self.coupled_to else 0.0
        delta = self.direction * (q + self.coupling * parent - self.offset_deg)
        return self.center_us + self.servo_for_delta(delta) * self.us_per_deg

    def pulse_to_joint(self, us, parent_deg=0.0):
        s = (us - self.center_us) / self.us_per_deg
        delta = self.linkage.joint_delta(s)
        parent = parent_deg if self.coupled_to else 0.0
        return self.direction * delta + self.offset_deg - self.coupling * parent
