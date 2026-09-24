"""Fitting servo calibration from (pulse, measured joint angle) samples."""

import math
from dataclasses import dataclass

import numpy as np

from .servo_model import ServoCal


@dataclass
class FitResult:
    joint: str
    direction: int
    offset_deg: float
    range_deg: float          # new servo travel for the same pulse_min/max (direct drive)
    rms_deg: float            # residual of the fit
    samples: int
    linkage: bool

    def params(self):
        p = {f'{self.joint}.direction': self.direction,
             f'{self.joint}.offset_deg': round(self.offset_deg, 2)}
        if not self.linkage:
            p[f'{self.joint}.range_deg'] = round(self.range_deg, 2)
        return p


def fit_joint(joint, cal: ServoCal, pulses, angles_eff):
    """Fit direction / offset (and servo scale for direct drive).

    angles_eff are measured joint angles plus coupling * measured parent
    angle, i.e. what the servo actually positions.
    """
    p = np.asarray(pulses, dtype=float)
    q = np.asarray(angles_eff, dtype=float)
    if len(p) < 3 or np.ptp(p) < 20.0:
        raise ValueError(f'{joint}: not enough servo travel in the samples')
    if cal.linkage.direct:
        m, c = np.polyfit(p, q, 1)
        if abs(m) < 1e-4:
            raise ValueError(f'{joint}: the joint did not move - wrong channel or servo unpowered?')
        direction = 1 if m > 0 else -1
        us_per_deg = 1.0 / abs(m)
        offset = m * cal.center_us + c
        rms = float(np.sqrt(np.mean((q - (m * p + c)) ** 2)))
        range_deg = (cal.pulse_max_us - cal.pulse_min_us) / us_per_deg
        return FitResult(joint, direction, float(offset), float(range_deg), rms, len(p), False)
    # Rod drive: keep the servo scale, fit sign and zero through the linkage model.
    g = np.array([cal.linkage.joint_delta((x - cal.center_us) / cal.us_per_deg) for x in p])
    best = None
    for d in (1, -1):
        offset = float(np.mean(q - d * g))
        rms = float(np.sqrt(np.mean((q - (offset + d * g)) ** 2)))
        if best is None or rms < best[2]:
            best = (d, offset, rms)
    d, offset, rms = best
    return FitResult(joint, d, offset, cal.range_deg, rms, len(p), True)


def find_limit(commanded, measured, predicted_slope_sign, tolerance_deg=3.0):
    """Index of the last sample that still followed the command.

    commanded / measured are sequences moving away from a safe start; the joint
    'stops following' when measured lags the commanded change by > tolerance.
    """
    c0, m0 = commanded[0], measured[0]
    last = 0
    for i in range(1, len(commanded)):
        expected = (commanded[i] - c0)
        got = (measured[i] - m0) * predicted_slope_sign
        if abs(expected - got) > tolerance_deg:
            break
        last = i
    return last


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def angle_error(a, b):
    return abs(wrap(a - b))


def fmt(v, nd=1):
    return ('+' if v >= 0 else '−') + f'{abs(v):.{nd}f}' if isinstance(v, float) and math.isfinite(v) else str(v)
