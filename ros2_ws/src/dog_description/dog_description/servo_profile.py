"""Servo profile of servo_model=real: bus-voltage factors and the command shaper.

Pure stdlib (math, dataclasses, collections). urdf.py turns the profile into
URDF attributes (joint friction, effort/velocity factors, knee rod ratio) and
joint_command_bridge.py turns it into backlash and command delay. The defaults
equal the servo_sim block of robot.yaml (D-16); the ideal model (the default,
D-15) uses none of this.
"""

import math
from collections import deque
from dataclasses import dataclass

V_MIN = 4.8
V_MAX = 6.6
V_REF_DEFAULT = 6.0
# MG996R datasheet points (docs/HARDWARE.md): 4.8 V -> 0.17 s/60deg, 9.4 kg*cm;
# 6.0 V -> 0.14 s/60deg, 11 kg*cm. The slopes are those two points per volt.
SPEED_PER_VOLT = 0.1471
TORQUE_PER_VOLT = 0.1212
_TIME_EPS = 1e-9  # [s] time comparison tolerance of DelayLine


def _number(key, value, minimum=None, exclusive=False):
    """Finite float or ValueError naming `key`; bool is not a number.

    `minimum` is inclusive unless `exclusive` is set."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'"{key}" must be a finite number')
    out = float(value)
    if minimum is not None and (out < minimum or (exclusive and out == minimum)):
        raise ValueError(f'"{key}" must be {"greater than" if exclusive else "at least"} {minimum:g}')
    return out


def _voltage(key, value):
    return _number(key, value, minimum=0.0, exclusive=True)


def _non_negative(name, value):
    """Non-negative finite number or ValueError naming `name`; returns float."""
    return _number(name, value, minimum=0.0)


def speed_factor(v, v_ref=V_REF_DEFAULT):
    """Speed multiplier at servo bus voltage `v` (clamped to [V_MIN, V_MAX])."""
    v = _voltage('v', v)
    v_ref = _voltage('v_ref', v_ref)
    if not V_MIN <= v_ref <= V_MAX:
        raise ValueError(f'"v_ref" must be within [{V_MIN}, {V_MAX}]')
    v = min(max(v, V_MIN), V_MAX)
    return 1.0 + SPEED_PER_VOLT * (v - v_ref)


def torque_factor(v, v_ref=V_REF_DEFAULT):
    """Torque multiplier at servo bus voltage `v` (clamped to [V_MIN, V_MAX])."""
    v = _voltage('v', v)
    v_ref = _voltage('v_ref', v_ref)
    if not V_MIN <= v_ref <= V_MAX:
        raise ValueError(f'"v_ref" must be within [{V_MIN}, {V_MAX}]')
    v = min(max(v, V_MIN), V_MAX)
    return 1.0 + TORQUE_PER_VOLT * (v - v_ref)


@dataclass
class ServoProfile:
    """Reference servo imperfections of the simulation (D-16); defaults = robot.yaml servo_sim."""

    backlash_deg: float = 1.5
    delay_ms: float = 40.0
    friction_nm: float = 0.06
    bus_voltage: float = 6.0
    bus_voltage_ref: float = 6.0

    @classmethod
    def from_params(cls, params):
        """From the servo_sim YAML block: missing keys keep the defaults, extra keys are ignored."""
        p = params or {}
        default = cls()
        return cls(
            backlash_deg=_number('backlash_deg', p.get('backlash_deg', default.backlash_deg), minimum=0.0),
            delay_ms=_number('delay_ms', p.get('delay_ms', default.delay_ms), minimum=0.0),
            friction_nm=_number('friction_nm', p.get('friction_nm', default.friction_nm), minimum=0.0),
            bus_voltage=_voltage('bus_voltage', p.get('bus_voltage', default.bus_voltage)),
            bus_voltage_ref=_voltage('bus_voltage_ref', p.get('bus_voltage_ref', default.bus_voltage_ref)),
        )


class BacklashPlay:
    """Output follows the command only after the play `width_rad` is used up."""

    def __init__(self, width_rad):
        self.half = 0.5 * _non_negative('width_rad', width_rad)
        self.y = None

    def __call__(self, x):
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
            return x  # a non-finite command comes back as it is, the state is untouched
        if self.y is None:
            self.y = x
        elif x - self.y > self.half:
            self.y = x - self.half
        elif self.y - x > self.half:
            self.y = x + self.half
        return self.y


class DelayLine:
    """FIFO of (t, item) entries; an entry is ready when `t + delay_s <= now`.

    The caller owns the clock (the bridge passes node time; under use_sim_time
    it is the simulation clock). `push` appends, `pop_ready` releases in order.
    """

    def __init__(self, delay_s):
        self.delay = _non_negative('delay_s', delay_s)
        self._items = deque()

    def push(self, t, item):
        self._items.append((t, item))

    def pop_ready(self, now):
        out = []
        while self._items and self._items[0][0] + self.delay <= now + _TIME_EPS:
            out.append(self._items.popleft()[1])
        return out

    def __len__(self):
        return len(self._items)


class CommandShaper:
    """Backlash per joint and one shared delay: the servo_model=real command path (D-14)."""

    def __init__(self, backlash_rad, delay_s):
        self.backlash_rad = _non_negative('backlash_rad', backlash_rad)
        self.delay = DelayLine(delay_s)
        self._plays = {}

    def push(self, t, names, positions):
        for name, pos in zip(names, positions):
            play = self._plays.get(name)
            if play is None:
                play = self._plays[name] = BacklashPlay(self.backlash_rad)
            self.delay.push(t, (name, play(pos)))

    def pop_ready(self, now):
        return self.delay.pop_ready(now)


def parse_override(name, text, positive=False):
    """Launch-argument override: None or blank -> None, else a finite float.

    Negative values are rejected; with `positive=True` zero is too.
    """
    if text is None:
        return None
    if isinstance(text, str):
        stripped = text.strip()
        if not stripped:
            return None
        try:
            value = float(stripped)
        except ValueError:
            raise ValueError(f'"{name}" must be a finite number') from None
    elif isinstance(text, (int, float)) and not isinstance(text, bool):
        value = float(text)
    else:
        raise ValueError(f'"{name}" must be a finite number')
    if not math.isfinite(value):
        raise ValueError(f'"{name}" must be a finite number')
    if value < 0.0 or (positive and value == 0.0):
        raise ValueError(f'"{name}" must be positive' if positive else f'"{name}" must not be negative')
    return value


def bridge_settings(servo_model, servo_sim, delay_ms=None):
    """(backlash_deg, delay_s) for joint_command_bridge; zeros on the ideal model (D-15)."""
    if servo_model not in ('ideal', 'real'):
        raise ValueError(f'unknown servo_model {servo_model!r} (use ideal or real)')
    if servo_model == 'ideal':
        return (0.0, 0.0)
    profile = ServoProfile.from_params(servo_sim)
    ms = profile.delay_ms if delay_ms is None else delay_ms
    return (profile.backlash_deg, _non_negative('delay_ms', ms) / 1000.0)
