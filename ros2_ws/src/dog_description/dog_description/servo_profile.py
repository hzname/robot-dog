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
