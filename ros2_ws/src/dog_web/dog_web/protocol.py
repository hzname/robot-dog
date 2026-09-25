"""Browser <-> robot message handling, independent of ROS (unit-tested).

Client -> server (JSON text frames):
  {"type": "drive", "vx": -1..1, "vy": -1..1, "wz": -1..1}   normalized, >= 10 Hz while moving
  {"type": "stop"}
  {"type": "command", "name": "stand" | "lie"}
  {"type": "estop", "active": true | false}
  {"type": "pose", "pitch": -1..1, "height": <metres offset>}
Server -> client:
  {"type": "hello", "limits": {...}}
  {"type": "state", "mode": "...", "estop": bool, "clients": n}
  {"type": "power", "voltage": V, "current": A}      only when a current sensor is fitted
  {"type": "guard", "state": "clear|caution|step_over|stop", "d": m | null}
                                                      only with dog_perception running
"""

import json
import math
from dataclasses import dataclass, field


@dataclass
class Limits:
    max_vx: float = 0.15
    max_vy: float = 0.08
    max_wz: float = 0.6
    max_pitch: float = 0.26
    min_height: float = -0.05
    max_height: float = 0.03


@dataclass
class Actions:
    """What the ROS side should publish for one incoming message."""

    twist: tuple = None       # (vx, vy, wz) in SI units
    command: str = None
    estop: bool = None
    pose: tuple = None        # (pitch, height)
    errors: list = field(default_factory=list)


def _num(msg, key, lo, hi, default=0.0):
    v = msg.get(key, default)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError(f'"{key}" must be a finite number')
    return min(max(float(v), lo), hi)


def handle_message(text: str, limits: Limits) -> Actions:
    out = Actions()
    try:
        msg = json.loads(text)
        if not isinstance(msg, dict):
            raise ValueError('message must be a JSON object')
        kind = msg.get('type')
        if kind == 'drive':
            out.twist = (
                _num(msg, 'vx', -1, 1) * limits.max_vx,
                _num(msg, 'vy', -1, 1) * limits.max_vy,
                _num(msg, 'wz', -1, 1) * limits.max_wz,
            )
        elif kind == 'stop':
            out.twist = (0.0, 0.0, 0.0)
        elif kind == 'command':
            name = msg.get('name')
            if name not in ('stand', 'lie'):
                raise ValueError('command must be "stand" or "lie"')
            out.command = name
            out.twist = (0.0, 0.0, 0.0)
        elif kind == 'estop':
            active = msg.get('active')
            if not isinstance(active, bool):
                raise ValueError('"active" must be true or false')
            out.estop = active
            if active:
                out.twist = (0.0, 0.0, 0.0)
        elif kind == 'pose':
            out.pose = (
                _num(msg, 'pitch', -1, 1) * limits.max_pitch,
                _num(msg, 'height', limits.min_height, limits.max_height),
            )
        else:
            raise ValueError(f'unknown message type {kind!r}')
    except (ValueError, json.JSONDecodeError) as exc:
        out = Actions(errors=[str(exc)])
    return out


class DriveWatchdog:
    """Stops the robot when the browser stops streaming drive messages
    (tab closed, network dropped, phone locked)."""

    def __init__(self, timeout: float):
        self.timeout = timeout
        self.moving = False
        self.last = 0.0

    def feed(self, twist, now: float):
        self.moving = any(abs(v) > 1e-9 for v in twist)
        self.last = now

    def expired(self, now: float) -> bool:
        """True once per timeout while moving; the caller then sends a stop."""
        if self.moving and now - self.last > self.timeout:
            self.moving = False
            return True
        return False


def hello(limits: Limits) -> str:
    return json.dumps({'type': 'hello', 'limits': limits.__dict__})


def state(mode: str, estop: bool, clients: int) -> str:
    return json.dumps({'type': 'state', 'mode': mode, 'estop': estop, 'clients': clients})


GUARD_STATES = ('clear', 'caution', 'step_over', 'stop')


def guard(raw: str) -> str:
    """perception/guard JSON -> the web message (only what the page shows)."""
    g = json.loads(raw)
    state = g.get('state') if g.get('state') in GUARD_STATES else 'clear'
    d = g.get('d')
    return json.dumps({'type': 'guard', 'state': state,
                       'd': round(float(d), 2) if isinstance(d, (int, float)) else None})


def power(voltage: float, current: float) -> str:
    return json.dumps({'type': 'power', 'voltage': round(float(voltage), 2),
                       'current': round(float(current), 2)})
