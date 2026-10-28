"""Browser <-> robot message handling, independent of ROS (unit-tested).

Client -> server (JSON text frames):
  {"type": "drive", "vx": -1..1, "vy": -1..1, "wz": -1..1}   normalized, >= 10 Hz while moving
  {"type": "stop"}
  {"type": "command", "name": "stand" | "lie" | "greet" | "survey" | "crawl" | "trot"}
  {"type": "estop", "active": true | false}
  {"type": "pose", "pitch": -1..1, "height": <metres offset>}
  {"type": "ping"}                                              keep-alive, no reply
Server -> client:
  {"type": "hello", "limits": {...}}
  {"type": "state", "mode": "...", "estop": bool, "clients": n}
  {"type": "power", "voltage": V, "current": A}      only when a current sensor is fitted
  {"type": "guard", "state": "clear|caution|step_over|crawl|stop|avoid", "d": m | null}
                                                      only with dog_perception running
"""

import json
import math
from dataclasses import dataclass, field

# locomotion_node commands the page may send (greet: sit, paws up, wave;
# crawl / trot: the operator's gait)
COMMANDS = ('stand', 'lie', 'greet', 'survey', 'crawl', 'trot')


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
            if name not in COMMANDS:
                raise ValueError('command must be one of ' + ', '.join(COMMANDS))
            out.command = name
            out.twist = (0.0, 0.0, 0.0)
        elif kind == 'estop':
            active = msg.get('active')
            if not isinstance(active, bool):
                raise ValueError('"active" must be true or false')
            out.estop = active
            if active:
                out.twist = (0.0, 0.0, 0.0)
        elif kind == 'ping':
            pass  # keep-alive from the page: the server closes silent sessions
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


def estop_decision(current: bool, requested: bool):
    """(allowed, publish, reason) for an E-STOP request from the web page.

    Both directions are the operator's: the page is the robot's own control panel,
    not a stranger's tab, and after an emergency the person in front of it must be
    able to release without walking to the gamepad (owner, 2026-10-29). What keeps a
    stranger out is the origin check, the private ROS domain and the network - not a
    missing button. `publish` is false when the request changes nothing, so a page
    that repeats itself does not spray the topic.
    """
    return True, bool(requested) != bool(current), ''


def gate_actions(actions, *, drive_enabled: bool, estop_engaged: bool):
    """Drop what this page may not do; returns (actions, errors).

    In view-only mode (profile:=floor) the page shows state and keeps the full
    E-STOP in both directions - releasing the latch is a safety action - but it
    cannot drive, command a pose or ask for a gait.
    """
    errors = []
    if actions.estop is not None:
        allowed, publish, reason = estop_decision(estop_engaged, actions.estop)
        if not allowed:
            errors.append(reason)
            actions.estop = None
        elif not publish:
            actions.estop = None
    if not drive_enabled:
        if actions.twist is not None:
            # An E-STOP message carries a zero twist on purpose; that is not an
            # attempt to drive, so it is dropped silently.
            if any(actions.twist):
                errors.append('view only: this robot runs the floor profile')
            actions.twist = None
        if actions.command:
            actions.command = None
            errors.append('view only: commands come from the gamepad')
        if actions.pose is not None:
            actions.pose = None
            errors.append('view only: body pose comes from the gamepad')
    return actions, errors


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


def state(mode: str, estop: bool, clients: int, view_only: bool = False) -> str:
    return json.dumps({'type': 'state', 'mode': mode, 'estop': estop, 'clients': clients,
                       'view_only': bool(view_only)})


GUARD_STATES = ('clear', 'caution', 'step_over', 'crawl', 'stop', 'avoid')


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


LEVELS = ('ok', 'warn', 'error')


def worst_status(items):
    """[(level, source, message), ...] -> the worst one (level, source, message).

    The page shows one banner, so one fault has to win: an ERROR outranks a WARN,
    and among equals the first one wins (the order is the diagnostic array's). An
    empty list is 'all clear'.
    """
    best = ('ok', '', '')
    for level, source, message in items:
        if level not in LEVELS:
            continue
        if LEVELS.index(level) > LEVELS.index(best[0]):
            best = (level, source, message)
    return best


def alarm(level: str, source: str, reason: str, since: float) -> str:
    return json.dumps({'type': 'alarm', 'level': level, 'source': source,
                       'reason': reason, 'since': round(float(since), 1)})
