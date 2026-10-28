"""One name, every conservative setting.

The first walk outside has to be one launch argument, not a list of numbers typed
by hand: a typo in that list is a fall. This module holds no ROS, so the values can
be tested without a running graph; `robot.launch.py` reads them and passes them as
node parameters.

`floor` follows the phase-5 criterion: 0.05 m/s, turbo no faster than the normal
speed, the IMU required, slope compensation and heading hold off, the web page
view-only and calibration closed.
"""

BENCH = 'bench'
FLOOR = 'floor'
PROFILES = (BENCH, FLOOR)

# Parameters that differ from the YAML files. Everything not named here stays as
# `robot.yaml` / `teleop.yaml` say, so bench runs and the simulator do not change.
_PARAMS = {
    BENCH: {},
    FLOOR: {
        # Locomotion clamps what any client can ask for, so the limit holds even if
        # a client ignores the teleop limits.
        'locomotion': {
            'limits.max_vx': 0.05,
            'limits.max_vy': 0.03,
            'limits.max_wz': 0.30,
            'slope.compensation': False,
            'heading.hold': False,
            'safety.profile': FLOOR,
            'safety.require_imu': True,
        },
        'joy_teleop': {
            'normal_scale': 0.5,
            'turbo_scale': 0.5,   # RB gives nothing extra on the floor
            'max_vx': 0.10,       # x0.5 = 0.05 m/s at full stick
            'max_vy': 0.06,
            'max_wz': 0.40,
        },
        'web_teleop': {
            'drive_enabled': False,
            'allow_calibration': False,
        },
    },
}


def check(name: str) -> str:
    """Validate a profile name; returns it, raises ValueError otherwise."""
    name = (name or '').strip().lower()
    if name not in PROFILES:
        raise ValueError('unknown profile %r: expected one of %s' % (name, ', '.join(PROFILES)))
    return name


def params(name: str, node: str) -> dict:
    """Node parameters for a profile; empty when the profile changes nothing there."""
    return dict(_PARAMS[check(name)].get(node, {}))


def web_drive(name: str) -> bool:
    """May the web page drive the robot in this profile?"""
    return bool(_PARAMS[check(name)].get('web_teleop', {}).get('drive_enabled', True))


def allow_calibration(name: str, calibrate: bool = False) -> bool:
    """Calibration is closed by default everywhere and never opens on the floor."""
    if check(name) == FLOOR:
        return False
    return bool(calibrate)


def summary(name: str) -> str:
    """One line for the launch log: what the profile changed."""
    name = check(name)
    parts = []
    for node, values in sorted(_PARAMS[name].items()):
        for key, value in sorted(values.items()):
            parts.append('%s.%s=%s' % (node, key, value))
    return 'profile %s: %s' % (name, ', '.join(parts) if parts else 'YAML defaults')
