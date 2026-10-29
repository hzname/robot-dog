#!/usr/bin/env python3
"""Preflight: is the hardware this launch is about to touch actually there?

Runs on the robot in front of ``ros2 launch`` (the container's entrypoint calls it)
and inside the arm64 image in CI, where it must refuse: there is no ``/dev/i2c-0``
in a container. Pure stdlib - it opens the bus and lets each address ACK or not, so
it needs neither smbus nor a particular python build.

    preflight.py --args "backend:=pca9685 gamepad:=true"
    preflight.py --args "backend:=mock"     # nothing to check, exits 0
    preflight.py --root /tmp/fake-bus       # for tests

Exit codes: 0 - ready; 2 - blocked (the launch must not start).
"""

import argparse
import fcntl
import glob
import os
import subprocess
import sys

I2C_SLAVE = 0x0703          # linux/i2c-dev.h

BUS = '/dev/i2c-0'

# (address, what it is, required). The robot puts PCA9685, MPU6050, INA219 and the
# ToF sensor on one bus (docs/HARDWARE.md); only the first three are needed to walk.
ADDRESSES = (
    (0x40, 'PCA9685 (12 servos)', True),
    (0x68, 'MPU6050 (IMU)', True),
    (0x41, 'INA219 (current and bus voltage)', True),
    (0x29, 'VL53L1X (ToF)', False),
    (0x70, 'PCA9685 ALLCALL', False),
)


def probe(path, address):
    """None when the chip answers, otherwise the reason it does not."""
    try:
        fd = os.open(path, os.O_RDWR)
    except OSError as e:
        return 'не открыть %s: %s' % (path, e.strerror)
    try:
        fcntl.ioctl(fd, I2C_SLAVE, address)
        os.read(fd, 1)
        return None
    except OSError as e:
        return 'не отвечает: %s' % (e.strerror or 'ошибка шины')
    finally:
        os.close(fd)


PROFILE_FLOOR = 'floor'     # the profile for walking in the open, and the only one that
                            # demands a calibration of this particular robot

# Where the launch reads its config from: the image (install share) and the source tree.
CONFIG_REL = (os.path.join('install', 'dog_bringup', 'share', 'dog_bringup', 'config'),
              os.path.join('ros2_ws', 'src', 'dog_bringup', 'config'))


def launch_profile(args):
    """The `profile:=` launch argument, or '' when there is none."""
    for word in (args or '').split():
        if word.startswith('profile:='):
            return word.split(':=', 1)[1]
    return ''


def config_dir(here=None):
    """The folder the launch will read. An explicit DOG_CONFIG_DIR wins; then the two
    layouts this tool runs in, relative to its own place in the tree."""
    explicit = os.environ.get('DOG_CONFIG_DIR')
    if explicit:
        return explicit if os.path.isdir(explicit) else None
    here = here or os.path.dirname(os.path.abspath(__file__))
    for rel in CONFIG_REL:
        path = os.path.abspath(os.path.join(here, '..', '..', rel))
        if os.path.isdir(path):
            return path
    return None


def verify_calibration(config_dir_path):
    """(exit code, output) of `robot_setup --verify`: 0 calibrated, 3 not, 2 unreadable."""
    tool = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'robot_setup',
                                       'robot_setup.py'))
    try:
        done = subprocess.run([sys.executable, tool, '--verify', '--config', config_dir_path],
                              capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.SubprocessError) as e:
        return 1, 'не запустить robot_setup: %s' % e
    return done.returncode, (done.stdout + done.stderr).strip()


def calibration_gate(config_dir_path, profile, run=None):
    """[(ok, text, required)] for the `profile:=floor` launch: a robot walks in the open
    only with a calibration of its own, not with the factory table nobody touched."""
    if profile != PROFILE_FLOOR:
        return []
    if not config_dir_path:
        return [(False, 'калибровка: папка config не найдена (задайте DOG_CONFIG_DIR) — '
                        'профиль floor без неё не проверить', True)]
    local = os.path.join(config_dir_path, 'servos.local.yaml')
    if not os.path.exists(local):
        return [(False, 'калибровка: нет servos.local.yaml — у этого робота нет своей '
                        'калибровки (docs/CALIBRATION.md, этап 6)', True)]
    code, text = (run or verify_calibration)(config_dir_path)
    if code == 0:
        return [(True, 'калибровка: ' + (text.splitlines() or ['ок'])[0], True)]
    if code == 3:
        return [(False, 'калибровка: ' + ' / '.join(text.splitlines()[:3]), True)]
    return [(False, 'калибровка: проверка не запустилась (%s)' % text.splitlines()[:1], True)]


def gamepad_devices(root=''):
    """['js0', ...] - the joystick devices the container will see."""
    pattern = os.path.join(root, 'dev/input/js*')
    return [os.path.basename(p) for p in sorted(glob.glob(pattern))]


def checks(root, bus, addresses, gamepad, profile='', config=None, run=None):
    """[(ok, text, required)] - every line the operator will see."""
    items = []
    path = os.path.join(root, bus.lstrip('/'))
    if not os.path.exists(path):
        items.append((False, '%s: нет устройства — шина I2C не включена?' % bus, True))
        # Still worth saying whether the robot is calibrated: otherwise the operator fixes
        # the bus, starts again, and only then learns about the second problem.
        items += calibration_gate(config, profile, run)
        return items
    items.append((True, '%s: есть' % bus, True))
    for address, name, required in addresses:
        why = probe(path, address)
        items.append((why is None,
                      '0x%02X %s: %s' % (address, name, 'отвечает' if why is None else why),
                      required))
    if gamepad:
        found = gamepad_devices(root)
        items.append((bool(found),
                      '/dev/input/js0: %s' % (found[0] if found else 'джойстик не найден'),
                      True))
    items += calibration_gate(config, profile, run)
    return items


def blocked(items):
    """The lines that must stop the launch: required, and not answering."""
    return [t for ok, t, required in items if required and not ok]


def main(argv=None):
    ap = argparse.ArgumentParser(description='Check the robot before the launch.')
    ap.add_argument('--args', default='', help='the launch arguments as one string')
    ap.add_argument('--root', default='', help='prefix for /dev (tests)')
    ap.add_argument('--bus', default=BUS)
    ap.add_argument('--gamepad', choices=('auto', 'true', 'false'), default='auto')
    ap.add_argument('--mock', action='store_true', help='hardware is not needed')
    ap.add_argument('--config-dir', default=None,
                    help='folder with servos.yaml / servos.local.yaml (default: autodetect)')
    a = ap.parse_args(argv)

    if a.mock or 'backend:=mock' in a.args:
        print('preflight: backend:=mock — железо не требуется')
        return 0

    gamepad = a.gamepad
    if gamepad == 'auto':
        gamepad = 'false' if 'gamepad:=false' in a.args else 'true'

    profile = launch_profile(a.args)
    items = checks(a.root, a.bus, ADDRESSES, gamepad == 'true', profile,
                   a.config_dir if a.config_dir is not None else config_dir())
    for ok, text, required in items:
        print('%s %s%s' % ('ok    ' if ok else 'ПЛОХО ', text,
                           '' if required else '   (необязательно)'))

    missing = blocked(items)
    if missing:
        print('preflight: запуск заблокирован, не хватает:')
        for text in missing:
            print('   - ' + text)
        if any(not text.startswith('калибровка') for text in missing):
            print('preflight: проверьте питание, шлейф I2C и подключённый джойстик')
        if any(text.startswith('калибровка') for text in missing):
            print('preflight: калибровка — docs/CALIBRATION.md (этап 6), '
                  'профиль floor без неё не запускается')
        return 2
    print('preflight: всё на месте')
    return 0


if __name__ == '__main__':
    sys.exit(main())
