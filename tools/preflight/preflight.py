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


def gamepad_devices(root=''):
    """['js0', ...] - the joystick devices the container will see."""
    pattern = os.path.join(root, 'dev/input/js*')
    return [os.path.basename(p) for p in sorted(glob.glob(pattern))]


def checks(root, bus, addresses, gamepad):
    """[(ok, text, required)] - every line the operator will see."""
    items = []
    path = os.path.join(root, bus.lstrip('/'))
    if not os.path.exists(path):
        items.append((False, '%s: нет устройства — шина I2C не включена?' % bus, True))
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
    a = ap.parse_args(argv)

    if a.mock or 'backend:=mock' in a.args:
        print('preflight: backend:=mock — железо не требуется')
        return 0

    gamepad = a.gamepad
    if gamepad == 'auto':
        gamepad = 'false' if 'gamepad:=false' in a.args else 'true'

    items = checks(a.root, a.bus, ADDRESSES, gamepad == 'true')
    for ok, text, required in items:
        print('%s %s%s' % ('ok    ' if ok else 'ПЛОХО ', text,
                           '' if required else '   (необязательно)'))

    missing = blocked(items)
    if missing:
        print('preflight: запуск заблокирован, не хватает:')
        for text in missing:
            print('   - ' + text)
        print('preflight: проверьте питание, шлейф I2C и подключённый джойстик')
        return 2
    print('preflight: всё на месте')
    return 0


if __name__ == '__main__':
    sys.exit(main())
