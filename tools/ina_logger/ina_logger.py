#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Логгер датчика тока INA219 (0x41, шина 0) + температур SoC.

Пишет строку ~4 раза в секунду в /home/sg/ina/ina-YYYYMMDD.log:
    время с мс, uptime, напряжение шины серв [В], ток [А], температуры зон [°C]
fsync раз в секунду — последние записи переживают пропадание питания.
Запуск: systemd-сервис ina-logger.service (tools/ina_logger/ina-logger.service).
"""
import datetime
import fcntl
import glob
import os
import time

I2C_SLAVE = 0x0703
DEV = '/dev/i2c-0'
ADDR = 0x41          # INA219 на шине робота
SHUNT_OHM = 0.1      # модуль R100
RATE_HZ = 4.0        # частота записи, Гц
OUT_DIR = '/home/sg/ina'


def open_ina():
    fd = os.open(DEV, os.O_RDWR)
    fcntl.ioctl(fd, I2C_SLAVE, ADDR)
    return fd


def read_reg16(fd, reg):
    """Регистр INA219: сначала адрес, затем 2 байта, старший вперёд."""
    os.write(fd, bytes([reg]))
    raw = os.read(fd, 2)
    if len(raw) != 2:
        return None
    v = (raw[0] << 8) | raw[1]
    return v - 65536 if v > 32767 else v


def temps():
    out = []
    for p in sorted(glob.glob('/sys/class/thermal/thermal_zone*/temp')):
        try:
            with open(p) as fh:
                out.append('%.1f' % (int(fh.read().strip()) / 1000.0))
        except (OSError, ValueError):
            pass
    return '/'.join(out) if out else '-'


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    fd = None
    f = None
    last_fsync = 0.0
    last_note = 0.0
    while True:
        t = time.time()
        if fd is None:
            try:
                fd = open_ina()
            except OSError:
                fd = None
        line = None
        if fd is not None:
            try:
                sh = read_reg16(fd, 0x01)      # shunt: 10 мкВ/разряд
                bs = read_reg16(fd, 0x02)      # bus: 4 мВ/разряд, биты 15..3
                if sh is not None and bs is not None:
                    amps = sh * 10e-6 / SHUNT_OHM
                    volts = (bs >> 3) * 4e-3
                    try:
                        with open('/proc/uptime') as fh:
                            up = float(fh.read().split()[0])
                    except OSError:
                        up = 0.0
                    line = '%s up=%7.1f  %6.3f V  %+8.4f A  T=%s' % (
                        datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                        up, volts, amps, temps())
            except OSError:
                try:
                    os.close(fd)
                except OSError:
                    pass
                fd = None
        if line is None:
            if t - last_note >= 10.0:
                line = '%s  !! нет данных с INA219/шины' % \
                    datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                last_note = t
        if line is not None:
            try:
                path = os.path.join(
                    OUT_DIR, 'ina-%s.log' % datetime.datetime.now().strftime('%Y%m%d'))
                if f is None or f.name != path:
                    if f is not None:
                        f.close()
                    f = open(path, 'a')
                f.write(line + '\n')
                if t - last_fsync >= 1.0:
                    f.flush()
                    os.fsync(f.fileno())
                    last_fsync = t
            except OSError:
                f = None
        time.sleep(1.0 / RATE_HZ)


if __name__ == '__main__':
    main()
