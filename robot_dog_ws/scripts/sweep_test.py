#!/usr/bin/env python3
"""
Последовательный тест 12 сервоприводов.
Каждый канал вращается от min до max и обратно, затем следующий.
Частота переключения между каналами — 1 секунда.
"""

import time
import smbus2

I2C_BUS = 0
I2C_ADDR = 0x40

SERVO_MIN = 150   # ~0°
SERVO_MAX = 600   # ~180°
SERVO_MID = 375   # ~90°

ANGLE_MIN = 30
ANGLE_MAX = 150
STEP = 10
STEP_DELAY = 0.03  # задержка между шагами (сек)

JOINTS = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "BL_hip", "BL_thigh", "BL_calf",
    "BR_hip", "BR_thigh", "BR_calf",
]


class PCA9685:
    def __init__(self, bus_num, addr):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr
        self._write(0x00, 0x00)
        time.sleep(0.01)
        self._write(0x00, 0x10)  # sleep
        time.sleep(0.01)
        self._write(0xFE, 121)   # prescale 50Hz
        self._write(0x00, 0x80)  # restart
        time.sleep(0.05)
        self._write(0x01, 0x04)  # totem pole

    def _write(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def set_angle(self, channel, angle):
        angle = max(0, min(180, angle))
        pwm = int(SERVO_MIN + (angle / 180.0) * (SERVO_MAX - SERVO_MIN))
        reg = 0x06 + 4 * channel
        self.bus.write_i2c_block_data(self.addr, reg, [0, 0, pwm & 0xFF, (pwm >> 8) & 0xFF])

    def off(self, channel):
        self.bus.write_i2c_block_data(self.addr, 0x06 + 4 * channel, [0, 0, 0, 0])


def sweep(pca, channel, name):
    """Вращает серву от min до max и обратно"""
    print(f"[Канал {channel:2d}] {name}: {ANGLE_MIN}° → {ANGLE_MAX}° → {ANGLE_MIN}°")

    # min → max
    for a in range(ANGLE_MIN, ANGLE_MAX + 1, STEP):
        pca.set_angle(channel, a)
        time.sleep(STEP_DELAY)

    # max → min
    for a in range(ANGLE_MAX, ANGLE_MIN - 1, -STEP):
        pca.set_angle(channel, a)
        time.sleep(STEP_DELAY)

    # возврат в нейтраль
    pca.set_angle(channel, 90)


if __name__ == "__main__":
    print("Robot Dog — Последовательный тест сервоприводов")
    print(f"PCA9685: I2C-{I2C_BUS}, адрес 0x{I2C_ADDR:02X}")
    print(f"Диапазон: {ANGLE_MIN}° → {ANGLE_MAX}°, шаг {STEP}°")
    print()

    pca = PCA9685(I2C_BUS, I2C_ADDR)

    # Сначала все в нейтраль
    for ch in range(12):
        pca.set_angle(ch, 90)
    time.sleep(1)

    # Тест одного канала
    ch = 0
    name = JOINTS[ch]
    print(f"[Тест] {name}: 30° жду 1с")
    pca.set_angle(ch, 30)
    time.sleep(1)
    print(f"[Тест] {name}: 150° жду 1с")
    pca.set_angle(ch, 150)
    time.sleep(1)
    print(f"[Тест] {name}: 90° жду 1с")
    pca.set_angle(ch, 90)
    time.sleep(1)
    print("[Тест] OK\n")

    # По очереди каждый канал
    for ch, name in enumerate(JOINTS):
        sweep(pca, ch, name)
        time.sleep(1)

    # Все в нейтраль
    for ch in range(12):
        pca.set_angle(ch, 90)

    print("\n=== Готово ===")
