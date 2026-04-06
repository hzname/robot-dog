#!/usr/bin/env python3
"""
Тест сервоприводов Robot Dog.
PCA9685 на I2C-5, адрес 0x30.
Прямое управление через smbus2 (без Adafruit Blinka).

12 сервоприводов (каналы 0-11):
  0-2:  FL — hip, thigh, calf
  3-5:  FR — hip, thigh, calf
  6-8:  BL — hip, thigh, calf
  9-11: BR — hip, thigh, calf
"""

import time
import sys
import struct

# PCA9685 регистры
MODE1      = 0x00
MODE2      = 0x01
PRESCALE   = 0xFE
LED0_ON_L  = 0x06

I2C_BUS = 5
I2C_ADDR = 0x30

SERVO_MIN = 150   # ~0°
SERVO_MAX = 600   # ~180°
SERVO_MID = 375   # ~90°

JOINTS = {
    0:  "FL_hip",   1:  "FL_thigh", 2:  "FL_calf",
    3:  "FR_hip",   4:  "FR_thigh", 5:  "FR_calf",
    6:  "BL_hip",   7:  "BL_thigh", 8:  "BL_calf",
    9:  "BR_hip",   10: "BR_thigh", 11: "BR_calf",
}

class PCA9685:
    def __init__(self, bus_num, addr):
        import smbus2
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr
        self._write(MODE1, 0x00)
        self.set_freq(50)

    def _write(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _read(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def set_freq(self, freq):
        prescale = int(25000000.0 / (4096 * freq) - 1)
        # Прочитаем текущий режим
        old = self._read(MODE1)
        # Сброс (нужен для PCA9685)
        self.bus.write_byte_data(self.addr, MODE1, 0x00)  # FULL STOP
        time.sleep(0.01)
        # Режим сна для установки prescale
        self.bus.write_byte_data(self.addr, MODE1, 0x10)  # SLEEP
        time.sleep(0.01)
        self.bus.write_byte_data(self.addr, PRESCALE, prescale)
        # Перезапуск
        self.bus.write_byte_data(self.addr, MODE1, 0x80)  # RESTART
        time.sleep(0.05)
        self._write(MODE2, 0x04)  # outdrv: totem pole

    def set_pwm(self, channel, on, off):
        reg = LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(self.addr, reg, [
            on & 0xFF, (on >> 8) & 0xFF,
            off & 0xFF, (off >> 8) & 0xFF
        ])

    def set_angle(self, channel, angle):
        """Установить угол (0-180°)"""
        angle = max(0, min(180, angle))
        pwm = int(SERVO_MIN + (angle / 180.0) * (SERVO_MAX - SERVO_MIN))
        self.set_pwm(channel, 0, pwm)

    def off(self, channel):
        self.set_pwm(channel, 0, 0)


def test_servo(pca, channel, name, swing=30):
    print(f"\n=== Канал {channel}: {name} ===")
    mid = 90

    print(f"  Нейтраль ({mid}°)")
    pca.set_angle(channel, mid)
    time.sleep(0.5)

    print(f"  +{swing}° ({mid + swing}°)")
    pca.set_angle(channel, mid + swing)
    time.sleep(0.5)

    print(f"  Нейтраль ({mid}°)")
    pca.set_angle(channel, mid)
    time.sleep(0.5)

    print(f"  -{swing}° ({mid - swing}°)")
    pca.set_angle(channel, mid - swing)
    time.sleep(0.5)

    print(f"  Нейтраль ({mid}°)")
    pca.set_angle(channel, mid)
    time.sleep(0.3)

    print(f"  ✓ {name} OK")


if __name__ == "__main__":
    print("Robot Dog — Тест сервоприводов")
    print(f"PCA9685: I2C-{I2C_BUS}, адрес 0x{I2C_ADDR:02X}")
    print(f"Диапазон: 90° ± 30°")

    pca = PCA9685(I2C_BUS, I2C_ADDR)

    if len(sys.argv) > 1:
        ch = int(sys.argv[1])
        test_servo(pca, ch, JOINTS.get(ch, f"ch{ch}"))
    else:
        for ch, name in JOINTS.items():
            try:
                test_servo(pca, ch, name)
            except Exception as e:
                print(f"  ✗ {name}: ОШИБКА — {e}")

    print("\n=== Тест завершён ===")
