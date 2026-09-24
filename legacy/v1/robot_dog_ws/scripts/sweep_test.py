#!/usr/bin/env python3
"""
Robot Dog — Последовательный тест 12 сервоприводов.
Каждый канал плавно вращается от min до max и обратно.
Частота переключения между каналами — 1 секунда.

PCA9685 на I2C-0, адрес 0x40.
Безопасное отключение по Ctrl+C.
"""

import time
import sys
import signal
import smbus2

# === Конфигурация ===
I2C_BUS  = 0
I2C_ADDR = 0x40

# PCA9685: 50 Hz, 12-bit (0–4095)
# 1.0 мс = tick 205,  2.0 мс = tick 410
SERVO_MIN = 150    # ~0.5 мс  → ~0°
SERVO_MAX = 600    # ~2.5 мс  → ~180°

ANGLE_MIN = 30
ANGLE_MAX = 150
STEP      = 10
STEP_DELAY = 0.03   # задержка между шагами (сек)

JOINTS = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "BL_hip", "BL_thigh", "BL_calf",
    "BR_hip", "BR_thigh", "BR_calf",
]

# === PCA9685 регистры ===
MODE1     = 0x00
MODE2     = 0x01
PRESCALE  = 0xFE
LED0_ON_L = 0x06


class PCA9685:
    """Драйвер PCA9685 через smbus2."""

    def __init__(self, bus_num, addr):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr
        self.set_freq(50)

    def _write(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _read(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def set_freq(self, freq):
        prescale = int(25000000.0 / (4096 * freq) - 1)
        # По даташиту: sleep -> prescale -> wake -> restart
        old = self._read(MODE1)
        self._write(MODE1, (old & 0x7F) | 0x10)   # SLEEP
        time.sleep(0.01)
        self._write(PRESCALE, prescale)
        time.sleep(0.01)
        self._write(MODE1, old & 0x7F)             # WAKE (сброс SLEEP)
        time.sleep(0.01)
        self._write(MODE1, old | 0x80)             # RESTART
        time.sleep(0.05)
        self._write(MODE2, 0x04)                    # TOTEM POLE

    def set_pwm(self, channel, on, off):
        reg = LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(self.addr, reg, [
            on & 0xFF, (on >> 8) & 0xFF,
            off & 0xFF, (off >> 8) & 0xFF
        ])

    def set_angle(self, channel, angle):
        angle = max(0, min(180, angle))
        pwm = int(SERVO_MIN + (angle / 180.0) * (SERVO_MAX - SERVO_MIN))
        self.set_pwm(channel, 0, pwm)

    def close(self):
        """Отключить все каналы."""
        for ch in range(16):
            self.set_pwm(ch, 0, 0)


def sweep(pca, channel, name):
    """Плавный sweep от min до max и обратно."""
    print(f"[Канал {channel:2d}] {name}: {ANGLE_MIN}° → {ANGLE_MAX}° → {ANGLE_MIN}°")

    for a in range(ANGLE_MIN, ANGLE_MAX + 1, STEP):
        pca.set_angle(channel, a)
        time.sleep(STEP_DELAY)

    for a in range(ANGLE_MAX, ANGLE_MIN - 1, -STEP):
        pca.set_angle(channel, a)
        time.sleep(STEP_DELAY)

    pca.set_angle(channel, 90)


if __name__ == "__main__":
    print("Robot Dog — Последовательный тест сервоприводов")
    print(f"PCA9685: I2C-{I2C_BUS}, адрес 0x{I2C_ADDR:02X}")
    print(f"Диапазон: {ANGLE_MIN}° → {ANGLE_MAX}°, шаг {STEP}°")
    print(f"PWM: {SERVO_MIN}–{SERVO_MAX} (1.0–2.0 мс)")
    print()

    pca = PCA9685(I2C_BUS, I2C_ADDR)

    # Безопасное завершение по Ctrl+C
    def shutdown(sig=None, frame=None):
        print("\n⏹ Остановка, отключаю все сервы...")
        pca.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        # Все в нейтраль
        for ch in range(12):
            pca.set_angle(ch, 90)
        time.sleep(1)

        # По очереди каждый канал
        for ch, name in enumerate(JOINTS):
            sweep(pca, ch, name)
            time.sleep(1)

        # Все в нейтраль
        for ch in range(12):
            pca.set_angle(ch, 90)
        time.sleep(0.5)

        print("\n=== Готово ===")

    except Exception as e:
        print(f"\n✗ Ошибка: {e}")
    finally:
        pca.close()
