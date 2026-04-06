#!/usr/bin/env python3
"""
Тест сервоприводов Robot Dog.
Поочерёдно двигает каждым суставом для проверки.

PCA9685 на I2C-5, адрес 0x30.
12 сервоприводов (каналы 0-11):
  0-2:  FL (Front-Left)  — hip, thigh, calf
  3-5:  FR (Front-Right) — hip, thigh, calf
  6-8:  BL (Back-Left)   — hip, thigh, calf
  9-11: BR (Back-Right)  — hip, thigh, calf
"""

import time
import sys
import board
from adafruit_servokit import ServoKit

# Инициализация PCA9685 на шине I2C-5, адрес 0x30
import busio
i2c = busio.I2C(board.SCL_5, board.SDA_5)
kit = ServoKit(channels=16, i2c=i2c, address=0x30)

# Маппинг каналов -> суставы
JOINTS = {
    0:  "FL_hip",
    1:  "FL_thigh",
    2:  "FL_calf",
    3:  "FR_hip",
    4:  "FR_thigh",
    5:  "FR_calf",
    6:  "BL_hip",
    7:  "BL_thigh",
    8:  "BL_calf",
    9:  "BR_hip",
    10: "BR_thigh",
    11: "BR_calf",
}

# Нейтральные позиции (градусы, 90 = центр)
NEUTRAL = 90
RANGE = 30  # отклонение от нейтрали (±30°)

def test_servo(channel, name):
    """Двигает один сервопривод: нейтраль -> +30° -> нейтраль -> -30° -> нейтраль"""
    print(f"\n=== Канал {channel}: {name} ===")
    servo = kit.servo[channel]
    servo.set_pulse_width_range(500, 2500)  # стандартный диапазон для SG90/MG996R
    
    print(f"  Нейтраль ({NEUTRAL}°)")
    servo.angle = NEUTRAL
    time.sleep(0.5)
    
    print(f"  +{RANGE}° ({NEUTRAL + RANGE}°)")
    servo.angle = NEUTRAL + RANGE
    time.sleep(0.5)
    
    print(f"  Нейтраль ({NEUTRAL}°)")
    servo.angle = NEUTRAL
    time.sleep(0.5)
    
    print(f"  -{RANGE}° ({NEUTRAL - RANGE}°)")
    servo.angle = NEUTRAL - RANGE
    time.sleep(0.5)
    
    print(f"  Нейтраль ({NEUTRAL}°)")
    servo.angle = NEUTRAL
    time.sleep(0.3)
    
    print(f"  ✓ {name} OK")

def test_all():
    """Тестирует все 12 сервоприводов по очереди"""
    print("Robot Dog — Тест сервоприводов")
    print(f"PCA9685: I2C-5, адрес 0x30")
    print(f"Сервоприводов: 12 (каналы 0-11)")
    print(f"Диапазон: {NEUTRAL}° ± {RANGE}°")
    print()
    
    for ch, name in JOINTS.items():
        try:
            test_servo(ch, name)
        except Exception as e:
            print(f"  ✗ {name}: ОШИБКА — {e}")
    
    print("\n=== Тест завершён ===")

def test_one(channel):
    """Тестирует один сервопривод по номеру канала"""
    name = JOINTS.get(channel, f"Unknown({channel})")
    test_servo(channel, name)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # python test_servos.py <канал>
        test_one(int(sys.argv[1]))
    else:
        test_all()
