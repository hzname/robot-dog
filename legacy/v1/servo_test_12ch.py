#!/usr/bin/env python3
"""
Скрипт для поочерёдной проверки всех 12 каналов серво-контроллера.

Каждая серва вращается:
  начальное положение → конечное положение → начальное положение

Переключение между каналами — каждые 1 секунду.
"""

import time
import sys

# ============================================================
#  НАСТРОЙКИ — отредактируйте под ваше железо
# ============================================================

NUM_CHANNELS   = 12        # количество каналов серво-контроллера
I2C_BUS        = 0         # номер I2C шины (0 = /dev/i2c-0)
PCA_ADDRESS    = 0x40      # I2C-адрес PCA9685 (дефолтный)
PWM_FREQ       = 50        # частота ШИМ для серв (Гц)
INITIAL_PULSE  = 500       # стартовый импульс (мкс) — ~0°
FINAL_PULSE    = 2500      # конечный импульс (мкс) — ~180°
SWITCH_DELAY   = 1.0       # задержка между переключениями каналов (сек)
MOVE_DELAY     = 0.02      # задержка между шагами анимации (сек)
STEPS          = 50        # количество шагов для плавного движения

# ============================================================
#  АДАПТЕРЫ СЕРВО-КОНТРОЛЛЕРА
# ============================================================

class PCA9685Adapter:
    """Адаптер для PCA9685 через adafruit-circuitpython-servokit.

    Явно указывается шина I2C (например, /dev/i2c-0) через
    adafruit_extended_bus.ExtendedI2C.
    """

    def __init__(self, num_channels=12, i2c_bus=0, address=0x40, frequency=50):
        try:
            from adafruit_servokit import ServoKit
        except ImportError:
            print("[!] Установите библиотеку:")
            print("    pip install adafruit-circuitpython-servokit")
            print("    pip install adafruit-extended-bus")
            sys.exit(1)

        try:
            from adafruit_extended_bus import ExtendedI2C as I2C
        except ImportError:
            print("[!] Установите adafruit-extended-bus:")
            print("    pip install adafruit-extended-bus")
            sys.exit(1)

        # Явное указание шины I2C
        self._i2c = I2C(i2c_bus)
        self.kit = ServoKit(channels=num_channels, i2c=self._i2c, address=address)
        self.kit.frequency = frequency
        self._channels = self.kit.servo

        # Диапазон импульсов PCA9685
        self.pulse_min = 500
        self.pulse_max = 2500

        print(f"[+] PCA9685: /dev/i2c-{i2c_bus}, адрес 0x{address:02X}, "
              f"{num_channels} каналов, {frequency} Гц")

    def _pulse_to_angle(self, pulse_us):
        """Конвертировать ширину импульса (мкс) в угол 0-180°."""
        fraction = (pulse_us - self.pulse_min) / (self.pulse_max - self.pulse_min)
        fraction = max(0.0, min(1.0, fraction))
        return fraction * 180.0

    def set_pulse(self, channel, pulse_us):
        angle = self._pulse_to_angle(pulse_us)
        self._channels[channel].angle = angle

    def release(self, channel):
        self._channels[channel].angle = None  # отключить сигнал (если поддерживается)


class MockAdapter:
    """Имитация сервоконтроллера для отладки без железа."""

    def __init__(self):
        print("[+] MockAdapter: режим симуляции (без реального железа)")

    def set_pulse(self, channel, pulse_us):
        angle = (pulse_us - 500) / (2500 - 500) * 180
        print(f"    [канал {channel:2d}] → импульс {pulse_us:5d} мкс (~{angle:5.1f}°)")

    def release(self, channel):
        print(f"    [канал {channel:2d}] → отключён")


# ============================================================
#  РАБОЧИЕ ФУНКЦИИ
# ============================================================

def lerp(a, b, t):
    """Линейная интерполяция от a к b по коэффициенту t ∈ [0, 1]."""
    return a + (b - a) * t


def test_channel(adapter, ch, initial, final, steps, move_delay):
    """
    Проверить один канал: начальное → конечное → начальное.
    Возвращает True, если всё прошло успешно.
    """
    print(f"\n━━━ Канал {ch + 1}/{NUM_CHANNELS} ━━━")

    # --- начальное → конечное ---
    for i in range(steps + 1):
        t = i / steps
        pulse = int(lerp(initial, final, t))
        adapter.set_pulse(ch, pulse)
        time.sleep(move_delay)

    # Пауза в конечном положении
    time.sleep(SWITCH_DELAY)

    # --- конечное → начальное ---
    for i in range(steps + 1):
        t = i / steps
        pulse = int(lerp(final, initial, t))
        adapter.set_pulse(ch, pulse)
        time.sleep(move_delay)

    # Пауза в начальном положении перед следующим каналом
    time.sleep(SWITCH_DELAY)

    adapter.release(ch)
    return True


def run_test(adapter, num_channels, initial, final, steps, move_delay, switch_delay):
    """Последовательно протестировать все каналы."""
    print("=" * 50)
    print("   ТЕСТИРОВАНИЕ СЕРВО-КАНАЛОВ")
    print(f"   Каналов: {num_channels}")
    print(f"   Диапазон импульса: {initial}–{final} мкс")
    print(f"   Задержка переключения: {switch_delay} с")
    print(f"   Шагов анимации: {steps}")
    print("=" * 50)

    results = {}
    for ch in range(num_channels):
        try:
            ok = test_channel(adapter, ch, initial, final, steps, move_delay)
            results[ch] = ok
        except Exception as e:
            print(f"    [!] Ошибка на канале {ch}: {e}")
            results[ch] = False

    # --- Итоги ---
    print("\n" + "=" * 50)
    print("   РЕЗУЛЬТАТЫ")
    print("=" * 50)
    passed = sum(1 for v in results.values() if v)
    for ch, ok in results.items():
        status = "✅ ОК" if ok else "❌ ОШИБКА"
        print(f"    Канал {ch + 1:2d}: {status}")
    print(f"\n    Итого: {passed}/{num_channels} каналов прошли проверку")

    return passed == num_channels


# ============================================================
#  ГЛАВНАЯ ФУНКЦИЯ
# ============================================================

def main():
    # Выберите адаптер:
    #   для реального PCA9685  → adapter = PCA9685Adapter(...)
    #   для отладки без железа → adapter = MockAdapter()

    USE_HARDWARE = True   # ← False для отладки без железа

    if USE_HARDWARE:
        adapter = PCA9685Adapter(
            num_channels=NUM_CHANNELS,
            i2c_bus=I2C_BUS,
            address=PCA_ADDRESS,
            frequency=PWM_FREQ,
        )
    else:
        adapter = MockAdapter()

    try:
        success = run_test(
            adapter=adapter,
            num_channels=NUM_CHANNELS,
            initial=INITIAL_PULSE,
            final=FINAL_PULSE,
            steps=STEPS,
            move_delay=MOVE_DELAY,
            switch_delay=SWITCH_DELAY,
        )
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n[!] Прервано пользователем (Ctrl+C)")
        sys.exit(130)
    finally:
        # Отключить все сервоприводы
        for ch in range(NUM_CHANNELS):
            try:
                adapter.release(ch)
            except Exception:
                pass
        print("\n[✓] Все сервы отключены.")


if __name__ == "__main__":
    main()
