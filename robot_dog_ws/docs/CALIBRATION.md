# Калибровка сервоприводов

## Содержание

1. [Подготовка](#подготовка)
2. [Нулевая позиция](#нулевая-позиция)
3. [Настройка диапазонов](#настройка-диапазонов)
4. [Инверсия](#инверсия)
5. [Проверка](#проверка)

---

## Подготовка

### Файл конфигурации

Калибровка хранится в `src/dog_control/config/servo_config.yaml`:

```yaml
servo_config:
  # Левая передняя нога
  left_front:
    hip:
      channel: 0
      min_pulse: 500    # мкс
      max_pulse: 2500   # мкс
      zero_offset: 0    # градусы
      inverted: false
    thigh:
      channel: 1
      min_pulse: 500
      max_pulse: 2500
      zero_offset: 0
      inverted: false
    calf:
      channel: 2
      min_pulse: 500
      max_pulse: 2500
      zero_offset: 0
      inverted: false
  
  # Правая передняя нога
  right_front:
    hip:
      channel: 3
      min_pulse: 500
      max_pulse: 2500
      zero_offset: 0
      inverted: true    # Инвертирована!
    thigh:
      channel: 4
      min_pulse: 500
      max_pulse: 2500
      zero_offset: 0
      inverted: true
    calf:
      channel: 5
      min_pulse: 500
      max_pulse: 2500
      zero_offset: 0
      inverted: true
  
  # Левая задняя нога
  left_rear:
    hip:
      channel: 6
      min_pulse: 500
      max_pulse: 2500
      zero_offset: 0
      inverted: false
    thigh:
      channel: 7
      min_pulse: 500
      max_pulse: 2500
      zero_offset: 0
      inverted: false
    calf:
      channel: 8
      min_pulse: 500
      max_pulse: 2500
      zero_offset: 0
      inverted: false
  
  # Правая задняя нога
  right_rear:
    hip:
      channel: 9
      min_pulse: 500
      max_pulse: 2500
      zero_offset: 0
      inverted: true
    thigh:
      channel: 10
      min_pulse: 500
      max_pulse: 2500
      zero_offset: 0
      inverted: true
    calf:
      channel: 11
      min_pulse: 500
      max_pulse: 2500
      zero_offset: 0
      inverted: true
```

---

## Нулевая позиция

### Что такое нулевая позиция?

Нулевая позиция — это положение, в котором все суставы установлены в 0°. Для робота-собаки это:

```
        [Корпус]
           │
    ┌──────┴──────┐
    │             │
   ╱│╲           ╱│╲
    │             │
   ╱             ╱
  
  (все ноги под углом 90° относительно корпуса)
```

### Процедура калибровки

1. **Запустите скрипт калибровки:**

```bash
cd ~/robot_dog_ws
source install/setup.bash
ros2 run dog_control calibrate_servos
```

2. **Установите все сервоприводы в 90° (среднее положение)**

3. **Визуально проверьте:**
   - Бедро параллельно корпусу
   - Голень перпендикулярна земле
   - Симметрия между ногами

4. **Запишите отклонения:**

Если сустав отклонён от идеальной позиции, запишите угол отклонения в `zero_offset`.

Пример:
- Если левое бедро отклонено на +5°, установите `zero_offset: -5`
- Если правое бедро отклонено на -3°, установите `zero_offset: 3`

---

## Настройка диапазонов

### Определение min/max

Для каждого сустава нужно определить физические ограничения:

```bash
# Запустите тестовый скрипт
ros2 run dog_control test_servo_limits --servo 0
```

### Процедура:

1. **Медленно увеличивайте угол**, пока не появится:
   - Механический упор
   - Неестественное звук сервопривода
   
2. **Запишите значение pulse** как `max_pulse`

3. **Медленно уменьшайте угол**, найдите `min_pulse`

4. **Оставьте запас 5-10%** для безопасности

### Типичные значения

| Сустав | min_pulse | max_pulse | min_angle | max_angle |
|--------|-----------|-----------|-----------|-----------|
| Hip    | 500       | 2500      | -45°      | +45°      |
| Thigh  | 600       | 2400      | -60°      | +60°      |
| Calf   | 500       | 2500      | -90°      | +90°      |

---

## Инверсия

### Когда нужна инверсия?

Правая сторона робота (right_front, right_rear) обычно **инвертирована**, так как сервоприводы установлены зеркально.

### Правило:

```
Левая сторона:  inverted: false
Правая сторона: inverted: true
```

### Проверка:

```bash
# Отправьте команду +30° на обе передние бедра
# Они должны двигаться симметрично (вперёд от корпуса)
```

Если одна нога движется вперёд, а другая назад — инверсия настроена неправильно.

---

## Проверка

### Тест стойки

```bash
# Установка в позу стойки
ros2 service call /dog_control/stand std_srvs/srv/Trigger
```

### Тест походки

```bash
# Тестовая походка
ros2 service call /dog_control/walk std_srvs/srv/Trigger
```

### Визуальная проверка

✅ **Правильно:**
- Робот стоит ровно
- Все ноги касаются земли
- Корпус горизонтален
- Движения симметричны

❌ **Неправильно:**
- Робот наклонён
- Одна нога не касается земли
- Движения рывками
- Асимметрия при походке

---

## Сохранение калибровки

После настройки пересоберите проект:

```bash
cd ~/robot_dog_ws
colcon build --symlink-install
source install/setup.bash
```

Калибровка загрузится автоматически при старте контроллера.
