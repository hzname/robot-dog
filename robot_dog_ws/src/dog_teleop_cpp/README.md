# dog_teleop_cpp

C++ телеоперационные ноды для робота-собаки с поддержкой lifecycle и системой безопасности.

## Ноды

### 1. keyboard_teleop

Управление с клавиатуры.

**Управление:**
- `W/S` - Вперед/Назад
- `A/D` - Влево/Вправо (страф)
- `Q/E` - Поворот
- `Пробел` - Остановка
- `+/-` - Регулировка высоты
- `ESC` - Аварийная остановка
- `R` - Сброс аварийной остановки
- `Ctrl+C` - Выход

**Запуск:**
```bash
ros2 launch dog_teleop_cpp teleop_keyboard.launch.py
```

### 2. gamepad_teleop

Управление с геймпада (Xbox-совместимый).

**Управление:**
- Левый стик - Движение вперед/назад, страф
- Правый стик X - Поворот
- LT/RT - Регулировка высоты
- A - Встать, B - Сесть
- X - Аварийная остановка, Y - Сброс
- Start - Включить сервоприводы, Back - Выключить
- LB/RB - Изменение скорости походки

**Запуск:**
```bash
ros2 launch dog_teleop_cpp teleop_joystick.launch.py
```

### 3. udp_teleop

Управление через UDP (для внешних приложений).

**Протокол:**
- Формат: `vx,vy,wz,stop` (comma-separated)
- Порт: 8888 (настраивается)
- Команды: `S` - стоп, `E` - аварийная остановка, `R` - сброс

**Запуск:**
```bash
ros2 launch dog_teleop_cpp teleop_udp.launch.py
```

**Пример отправки команд:**
```bash
# Движение вперед
echo "0.5,0.0,0.0,0" | nc -u localhost 8888

# Остановка
echo "S" | nc -u localhost 8888

# Аварийная остановка
echo "E" | nc -u localhost 8888
```

## Общий запуск

Запуск всех режимов одновременно:
```bash
ros2 launch dog_teleop_cpp teleop_all.launch.py \
  enable_keyboard:=true \
  enable_joystick:=false \
  enable_udp:=false
```

## Безопасность

Все ноды включают:

1. **Lifecycle Node** - корректное управление жизненным циклом
2. **Watchdog** - остановка при отсутствии команд (timeout: 0.5s)
3. **Emergency Stop** - немедленная остановка (публикует в `/emergency_stop_trigger`)
4. **Публикация нулевой скорости** - при деактивации и ошибках

## Топики

**Выход:**
- `/cmd_vel` (geometry_msgs/Twist) - Команды скорости
- `/cmd_height` (std_msgs/Float64) - Целевая высота
- `/emergency_stop_trigger` (std_msgs/Bool) - Статус аварийной остановки
- `/servo_enable` (std_msgs/Bool) - Управление сервоприводами (только gamepad)

**Вход (gamepad):**
- `/joy` (sensor_msgs/Joy) - Данные с геймпада

## Параметры

Общие параметры для всех нод:
- `max_linear_speed` - Максимальная линейная скорость (м/с), default: 0.5
- `max_angular_speed` - Максимальная угловая скорость (рад/с), default: 2.0
- `max_lateral_speed` - Максимальная боковая скорость (м/с), default: 0.3
- `publish_rate` - Частота публикации (Гц), default: 50.0
- `watchdog_timeout` - Таймаут watchdog (с), default: 0.5

Специфичные:
- `axis_deadzone` (gamepad) - Мертвая зона осей, default: 0.1
- `udp_port` (udp) - UDP порт, default: 8888
- `max_packet_size` (udp) - Макс. размер пакета, default: 256

## Сборка

```bash
cd ~/robot-dog/robot_dog_ws
colcon build --packages-select dog_teleop_cpp
source install/setup.bash
```

## Зависимости

- rclcpp
- rclcpp_lifecycle
- rclcpp_components
- geometry_msgs
- sensor_msgs
- std_msgs
- joy

## Лицензия

MIT
