# 06-02: запись bag одной командой — сводка волны

Код `3e05e47`. Проверено: `pytest ros2_ws/src/dog_bringup/test/test_record.py` — 5 passed;
шаг `ament_add_pytest_test(test_record ...)` проходит в `build + test`.

## Что сделано

1. `ros2_ws/src/dog_bringup/launch/record.launch.py` — `ros2 bag record` с фиксированным
   списком `TOPICS` (14 топиков), zstd, каталог `~/dog-bags/<дата-время>`, переопределяется
   аргументом `out`.
2. Состав списка: состояние и команды (`state`, `command`, `estop`, `cmd_vel`), суставы
   (`joint_states`, `joint_commands`, `servo_pulses`), `guard`, `imu/data`, `power`, `joy`,
   `body_pose`, `profile`, `terrain/profile`.
3. `test_record.py` — список плоский, все имена с `/dog/`, **каждое имя встречается в
   исходниках** (переименование топика ломает тест), набор для разбора падения на месте,
   каталог на запуск уникален.

## Чего в списке нет и почему

- `/dog/odom` — локомоушен публикует его только при `odom.publish`, в обычном прогоне на
  роботе топика может не быть; для разбора падения хватает `joint_commands` и `servo_pulses`.
- `localization/*` и `perception/*` — зависят от конфигурации запуска; при необходимости
  добавляются одной строкой в `TOPICS`, тест сразу проверит имена.

## Проверки

| Требование | Где | Чем проверено |
|---|---|---|
| CAL-10 | `launch/record.launch.py`, `test/test_record.py`, `CMakeLists.txt` | 5 тестов + сборка (`ament_add_pytest_test`) + быстрая python-джоба |
