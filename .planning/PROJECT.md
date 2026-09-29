# RobotDog 2.0

## What This Is

Четвероногий робот на Banana Pi BPI-M4 Zero (PCA9685, 12 × MG996R, IMU MPU6050) под ROS 2 Jazzy и Lyrical. Управляется геймпадом, клавиатурой и веб-пультом. Полностью написан и проверен в симуляции Gazebo; на настоящем роботе (сейчас есть корпус, сервы и IMU, без лидаров и других датчиков) код ещё ни разу не запускался. Проект владельца репозитория `hzname/robot-dog`, версия 2.0 написана с нуля, v1 лежит в `legacy/v1/`.

## Core Value

Робот надёжно и безопасно ходит по полу с геймпада, не падая и не убивая свои сервы.

## Requirements

### Validated

Выведено из кода и карты `.planning/codebase/`. Всё это работает в коде и в симуляции; на настоящем роботе ничего из списка не проверялось.

- ✓ Ходьба: вперёд и назад, шаг вбок, поворот на месте и их сочетания; встать, лечь, наклон и высота корпуса (рысь и ползком, `LocomotionController`) — existing
- ✓ Три пульта одновременно: геймпад USB/Bluetooth (Xbox, PlayStation), клавиатура в терминале (в том числе по SSH), веб-страница на `:8080` — existing
- ✓ Цепочка безопасности в софте: deadman, тайм-аут команд 0.4–0.5 с, E-STOP с любого пульта, ограничение скорости суставов, поочерёдное включение ног — existing
- ✓ Компенсация наклона по IMU и рельеф: уклоны, волны, камни, ступени (пределы измерены в Gazebo, `docs/TERRAIN.md`) — existing (simulation)
- ✓ Симуляция Gazebo с тем же кодом управления и автоматические проверки: `walk_check`, `terrain_sweep`, `perception_check`, `localization_check` — existing
- ✓ Восприятие и локализация по карте по лидарам, VL53L1X и GS2 — existing (simulation only, драйверов для железа нет)
- ✓ Калибровка серв: веб-канал, `tools/autocal` (камера и ArUco), `tools/robot_setup`; значения по умолчанию — оценка по v1 — existing (на железе не выполнялась)
- ✓ Сборка и развёртывание: Docker для Banana Pi (arm64) и для симуляции, CI на Jazzy и Lyrical — existing

### Active

Цель этапа: первый устойчивый ход по полу на настоящем роботе. Порядок задан владельцем: сначала симуляция, затем железо. Детализация и идентификаторы будут в `REQUIREMENTS.md`.

- [ ] Походка в симуляции доведена до состояния «можно ставить на пол»: слабые места (задний ход и спуск, см. `docs/TERRAIN.md`) проходят проверки в Gazebo
- [ ] Защита серв и корпуса до первого выхода на пол: ошибки записи по I2C не теряются (`servo_driver.cpp` сейчас игнорирует результат `setPulseUs`), есть watchdog на выход PWM, есть реакция на опрокидывание (сейчас показания IMU свыше 25° просто игнорируются)
- [ ] Калибровка серв на настоящем роботе по `docs/CALIBRATION.md` и запуск в Docker на Banana Pi
- [ ] Робот идёт по полу с геймпада стабильно, без падений и перегрева серв (критерий готовности этапа)

### Out of Scope

- Драйверы лидара, GS2 и VL53L1X и локализация по карте на железе — этих датчиков на роботе нет; вернуться, когда их установят
- Обход комнаты, подъём и спуск (склоны, ступени) на настоящем роботе — после устойчивого хода по ровному полу
- Работа с `legacy/v1/` и с неотслеживаемыми остатками v1 в корне репозитория (`robot_dog_ws/`, `robot_configurator.py`, `test_servo_config_reader.sh`) — это архитектура v1, она не относится к `ros2_ws/`
- Старые планы `.planning/phase-1-*` и `.planning/phases/phase-2-*` от мая 2026 — устарели, решено игнорировать

## Context

- **Железо сейчас:** есть корпус, 12 серв MG996R на PCA9685 (I2C 0x40) и IMU MPU6050. Нет лидаров, GS2 и VL53L1X. Датчик тока и напряжения (INA226/INA219) в коде опциональный, есть ли он на роботе, не подтверждено.
- **Платформа:** ROS 2 Jazzy (основная) и Lyrical, на роботе ROS работает внутри Docker (Armbian на Banana Pi, 2–4 ГБ ОЗУ); симуляция на ПК в Gazebo Sim.
- **Состояние кода:** активный код в `ros2_ws/src` (8 пакетов, все ноды в namespace `/dog`), вся геометрия и походка задаются в одном `robot.yaml`. Карта кода: `.planning/codebase/` (сделана 2026-09-29, проверена выборочно).
- **Известные слабые места:** задний ход слабее остальных манёвров, а на неровностях особенно (`docs/TERRAIN.md`); в отдельных конфигурациях на спуске робот опрокидывался.
- **Безопасность:** веб-пульт слушает `0.0.0.0:8080` без авторизации, калибровка через него разрешена по умолчанию. Входит ли авторизация в защитный этап, ещё не решено. В URL remote `origin` вшит токен GitHub (в `.git/config`): его стоит перевыпустить и перейти на SSH или credential helper.
- **Источник истины:** GitHub `hzname/robot-dog`, ветка `main`.

## Constraints

- **Tech stack**: C++17 и ROS 2 Jazzy вместе с Lyrical должны собираться без предупреждений (`-Wall -Wextra -Wpedantic`) — так проходит CI
- **Tech stack**: рантайм робота требует только `ros-base`, без xacro, joy и ros2_control — образ для Pi должен оставаться лёгким
- **Compatibility**: все ноды и топики в namespace `/dog`; параметры только через YAML (`robot.yaml`, `servos.yaml`) — это единый источник правды
- **Hardware**: Banana Pi с 2 ГБ ОЗУ, Docker-сборка идёт с `BUILD_JOBS=2`, одна шина I2C на PCA9685, IMU и датчик питания
- **Dependencies**: восприятие и локализация работают только в симуляции, пока на роботе нет датчиков
- **Safety**: на пол ставить робота только после защиты серв (см. решения ниже)

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Симуляция раньше железа | Выбор владельца: походку дешевле дорабатывать в Gazebo, к тому же датчиков на роботе нет | — Pending |
| Защита серв до первого выхода на пол | Предложено при постановке цели (I2C-ошибки не обрабатываются, PWM без watchdog), владелец не возражал | — Pending |
| Датчики на железе отложены | Лидаров, GS2 и VL53L1X на роботе нет | — Pending |
| Старые `phase-1`/`phase-2` игнорировать | Описывают v1 (`servo_config.json`, `calibrate_web :8081`), не совпадают с `ros2_ws/` | — Pending |

## Evolution

This document evolves at phase transitions and milestone boundaries.

**After each phase transition** (via `/gsd-transition`):
1. Requirements invalidated? → Move to Out of Scope with reason
2. Requirements validated? → Move to Validated with phase reference
3. New requirements emerged? → Add to Active
4. Decisions to log? → Add to Key Decisions
5. "What This Is" still accurate? → Update if drifted

**After each milestone** (via `/gsd-complete-milestone`):
1. Full review of all sections
2. Core Value check — still the right priority?
3. Audit Out of Scope — reasons still valid?
4. Update Context with current state

---
*Last updated: 2026-09-29 after initialization*
