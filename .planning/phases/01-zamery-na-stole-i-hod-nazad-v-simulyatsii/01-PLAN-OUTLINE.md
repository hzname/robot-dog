# Фаза 1 — Outline планов (chunked mode)

Контракт между прогонами: каждый план пишется отдельным коротким прогоном и коммитится отдельно. Имена, ключи, флаги и пути ниже зафиксированы здесь и в планах заново не выбираются. Проза на русском; идентификаторы, пути, команды и ключи YAML на английском.

## Таблица планов

| Plan ID | Objective | Wave | Depends On | Requirements |
|---------|-----------|------|------------|--------------|
| 01-01 | Ветка фазы `gsd/phase-1-…` от `main`, помощники `tools/ci_dispatch/ci_dispatch.sh` (запуск CI по `workflow_dispatch`, ожидание, артефакты) и `tools/local_gtest/run.sh` (локальный gtest ядра без colcon). Уже написан и принят, не менять | 1 | none | GAIT-02, GAIT-06 |
| 01-02 | Контракт конфигурации: новые ключи `robot.yaml` (`servo.*`, `servo_sim.*`, `gait.auto_period`, `gait.min_period`, `description.body_com_x`), инвариант трёх скоростей; `robot_setup`: поле `body_com_x`, порт расчёта `knee_ratio`, ошибки несуществующих датчиков не блокируют сохранение | 2 | 01-01 | CAL-16, CAL-17, GAIT-10 |
| 01-03 | Чистая статистика приёмки `acceptance_stats` (TDD): классификация повторов, минимум и медиана, правила A/B/Lyrical/неровности, схема JSON 1, `derive_push_threshold` | 2 | 01-01 | GAIT-01, GAIT-02 |
| 01-04 | Ядро `dog_control/servo_limits` (TDD): `peakServoSpeed` и `minimalPeriod` (перебор с окном, нижняя граница 0.55 с) | 2 | 01-01 | CAL-17, GAIT-06 |
| 01-05 | Новый пакет `dog_bench`, трассер режима `selftest`: шина I2C с подменой для тестов, быстрый INA219 (0x199F), проверка частоты опроса | 2 | 01-01 | CAL-17 |
| 01-06 | `tools/servo_speed`: анализ токовых следов (излом расстояния между следами и кросс-проверка по длительности), синтетический генератор, README | 2 | 01-01 | CAL-17 |
| 01-07 | Документы владельца и процесса: лист замеров, список заказа деталей, запись о зажиме скорости драйвера в `docs/REVIEW.md`, правка «8/8» → «10/10», разделы SIMULATION/DEPLOYMENT | 2 | 01-01 | CAL-16, CAL-17, GAIT-06, GAIT-10 |
| 01-08 | Автопериод: период из скорости серв в `LocomotionController`, пересчёт при смене параметров (D-12 дословно), проводка в `locomotion_node`, новые ворота `JointSpeedsFitTheServos` | 3 | 01-02, 01-04 | CAL-17, GAIT-06 |
| 01-09 | `dog_bench`: рампа скоростей, предохранители (ток, насыщение, правдоподобие шунта), PWM с гарантированным отпусканием и проверкой «чип свободен» | 3 | 01-05 | CAL-17 |
| 01-10 | Реалистичный профиль серв (`servo_model:=real`): `servo_profile.py`, `urdf.py` (трение, множители напряжения, `body_com_x`), мост (люфт, лаг), `sim.launch.py` | 3 | 01-02 | CAL-16, GAIT-10 |
| 01-11 | `walk_check`: `--maneuvers`, `--backward-speed`, `dyaw5_deg`; CLI `acceptance` (повторы со свежей симуляцией, ячейки D-03) | 3 | 01-03 | GAIT-01, GAIT-02, GAIT-06 |
| 01-12 | `dog_bench`: сессия замера (`Session`), режимы `dry-run` и `run`, CSV и `.meta.json`, README запуска на роботе, проверка сборки в CI | 4 | 01-09, 01-06 | CAL-17 |
| 01-13 | `ci.yml`: входы `workflow_dispatch`, job `acceptance`, job `servo-speed`, пробный запуск (`repeats=1`), базовый прогон «до» | 5 | 01-11, 01-10, 01-12, 01-08, 01-06, 01-03 | CAL-17, GAIT-01, GAIT-02, GAIT-06, GAIT-10 |
| 01-14 | Владелец: замер скорости MG996R (проверка питания, шунт, транспортир, `selftest`, `run`), анализ, утверждение скорости и доли ворот | 6 | 01-13, 01-12, 01-06, 01-07 | CAL-17 |
| 01-15 | Ввод измеренных чисел (D-26): геометрия и массы (чекпоинт владельца), три скорости, `auto_period: true`, синхронизация умолчаний кода | 7 | 01-14, 01-13, 01-07, 01-02 | CAL-16, CAL-17, GAIT-06 |
| 01-16 | Правки походки (условно, только если данные после 01-15 не дают ≥ 40 % на Jazzy): форма траектории переноса без общего подъёма `step_height` | 8 | 01-15 | GAIT-01, GAIT-06 |
| 01-17 | Итоговая приёмка (4 элемента, ≥ 5 повторов), порог `--backward-ratio` по дистрибутивам (последняя правка кода фазы), итоги в `docs/TERRAIN.md` | 9 | 01-16, 01-13 | GAIT-01, GAIT-02, GAIT-06, GAIT-10 |

Итого: 17 планов, 9 волн (W1…W9). Соответствие волнам RESEARCH Pattern 7: W0 = волны 2-4 здесь (владелец-независимая работа), W1 = 01-13, CP-A/B/C = 01-14 и первая задача 01-15, W2 = 01-15, W3 = 01-16, W4 = 01-17.

## Plan details

Общие соглашения для всех планов:
- Ветка `gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii`; всё выполняется на ней (создаёт 01-01).
- Локальные команды проверки (из корня репозитория): `python3 -m pytest -q tools/robot_setup/test`; `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_description/test`; `PYTHONPATH=ros2_ws/src/dog_gazebo:ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_gazebo/test`; `python3 -m pytest -q tools/servo_speed/tests`; `bash tools/local_gtest/run.sh <dog_control|dog_bench> <test> [--gtest_filter=…]`.
- CI-only: только через `bash tools/ci_dispatch/ci_dispatch.sh --ref gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii …` после `git push` ветки (вывод push пропускать через sed, адрес `origin` не печатать). Запуск `ci.yml` прогоняет все его job; ждать только нужные через `--wait-job`.
- Каждый `<automated>` имеет `<fails_when>` (protected block); в `<verify>` не использовать идентификаторы прошлых запусков CI.
- Каждый план содержит `<threat_model>` (ASVS 1, блок на high) и раздел «Artifacts this phase produces».
- `ros2_ws/src/dog_hardware/**` — только чтение (аналоги), правки запрещены (параллельная Phase 3).

### 01-01 — Ветка фазы и помощники (уже написан)

- Поток: docs+process (инфраструктура). Трассера нет: план создаёт ветку и два скрипта-помощника, сквозного пути через слои фазы здесь нет.
- Файлы: `tools/ci_dispatch/ci_dispatch.sh` (new), `tools/local_gtest/run.sh` (new); git-ветка (файлы не меняются).
- Имена-контракт: `ci_dispatch.sh` флаги `--ref`, `-f key=value` (повторяемый), `--wait-job SUBSTR` (повторяемый), `--download DIR`, `--run-id ID`, `--timeout-min`, `--poll-sec`, `--dry-run`; печатает `RUN_ID=<id>`; коды выхода 0 (все ждавшиеся `success`), 1, 2 (неверные аргументы), 3 (таймаут); значения `-f` по шаблону `^[a-z_][a-z0-9_]*=[A-Za-z0-9._,:-]*$`. `run.sh <package> <test> [gtest args]`, пакеты только `dog_control` и `dog_bench`, бинарь в `ros2_ws/build/_local/<package>/test_<test>`, из исходников исключаются `*_node.cpp` и `*_main.cpp`, сборка с `-Wall -Wextra -Wpedantic -Werror`.
- Следствие для других планов: исполняемый файл CLI в `dog_bench` называется `src/servo_speed_test_main.cpp` (иначе `run.sh` слинкует чужой `main`), тест `test_<name>.cpp` лежит в `test/` пакета.
- Решения: D-24, D-25, D-04.
- Чекпоинты владельца: нет. CI-only: нет (живая проверка только читает последний завершённый запуск `ci.yml`, идентификатор берётся из `gh run list`).
- Задач: 3 (как написано). Оркестратор убрал из `<verify>` задачи 2 зашитый идентификатор прошлого запуска CI: вместо него сухой прогон `--dry-run --run-id` и динамический выбор последнего завершённого запуска.

### 01-02 — Контракт конфигурации и `robot_setup` (W2)

- Поток: S1 замеры и конфиг (с ключами для S2 и S3). Трассер (задача 2): `body_com_x` проходит путь `robot.yaml` → `robot_setup` (load, validate, save) → те же комментарии и раскладка.
- Файлы: `ros2_ws/src/dog_bringup/config/robot.yaml`; `tools/robot_setup/robot_setup.py`; `tools/robot_setup/test/test_robot_setup.py`; `tools/robot_setup/test/test_yaml_contract.py` (new).
- Имена-контракт (все новые ключи `robot.yaml` добавляются ЗДЕСЬ и только здесь, значения сохраняют нынешнее поведение; формат `key: value  # comment`, один ключ на строку):
  - `gait.auto_period: false`, `gait.min_period: 0.55` (с): период не короче проверенного, D-12.
  - блок `servo:` с `max_speed: 6.0` (рад/с, «предполагаемая» скорость, по ней считается период), `margin: 0.8` (ворота D-11), `knee_ratio: 1.0` (серва-рад на сустав-рад колена, D-20).
  - блок `servo_sim:` с `backlash_deg: 1.5`, `delay_ms: 40.0`, `friction_nm: 0.06`, `bus_voltage: 6.0`, `bus_voltage_ref: 6.0`; у каждого числа комментарий-источник (D-16: середина 1–2° из `docs/TERRAIN.md`; середина 30–50 мс из GAIT-05; трение ≈ 5 % момента заклинивания 1.079 Н·м при токе холостого хода 0.15 А, оценка; 6.0 В без просадки).
  - `description.body_com_x: 0.0  # [m] …` (D-21, `0` = центр корпуса).
  - комментарий `sim_p_gain`: параметр не используется в режиме velocity commands; значение 25.0 не менять.
- `robot_setup.py`: поле формы `body_com_x` (группа `body`, мм, диапазон −60…60, scale 1000), предупреждение в `validate` при `|com_x| > hip_x`; `sensor_checks` понижает `error` до `warn` для лидаров, GS2 и ToF, если соответствующие флаги `sensors.*` равны 0 (иначе смена `stand_height` блокирует `save` и `--check`); функция `knee_ratio_max(servo_arm_mm, joint_arm_mm, rod_mm, axis_distance_mm, lo_deg=-105.7, hi_deg=-80.5)` (порт четырёхшарнирника из `tools/autocal/robotdog_autocal/servo_model.py`, золотые значения C++: 1.605 при −30°, 1.427 при −20°, 1.357 при −10°, 1.334 при 0°, 1.344 при +10°, 1.392 при +20°, 1.507 при +30°; максимум в рабочем диапазоне ≈ 1.40) и предупреждение `--check`, если `servo.knee_ratio` меньше вычисленного.
- `test_yaml_contract.py` (new, ловит его существующий job `robot-setup`): все ключи выше присутствуют; инвариант D-13 «`description.servo_velocity` == `servo.max_speed` == `max_joint_speed` в `servos.yaml`»; `0 < servo.margin ≤ 1`; `servo_sim.*` числа.
- Решения: D-13, D-16, D-18, D-20, D-21.
- Чекпоинты владельца: нет. CI-only: нет (pytest `tools/robot_setup/test`; ключи без эффекта на поведение, `auto_period: false`).
- Задач: 4 (ключи и контракт-тест; `body_com_x` в форме; `sensor_checks`; `knee_ratio`).

### 01-03 — `acceptance_stats` (W2, TDD)

- Поток: S4 статистическая приёмка. Трассер: сквозной путь на синтетическом JSON — повторы → `summarize_cell` → вердикт → `push_threshold` (pytest, без симуляции).
- Файлы: `ros2_ws/src/dog_gazebo/dog_gazebo/acceptance_stats.py` (new); `ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py` (new); `ros2_ws/src/dog_gazebo/setup.py` (только `extras_require={'test': ['pytest']}`); `ros2_ws/src/dog_gazebo/package.xml` (`<test_depend>python3-pytest</test_depend>`).
- Имена-контракт (чистый модуль, без rclpy и numpy, Python 3.12 и 3.14):
  - `SCHEMA_VERSION = 1`, `MIN_REPEATS = 5`, `MIN_BODY_HEIGHT = 0.108` (комментарий: равно `walk_check.MIN_BODY_HEIGHT`), `MAX_TILT_DEG = 20.0`, `MAX_DYAW5_DEG = 10.0`, `MIN_RATIO = 0.40`.
  - `CELLS`: словарь имён ячеек D-03: `flat_A_bwd10` (flat, режим A, −0.10, манёвры `backward,left,right`), `flat_B_bwd10` (flat, режим B: `heading_hold:=false slope_compensation:=false`, −0.10, `backward,left,right`), `flat_B_bwd05` (режим B, −0.05, `backward`, только отчёт), `waves10_A_bwd10` (terrain `waves`, level 10, `backward`), `rocks10_A_bwd10` (terrain `rough`, level 10, `backward`, `seed` = номер повтора). Поля ячейки: `terrain`, `level`, `mode`, `cmd_vx`, `maneuvers`, `rule` (`score_a | score_b | report | terrain`).
  - `classify_run(data) -> 'ok' | 'fell' | 'no_stand' | 'error'` (`fell` — есть `fallen=True`; `no_stand` — `stand` провален с `state=passive`, как `terrain_sweep.never_stood`; `error` — пустой JSON или таймаут).
  - `summarize_cell(cell, runs, distro) -> dict` с ключами `n`, `n_invalid`, `ratio_min`, `ratio_median`, `falls`, `max_abs_dyaw5_deg`, `pass`, `reasons`; `pass` равен `None` (недостаточно данных) при числе валидных повторов меньше `MIN_REPEATS`, такой вердикт не считается зачётным.
  - Правила (D-01, D-06, D-07), Jazzy = зачёт: A: `falls == 0`, `ratio_min ≥ 0.40`, `max|dyaw5| ≤ 10°` по `backward`, `left`, `right`; B −0.10: `falls == 0`, `ratio_min ≥ 0.40`, увод в отчёт; B −0.05: только отчёт; `waves10`/`rocks10`: `falls == 0`, `tilt_deg < 20`, `z > 0.108` на каждом повторе, ratio в отчёт. Lyrical: `falls == 0` и `ratio_min ≥ floor_ratio` (по умолчанию 0.2) на плоских ячейках, неровности как у Jazzy; 40 % не требуется.
  - `derive_push_threshold(min_ratio, lo=0.2, hi=0.4, step=0.05, margin=0.8) -> float` (через `math.floor(round(margin * min_ratio / step, 9)) * step`, затем зажим; 0.52 → 0.40, 0.45 → 0.35, 0.30 → 0.20); `push_threshold_for(result) -> dict | None` (только `servo_model == 'ideal'`, ячейка `flat_A_bwd10`, валидных повторов ≥ 5).
  - `build_result(distro, servo_model, repeats, git_sha, cells_runs, floor_ratio=0.2) -> dict` по схеме 1 из RESEARCH Pattern 3: ключи `schema`, `distro`, `servo_model`, `repeats`, `git_sha`, `cells` (у ячейки `terrain`, `level`, `heading_hold`, `cmd_vx`, `runs`, `summary`), `scoring`, `verdict`, `push_threshold`; в `runs[]`: `status`, `ratio` (манёвр `backward`), `ratios` (все манёвры), `dyaw5_deg` (словарь по манёврам), `tilt_deg`, `z`, `wall_s`. `render_summary(result) -> str` (markdown для `$GITHUB_STEP_SUMMARY`).
  - `main()` для `python3 -m dog_gazebo.acceptance_stats threshold RESULT.json [...]`: печатает `distro=<d> min_ratio=<m> push_threshold=<v>` (используется в 01-17).
- Решения: D-01, D-02, D-03, D-05, D-06, D-07.
- Чекпоинты владельца: нет. CI-only: нет (весь модуль тестируется pytest; запуск как первый шаг job `acceptance` в 01-13).
- Задач: 3 (ядро статистики и правила; схема JSON, порог, сводка; `setup.py`, `package.xml`, CLI `threshold`).

### 01-04 — `servo_limits` (W2, TDD)

- Поток: S2 скорость серв и период. Трассер (задача 1): `peakServoSpeed` на `LocomotionParams` по умолчанию даёт 5.077 рад/с (колено, команда `{0.15, 0.08, 0.6}`) — одна функция, один путь через `TrotGait` и `inverseKinematics`.
- Файлы: `ros2_ws/src/dog_control/include/dog_control/servo_limits.hpp` (new); `ros2_ws/src/dog_control/src/servo_limits.cpp` (new); `ros2_ws/src/dog_control/test/test_servo_limits.cpp` (new); `ros2_ws/src/dog_control/CMakeLists.txt` (строка в `add_library(dog_control_core …)` и имя `servo_limits` в `foreach(t …)`).
- Имена-контракт (namespace `dog_control`): `struct ServoSpeedModel {double max_speed{6.0}; double margin{0.8}; double knee_ratio{1.0};}`; `struct PeakSpeed {double peak{0.0}; int joint_kind{0};}` (0 hip, 1 thigh, 2 knee); `PeakSpeed peakServoSpeed(const LocomotionParams & p, const ServoSpeedModel & s)`; `double minimalPeriod(const LocomotionParams & p, const ServoSpeedModel & s, double min_period, double max_period)` (возвращает `0.0`, если ничего не подходит); `constexpr double kMaxAutoPeriod{1.5}` (константа ядра, НЕ ключ YAML), `kPeriodScanStep{0.0025}`, `kPeriodGuard{0.05}`, допуск сравнений `1e-9`. Заголовок делает forward-declaration `struct LocomotionParams;` и НЕ включает `locomotion.hpp` (чтобы `locomotion.hpp` в 01-08 мог включить `servo_limits.hpp`, циклов нет); `servo_limits.cpp` включает `locomotion.hpp`.
- Таблица для gtest (duty 0.65, шаг 20 мм, `max_step` 0.06, dt 0.02, ворота 80 %): 3.5 → 1.030 с, 4.0 → 0.8975 с, 5.0 → 0.720 с, 6.0 → 0.600 с, 6.35 → 0.550 с, 7.0 → 0.550 с (нижняя граница, период не укорачивается, D-12). Имена тестов: `ServoLimits.PeakAtShippedGaitIs5p077`, `ServoLimits.MinimalPeriodTable`, `ServoLimits.PeriodNeverBelowMinimum`, `ServoLimits.NoFitReturnsZero`, `ServoLimits.PeakMatchesController` (TrotGait и контроллер совпадают до 3 знаков).
- Решения: D-11, D-12, D-13 (предполагаемая скорость), D-20 (пик в пространстве серв: `max(hip, thigh, knee_ratio * calf)`).
- Чекпоинты владельца: нет. CI-only: нет (локально `bash tools/local_gtest/run.sh dog_control servo_limits`, затем остальные четыре теста пакета); сборка под GCC 13/15 проверяется в 01-08.
- Задач: 3 (пик; минимальный период с окном; регистрация в CMake и сверка с контроллером).

### 01-05 — `dog_bench`: трассер `selftest` (W2)

- Поток: S2, инструмент замера. Трассер: CLI `servo_speed_test selftest` (только INA219, PWM не включается) проходит все слои: `LinuxI2cBus` → `Ina219Fast` → `runSelftest` → вердикт и код выхода; на машине без железа проверяются отказ по аргументам (код 2) и путь на `FakeI2cBus` (gtest).
- Файлы (все new): `ros2_ws/src/dog_bench/package.xml` (format 3, без `rclcpp` и без `dog_hardware`); `ros2_ws/src/dog_bench/CMakeLists.txt` (`dog_bench_core`, `servo_speed_test`, `foreach(t …)`); `include/dog_bench/i2c_bus.hpp`, `src/i2c_bus.cpp`; `include/dog_bench/ina219_fast.hpp`, `src/ina219_fast.cpp`; `include/dog_bench/selftest.hpp`, `src/selftest.cpp`; `src/servo_speed_test_main.cpp`; `test/test_ina219_fast.cpp`, `test/test_selftest.cpp`.
- Имена-контракт (namespace `dog_bench`): `I2cBus` (абстрактный; `read16`, `write16`, чтение без записи указателя, запись массива), `LinuxI2cBus(device)` (`ioctl(I2C_RDWR)`, адрес в каждом сообщении, без `I2C_SLAVE`), `FakeI2cBus`; `ina::kIna219Fast320mv = 0x199F` (шунт 0.1 Ом, ±3.2 А), `ina::kIna219Fast80mv = 0x0999` (10 мОм, ±8 А), `Ina219Fast` (`configure`, чтение шунта и шины, `currentA(raw, shunt_ohm)`), `SelftestResult {median_ms, p99_ms, max_ms, errors, ok, reason}`, `runSelftest(…)` с подставляемыми часами (детерминизм); порог: отказ при медиане > 1.5 мс или p99 > 5 мс (совет про 400 кГц, `docs/REVIEW.md:27`). Адрес 0x40 никогда не зондируется как INA; `--ina-address` и `--shunt-ohm` обязательны, без значений по умолчанию. CLI: режимы `selftest | dry-run | run`, код 2 при неверных аргументах, 1 при ошибке времени выполнения, 3 при аварийном отпускании; `dry-run` и `run` здесь отвечают сообщением и кодом 2 до 01-12. Имена тестов: `Ina219Fast.ConfigRegisterValues`, `Ina219Fast.ShuntScaling`, `Selftest.AcceptsFastBus`, `Selftest.RejectsSlowMedian`, `Selftest.RejectsSlowP99`, `Selftest.CountsConsecutiveErrors`. Тесты в `foreach(t ina219_fast selftest)`.
- Решения: D-08 (уровень измерений INA), D-09 (INA219, не INA226), D-10 (порог ниже предела INA219, PGA по шунту), Claude's Discretion (частота опроса ≥ 500 Гц).
- Чекпоинты владельца: нет (владелец запускает `selftest` позже, в 01-14). CI-only: сборка и gtest пакета под GCC 13/15 (проверяются в 01-12).
- Задач: 3 (скелет пакета, шина и INA; `selftest`; CLI `selftest` с валидацией аргументов, локальная сборка g++ и проверка кода 2).

### 01-06 — `tools/servo_speed` (W2, TDD)

- Поток: S2, анализ токовых следов. Трассер: `synth_run(v_max=6.0)` → `analyze` → `v_sat` в пределах одной ступени сетки.
- Файлы (все new): `tools/servo_speed/analyze.py`; `tools/servo_speed/synth.py`; `tools/servo_speed/tests/conftest.py`; `tools/servo_speed/tests/test_analyze.py`; `tools/servo_speed/requirements.txt` (`numpy>=1.24`); `tools/servo_speed/README.md` (русский; раздел анализа, раздел запуска на роботе дописывает 01-12).
- Имена-контракт: входной CSV `t_s, stroke_id, direction, cmd_us, shunt_raw, bus_raw`; файл метаданных `<csv>.meta.json` с полями `shunt_ohm`, `us_per_deg`, `amp_deg`, `center_us`, `channel`, `speeds_rad_s`, `strokes_per_speed`, `hold_s`, `rest_s`, `ina_config`, `stop_reason`; CLI `python3 tools/servo_speed/analyze.py --csv FILE [--meta FILE] [--out result.json] [--plot out.png]`; выход JSON: `v_sat_rad_s`, `v_sat_status` (`ok | not_saturated`), `v_dur_plateau_rad_s`, `agreement` (bool, расхождение ≤ 15 %), `servo_max_speed_rad_s` (= `v_sat`, если согласие, иначе меньшее из двух, плюс `flag`), `plateau_current_a`, `noise_floor`, `bus_v_mean`, `per_direction`. Порог `ε = max(3·σ_rep, 0.02·I_plateau)` (относительные единицы, не зависит от шунта), `v_sat` — наименьшая скорость, после которой не меньше трёх подряд пар ниже `ε`; нет такой — «не насыщено», не число. Функции `analyze(rows, meta) -> dict`, `find_saturation(speeds, distances, eps)`, `synth_run(v_max, seed=0, noise_a=0.008, shunt_scale=1.0, …) -> (rows, meta)`. Тесты: `v_max ∈ {3.5, 4.5, 6.0, 7.5}`, шум 8–30 мА, инвариантность к масштабу шунта (×10), сетка без насыщения, согласие `v_dur` и `v_sat`, рассинхрон старта 0–20 мс.
- Решения: D-08, D-09.
- Чекпоинты владельца: нет. CI-only: нет (запуск pytest как job `servo-speed` добавляет 01-13).
- Задач: 3 (ядро излома; кросс-проверка, согласие и инвариантность; README и CLI).

### 01-07 — Документы владельца и процесса (W2)

- Поток: docs+process, S1. Трассера нет: документация.
- Файлы: `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-MEASUREMENT-SHEET.md` (new); `docs/PARTS_ORDER.md` (new); `docs/REVIEW.md`; `docs/SIMULATION.md`; `docs/DEPLOYMENT.md`; `docs/TERRAIN.md`; `README.md`.
- Состав: (1) лист замеров часть 1: таблица «поле / единица / как мерить / ключ YAML / схема из `docs/img/`» по RESEARCH Pattern 5 (`thigh`, `calf`, `hip_offset`, `knee_direction`, `hip_x`, `hip_y`, размеры корпуса, `foot_radius`, четыре массы и `total_mass`, пределы суставов минус 5°, четыре длины тяги колена, `body_com_x` ±3 мм, блок D-22: ось hip вдоль корпуса, три сустава на ногу, колено назад, высоты осей hip и thigh; при несовпадении СТОП и вернуться к владельцу), правила «между центрами осей», формат передачи чисел в чат; (2) часть 2: подготовка к замеру скорости (мультиметр: 6.0 В на V+, общая земля, VCC PCA9685 3.3 В, конденсатор на V+; маркировка шунта R100 = 0.1 Ом или R010 = 0.01 Ом; адрес INA219 из 0x41/0x44/0x45; `docker compose stop`; транспортир: два импульса `pca9685_probe pulse 1 <центр−Δ>` и `<центр+Δ>`, угол между положениями), ссылка на README `tools/servo_speed` для запуска; (3) `docs/PARTS_ORDER.md`: выключатель V+ (SAF-09), сигнализатор LiPo (SAF-17), лабораторный блок с ограничением тока (нужен Phase 7), запасные MG996R; (4) запись в таблицу «Открыто» `docs/REVIEW.md`: драйвер ограничивает скорость в пространстве сустава (`servo_driver.cpp:226`), для колена с тягой серва быстрее в 1.3–1.6 раза, пометка для Phase 3; (5) «8/8» → «10/10 (stand, 8 манёвров, lie)» в `docs/SIMULATION.md:42`, `docs/DEPLOYMENT.md:244` и `:250`, `docs/TERRAIN.md:80`, `README.md:88`; `docs/SIMULATION.md:3` (не «позиционный регулятор», а идеальный ограничитель скорости без запаздывания) и новый раздел про `servo_model:=real`, `servo_speed`, `servo_delay_ms`, автопериод (`gait.auto_period`, `servo.*`); `docs/DEPLOYMENT.md` «Этап 1»: строки про `body_com_x` и четыре длины тяги колена, критерий перехода «10/10».
- Имена-контракт: путь листа замеров `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-MEASUREMENT-SHEET.md`; имена аргументов в документах те же, что в 01-10 и 01-08.
- Решения: D-18, D-19, D-20, D-21, D-22, D-26, D-10 (чек-лист питания), D-11 и D-12 (описание автопериода), D-14..D-16 (описание профиля), Claude's Discretion (заказ деталей).
- Чекпоинты владельца: нет (лист — входной документ для 01-14 и 01-15). CI-only: нет.
- Задач: 4 (лист, часть 1; лист, часть 2; заказ деталей и `docs/REVIEW.md`; правки «8/8», `SIMULATION.md`, `DEPLOYMENT.md`).

### 01-08 — Автопериод в контроллере и узле (W3)

- Поток: S2. Трассер: `LocomotionController` с `auto_period = true` и `servo.max_speed = 6.0` строит походку с периодом 0.600 с (gtest), затем расширение до проводки.
- Файлы: `ros2_ws/src/dog_control/include/dog_control/locomotion.hpp`; `ros2_ws/src/dog_control/src/locomotion.cpp`; `ros2_ws/src/dog_control/src/locomotion_node.cpp`; `ros2_ws/src/dog_control/test/test_locomotion.cpp`.
- Имена-контракт: поля `LocomotionParams`: `bool auto_period{false}`, `double min_period{0.55}`, `ServoSpeedModel servo` (`locomotion.hpp` включает `servo_limits.hpp`); `GaitParams::period{0.55}` не меняется; приватная `effectivePeriod(const LocomotionParams &)` в `locomotion.cpp` (ручной `gait.period`, если `auto_period == false`, иначе `minimalPeriod(p, p.servo, p.min_period, kMaxAutoPeriod)`; `0.0` → конструктор бросает `std::runtime_error`, узел пишет `RCLCPP_FATAL` и выходит с кодом 1); `double LocomotionController::gaitPeriod() const` (действующий период); `bool LocomotionController::reconfigureGait(double period, bool auto_period, double min_period, const ServoSpeedModel & servo)` — принимает и пересоздаёт `gait_` только в `Mode::PASSIVE`, `STAND`, `LYING`, иначе `false`. Узел: параметры `gait.auto_period`, `gait.min_period`, `servo.max_speed`, `servo.margin`, `servo.knee_ratio` (числа из YAML читать терпимо к int и double по образцу `declareNumber` из `servo_driver_node.cpp`); колбэк `add_on_set_parameters_callback` регистрируется ПОСЛЕ `loadParams()`, реагирует на `servo.*`, `gait.auto_period`, `gait.min_period`, `gait.period` (на остальные имена отвечает `successful=true` без изменений, как сейчас), при отказе контроллера возвращает `successful=false` с причиной («period change rejected in mode walk»); `main()` с `try/catch` и `RCLCPP_FATAL`; лог вычисленного периода при старте; обновить блок комментария узла. Имена gtest: `Locomotion.JointSpeedsFitTheServos` (прежний, явно `auto_period = false`, порог 5.5), `Locomotion.JointSpeedsFitTheServosAuto` (таблица 3.5…7 рад/с, пик ≤ `margin * max_speed + 1e-9`), `Locomotion.AutoPeriodAtStart`, `Locomotion.ReconfigureAcceptedWhenStanding`, `Locomotion.ReconfigureRejectedWhenWalking`, `Locomotion.NoFitThrows`. Тесты НЕ опираются на умолчания `auto_period`/`servo` (задают явно): умолчания кода синхронизирует 01-15.
- Решения: D-11, D-12 (пересчёт и при старте, и при смене параметра; владелец отклонил рекомендацию исследования «только при старте»), D-13, D-20. `<reversibility rating="costly">`: затрагивает `LocomotionParams`, `robot.yaml`, `servos.yaml`, `urdf.py` и тесты (D-12).
- Чекпоинты владельца: нет. CI-only: сборка `locomotion_node` и `colcon test` на Jazzy и Lyrical (последняя задача: `git push` ветки и `ci_dispatch.sh --wait-job 'build + test'`); `rclcpp` локально недоступен.
- Задач: 4 (ядро и ворота; `reconfigureGait`; проводка узла; проверка сборки в CI).

### 01-09 — `dog_bench`: рампа, предохранители, PWM (W3)

- Поток: S2, инструмент замера. Трассер (задача 1): рампа → события PWM на `FakePwm` на одной скорости.
- Файлы: `ros2_ws/src/dog_bench/include/dog_bench/ramp.hpp`, `src/ramp.cpp`, `include/dog_bench/safety.hpp`, `src/safety.cpp`, `include/dog_bench/pwm_out.hpp`, `src/pwm_out.cpp`, `test/test_ramp.cpp`, `test/test_safety.cpp`, `test/test_pwm_out.cpp` (все new); `ros2_ws/src/dog_bench/CMakeLists.txt` (добавляет источники и `ramp safety pwm_out` в `foreach`).
- Имена-контракт: `RampParams {amp_deg{25.0} (жёсткий максимум 30), center_us (1000…1800), us_per_deg{9.444}, hold_s{0.4}, rest_s{3.0}}` и `std::string validate() const`; `kDefaultSpeedsRadS = {1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 8.0, 9.0, 10.0}` (по возрастанию, ≤ 10 рад/с), по 5 пар ходов A→B, B→A на скорость; генератор `Ramp` с `update(dt)` и полями `stroke_id`, `direction`, `cmd_us`; `SafetyParams {overcurrent_a{2.0}, overcurrent_time_s{0.05}, hard_fraction{0.95} (от шкалы PGA), plausibility_min_a{0.03}, plausibility_max_a{3.0}, max_consecutive_ina_errors{3}, max_tick_overrun_s{0.05}, max_seconds{600}}`; `enum class SafetyEvent {NONE, OVERCURRENT, SATURATION, IMPLAUSIBLE_SHUNT, INA_ERRORS, PCA_ERROR, TICK_OVERRUN, TIMEOUT}`; `SafetyGuard::update(reading, now)` (без фильтра по току, событие один раз); `PwmOut` (интерфейс), `Pca9685Out` (RAII: деструктор пишет `ALL_LED_OFF` = байты `{0xFA, 0, 0, 0, 0x10}`; перед стартом читает `MODE1` (не sleep), `PRE_SCALE == 121`, `LEDn_OFF_H` всех 16 каналов и отказывается при активном чужом канале; чип не переинициализирует), `FakePwm`. Имена gtest: `Ramp.ValidateRejectsAmpAbove30`, `Ramp.SpeedsAscendAndCapped`, `Safety.SpikesDoNotTrip`, `Safety.SustainedOvercurrentTripsOnce`, `Safety.HardSaturationTripsImmediately`, `Safety.ImplausibleShuntRefuses`, `Safety.ConsecutiveInaErrorsTrip`, `PwmOut.DestructorReleasesAllChannels`, `PwmOut.RefusesForeignActiveChannel`, `PwmOut.RefusesSleepingOrWrongPrescale`.
- Решения: D-08 (сетка скоростей), D-10 (порог тока ниже предела INA219, один канал, ограниченный диапазон, выходы сняты при любом завершении), D-09.
- Чекпоинты владельца: нет. CI-only: сборка под GCC 13/15 (проверяется в 01-12).
- Задач: 3 (рампа; предохранители; PWM с RAII).

### 01-10 — Реалистичный профиль серв (W3)

- Поток: S3 реалистичная модель (GAIT-10; `servo_model:=real`); `body_com_x` (S1). Трассер (задача 1): `build_urdf(..., servo_model='real')` на реальном `robot.yaml` даёт 12 `<dynamics friction="0.06">` и пересчитанные `limit velocity/effort`; `ideal` байт-в-байт прежний.
- Файлы: `ros2_ws/src/dog_description/dog_description/servo_profile.py` (new); `ros2_ws/src/dog_description/dog_description/urdf.py`; `ros2_ws/src/dog_description/test/test_servo_profile.py` (new); `ros2_ws/src/dog_description/test/test_urdf.py`; `ros2_ws/src/dog_gazebo/dog_gazebo/joint_command_bridge.py`; `ros2_ws/src/dog_gazebo/launch/sim.launch.py`.
- Имена-контракт: `servo_profile`: `ServoProfile` (`@dataclass`: `backlash_deg`, `delay_ms`, `friction_nm`, `bus_voltage`, `bus_voltage_ref`; `from_params(dict)`; умолчания равны `robot.yaml`), `BacklashPlay(width_rad)` (`__call__(x)`), `DelayLine(delay_s)` (`push(t, item)`, `pop_ready(now)`), `CommandShaper(backlash_rad, delay_s)` (`push(t, names, positions)`, `pop_ready(now)`; нули = прозрачный проход, мост остаётся тонким), `speed_factor(v, v_ref=6.0) = 1 + 0.1471 * (v - v_ref)`, `torque_factor(v, v_ref=6.0) = 1 + 0.1212 * (v - v_ref)`, напряжение зажато в [4.8, 6.6]. `urdf.py`: `build_urdf(geometry, description=None, gazebo=False, namespace='dog', initial=None, servo_model='ideal')`; `DEFAULT_DESCRIPTION['body_com_x'] = 0.0`; `load_config` добавляет `desc['servo_sim']` и `desc['servo']` словарями (кортеж `(geometry, description)` не меняется); при `real`: `<dynamics damping="0" friction="f"/>` у 12 revolute-суставов, множители на `effort`, `velocity` и `cmd_max`, для колена при `servo.knee_ratio != 1.0` скорость делится, момент умножается на `knee_ratio` (D-20); `body_com_x` входит в `inertial` ствола для обоих режимов (при `0.0` вывод прежний). Запуск: аргументы `servo_model` (умолчание `ideal`, значения `ideal | real`), `servo_speed` (пусто = из YAML; переопределяет физическую `description.servo_velocity`, предполагаемая `servo.max_speed` не трогается, D-13), `servo_delay_ms` (пусто = из YAML); параметры моста `backlash_deg` (умолчание 0.0) и `delay_s` (умолчание 0.0) — `sim.launch.py` передаёт нули при `ideal`; мост использует `CommandShaper`, таймер 500 Гц по времени узла, при нулях поведение идентично нынешнему. Имена pytest: `test_ideal_urdf_unchanged`, `test_real_urdf_has_12_friction_joints`, `test_speed_and_torque_factors`, `test_voltage_clamped`, `test_backlash_dead_zone_and_reversal`, `test_delay_line_orders_by_sim_time`, `test_shaper_zero_is_passthrough`, `test_body_com_x_moves_trunk_inertial`.
- Решения: D-13, D-14, D-15 (идеальная модель по умолчанию; `<reversibility rating="costly">`), D-16, D-20, D-21; просадка только статическая (динамическая от тока вне фазы, Deferred).
- Чекпоинты владельца: нет. CI-only: проверка, что `<dynamics>` доходит до SDF, и что робот встаёт и ходит на `real` (делают job `acceptance` и шаг `gz sdf -p` из 01-13, `backstop`); последняя задача плана: `git push` и `ci_dispatch.sh --wait-job 'gazebo walk check'` (путь `ideal` не изменился).
- Задач: 4 (трассер: факторы и `build_urdf real`; `BacklashPlay`, `DelayLine`, `CommandShaper`; `body_com_x`; мост и `sim.launch.py`).

### 01-11 — `walk_check` и CLI `acceptance` (W3)

- Поток: S4. Трассер (задача 1): `walk_check --maneuvers backward --backward-speed 0.05` разбирается чистым модулем и записывает `dyaw5_deg` в значения манёвра (pytest без rclpy).
- Файлы: `ros2_ws/src/dog_gazebo/dog_gazebo/walk_check_args.py` (new); `ros2_ws/src/dog_gazebo/dog_gazebo/walk_check.py`; `ros2_ws/src/dog_gazebo/test/test_walk_check_args.py` (new); `ros2_ws/src/dog_gazebo/dog_gazebo/acceptance.py` (new); `ros2_ws/src/dog_gazebo/test/test_acceptance_cli.py` (new); `ros2_ws/src/dog_gazebo/setup.py` (строка `'acceptance = dog_gazebo.acceptance:main'`).
- Имена-контракт: `walk_check_args.parse_args(argv=None)` (без импорта rclpy; `walk_check.py` импортирует его и `WalkCheck` принимает `backward_speed` и `maneuvers`); флаги `--maneuvers NAME[,NAME…]` (подмножество; `lie` пропускается при подмножестве; на `slope` не влияет), `--backward-speed` (модуль, 0.10 по умолчанию, `('x', -speed * T)`), существующие флаги и ключи результата не меняются (`dx`, `dy`, `dyaw_deg`, `ratio`, `tilt_deg`, `z`), новый ключ `dyaw5_deg` записывается сразу после командной части (до выбега 1.5 с); команда push-CI `walk_check --backward-ratio 0.2` работает без изменений. `acceptance`: флаги `--repeats N` (1…20, умолчание 5), `--cells NAME[,NAME…]|all` (имена из `CELLS`), `--servo-model ideal|real`, `--distro` (умолчание `$ROS_DISTRO`), `--domain` (база 80; на повтор `domain = 80 + i % 10`, домены 80–89), `--floor-ratio` (0.2), `--out`, `--summary`, `--git-sha` (умолчание `$GITHUB_SHA`), `--strict`, `--dry-run` (печатает командные строки, симуляцию не запускает); чистые функции `cell_launch_args(cell, servo_model)` (режим B: `heading_hold:=false`, `slope_compensation:=false`; `real`: `servo_model:=real`), `cell_walk_check_args(cell)` (`--maneuvers …`, `--backward-speed …`, `--min-ratio 0 --backward-ratio 0`, чтобы зачёт жил только в `acceptance_stats`), `plan_runs(cells, repeats)`; запуск повтора через `terrain_sweep.run_level(kind, level, seed, domain, extra, launch_args, keep, sim_log)` (импорт `run_level` и `never_stood`, `terrain_sweep.py` не правится), перезапуск один раз при `never_stood`, замена `no_stand` до 2·n попыток; JSON пишется после каждого повтора; код выхода 0, если артефакты получены, 1 при сбое инфраструктуры или при `--strict` и провале зачёта; артефакты `acceptance_<distro>_<servo_model>.json`, `…_summary.md`, `*.sim.log`.
- Решения: D-01, D-02, D-03, D-04 (обычная CLI-команда, локально не запускается), D-06, D-07, D-24; запреты D-17 (пороги не ослабляются).
- Чекпоинты владельца: нет. CI-only: запуск настоящей приёмки (делает 01-13); локально pytest (`--dry-run`, разбор аргументов, формирование строк).
- Задач: 3 (чистый разбор аргументов и `dyaw5_deg` в `walk_check`; `acceptance.py`; `setup.py` и тесты CLI).

### 01-12 — `dog_bench`: сессия, `dry-run` и `run` (W4)

- Поток: S2. Трассер (задача 1): `Session::run` на `FakeI2cBus` и `FakePwm` проходит рампу, пишет CSV и `.meta.json`, отпускает каналы.
- Файлы: `ros2_ws/src/dog_bench/include/dog_bench/session.hpp`, `src/session.cpp`, `test/test_session.cpp` (new); `ros2_ws/src/dog_bench/src/servo_speed_test_main.cpp`; `ros2_ws/src/dog_bench/CMakeLists.txt` (`session` в `foreach`); `tools/servo_speed/README.md` (раздел «Запуск на роботе»).
- Имена-контракт: `SessionConfig`, `SessionResult {stop_reason, samples, exit_code}`, `Session::run()` (на любом выходе, включая исключения, PWM отпущен; мониторинг тика ≤ 1 мс, опрос INA ≥ 500 Гц, метки `CLOCK_MONOTONIC`, середина и длительность транзакции); сигналы SIGINT, SIGTERM, SIGHUP, SIGQUIT ставят флаг (`volatile sig_atomic_t`), SIGSEGV, SIGABRT, SIGBUS, SIGFPE делают один `write()` пяти байт `ALL_LED_OFF` в заранее открытый дескриптор и `_exit(3)`; CSV `t_s, stroke_id, direction, cmd_us, shunt_raw, bus_raw`; `<out>.meta.json` (поля из 01-06); значения `stop_reason`: `completed`, `overcurrent`, `saturation`, `implausible_shunt`, `ina_errors`, `pca_error`, `tick_overrun`, `timeout`, `signal`, `refused_foreign_channel`, `refused_not_confirmed`. CLI флаги: `--device` (`/dev/i2c-0`), `--ina-address` (обязателен), `--pca-address` (0x40), `--shunt-ohm` (обязателен, > 0), `--channel` (обязателен, ровно один), `--amp-deg` (25, максимум 30), `--center-us` (1000…1800), `--us-per-deg` (9.444; из транспортира: `us_per_deg = Δus_полного_хода / угол_в_градусах`, вместо условного `--sweep-deg` из RESEARCH), `--speeds` (по умолчанию `kDefaultSpeedsRadS`), `--max-seconds` (600), `--out`, `--yes` (по умолчанию выключен: вопрос «питание проверено мультиметром, нога свободна, рука на выключателе? (YES)»); `dry-run` печатает план и ничего не пишет на шину.
- Решения: D-08, D-09 (одна серва, вес своей ноги, реальное напряжение шины), D-10 (все защиты), Claude's Discretion (серва `lf_thigh_joint`, канал 1, RESEARCH Open Question 6).
- Чекпоинты владельца: нет (запуск на железе — 01-14). CI-only: последняя задача: `git push` и `ci_dispatch.sh --wait-job 'build + test'` (пакет `dog_bench` на Jazzy и Lyrical без предупреждений, все gtest).
- Задач: 3 (сессия; CLI `dry-run` и `run`, README; проверка сборки в CI).

### 01-13 — `ci.yml`: job `acceptance`, job `servo-speed`, пробный и базовый прогоны (W5)

- Поток: S4 приёмка и CI-job. Трассер (задача 1): один job `acceptance` с одной ячейкой и `repeats=1` проходит путь «вход workflow → `env:` → CLI → JSON → артефакт».
- Файлы: `.github/workflows/ci.yml`; `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-BASELINE.md` (new); `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/acceptance/baseline/` (new; только JSON и `summary.md`, логи симулятора не коммитить).
- Имена-контракт: входы `workflow_dispatch`: `acceptance` (boolean, false), `repeats` (string, '5'), `cells` (choice: `all`, `flat_A_bwd10`, `flat_B_bwd10`, `flat_B_bwd05`, `waves10_A_bwd10`, `rocks10_A_bwd10`), `strict` (boolean, false); job `acceptance` с именем `acceptance (${{ matrix.distro }}, ${{ matrix.servo_model }})`, `if: github.event_name == 'workflow_dispatch' && inputs.acceptance`, матрица `distro: [jazzy, lyrical]` × `servo_model: [ideal, real]`, `container: osrf/ros:${{ matrix.distro }}-simulation`, `permissions: contents: read`, `timeout-minutes: 90`, первым шагом pytest `ros2_ws/src/dog_gazebo/test`, значения входов только через `env:` (`REPEATS`, `CELLS`, `STRICT`), артефакт `acceptance-${{ matrix.distro }}-${{ matrix.servo_model }}` (`*.json`, `*_summary.md`, `*.sim.log`, `if: always()`), вывод в `$GITHUB_STEP_SUMMARY`, шаг «`gz sdf -p` + `grep friction`» только для `real` (если CLI `gz` отсутствует, шаг сообщает и пропускается: `[ASSUMED]` наличие CLI в образе); job `servo-speed` с именем `servo speed analysis (tools/servo_speed)` (`pip install numpy pytest`, `python -m pytest -q tools/servo_speed/tests`). Строка `walk_check --backward-ratio 0.2` НЕ меняется (последняя правка кода фазы — 01-17); существующие job не переформатируются; правки ci.yml: каждый блок отдельной задачей и отдельным коммитом. Базовый прогон «до»: `repeats=5 cells=all` для всех 4 элементов матрицы + отдельный запуск `cells=flat_A_bwd10 repeats=10` (разброс, данные для D-05) до любых правок походки и до ввода чисел 01-15; `01-BASELINE.md` содержит таблицы минимум/медиана по ячейкам и дистрибутивам, `wall_s` повтора, кандидат порога по `derive_push_threshold` (только запись, в ci.yml не вносится), выводы о том, нужны ли правки походки (вход для 01-16).
- Решения: D-01, D-03, D-04, D-05, D-06, D-15, D-23 (базовый прогон «до»), D-24, D-25. Безопасность: `${{ inputs.* }}` не подставляется в `run:` (ASVS V5), `permissions: contents: read`, `choice` и boolean.
- Чекпоинты владельца: нет. CI-only: всё содержимое плана (пробный запуск `repeats=1` проверяет и сборку всего кода волн 2-4 на обоих дистрибутивах; найденные дефекты в файлах уже завершённых планов допустимо чинить мелкими правками с записью в SUMMARY).
- Задач: 4 (входы и job `acceptance`; job `servo-speed`; пробный запуск с исправлениями; базовый прогон «до» и `01-BASELINE.md`).

### 01-14 — Владелец: замер скорости MG996R (W6)

- Поток: S2, владелец. Трассера нет: план из чекпоинтов владельца и анализа. Зависимость от 01-13 искусственная: владельческие шаги идут после всей независимой работы и базового прогона, чтобы не блокировать независимые волны (D-23).
- Файлы (все new): `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/run_01.csv`; `…/servo_speed/run_01.csv.meta.json`; `…/servo_speed/analysis_01.json`; `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-SERVO-SPEED-RESULT.md`.
- Чекпоинты владельца (`checkpoint:human-action`): (1) подготовка: мультиметр (6.0 В на V+, общая земля, VCC PCA9685 3.3 В, конденсатор на V+), маркировка шунта (R100 или R010), адрес INA219, транспортир (угол между двумя импульсами → `us_per_deg`), `docker compose stop`, обновление образа на Pi с ветки фазы; (2) `servo_speed_test selftest` (PWM не включается), затем `dry-run`, затем `run` с рукой на питании, пересылка CSV и `.meta.json`; (4) утверждение измеренной скорости и итоговой доли ворот `servo.margin` (D-11). Автоматическая задача (3): `analyze.py`, график «расстояние между следами от заданной скорости» (проверить излом, D-08), отчёт `01-SERVO-SPEED-RESULT.md` с рекомендацией для `servo.max_speed`, `description.servo_velocity`, `max_joint_speed`, `servo.margin`, `servo_sim.bus_voltage_ref`.
- Имена-контракт: результат фиксируется в `01-SERVO-SPEED-RESULT.md` строками `servo_max_speed_rad_s`, `bus_v_mean`, `servo_margin_approved`; 01-15 читает именно их.
- Решения: D-08, D-09, D-10, D-11.
- CI-only: нет.
- Задач: 4 (чекпоинт подготовки; чекпоинт `selftest` и `run`; анализ; чекпоинт утверждения).

### 01-15 — Ввод измеренных чисел (W7)

- Поток: S1 и S2, владелец. Трассера нет: применение данных владельца (D-26: таблица в чат → `robot_setup --cli` → `--check` → `walk_check` в CI → коммит).
- Файлы: `ros2_ws/src/dog_bringup/config/robot.yaml`; `ros2_ws/src/dog_bringup/config/servos.yaml` (одна строка `max_joint_speed`; возможен тривиальный конфликт с Phase 3); `ros2_ws/src/dog_control/include/dog_control/servo_limits.hpp` (умолчание `ServoSpeedModel`); `ros2_ws/src/dog_control/include/dog_control/locomotion.hpp` (умолчание `auto_period`); `ros2_ws/src/dog_control/test/test_locomotion.cpp` (только если умолчания потребуют правки ожиданий); `ros2_ws/src/dog_description/dog_description/urdf.py` (умолчание `servo_velocity`); `docs/HARDWARE.md` (5.1 → измеренное); `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-MEASUREMENTS-RAW.md` (new).
- Состав: (1) `checkpoint:human-action`: владелец присылает таблицу замеров по листу и ответ по блоку D-22 (при несовпадении оси, схемы ноги или направления колена СТОП и возврат к владельцу, кинематику молча не менять); (2) числа через `python3 tools/robot_setup/robot_setup.py --cli`, затем `--check` без ошибок (стойка достижима, колено 60–100°, тяга замыкается, массы ≤ 8 %), `body_com_x`, `servo.knee_ratio` правятся вручную, комментарии «измерено <дата>» вместо «v1: …»; (3) три скорости: `description.servo_velocity`, `servo.max_speed`, `max_joint_speed` равны измеренной (инвариант из 01-02 зелёный), `servo.margin` утверждённая, `servo_sim.bus_voltage_ref` из лога, `gait.auto_period: true`; умолчания кода приводятся к YAML (`ServoSpeedModel`, `auto_period`, `servo_velocity` в `urdf.py`; `servo_driver.hpp` не трогать: умолчание `max_joint_speed{6.0}` остаётся, YAML главнее); все пять gtest `dog_control` проходят через `tools/local_gtest/run.sh`; (4) CI-only: `git push` и `ci_dispatch.sh` (`build + test` и `gazebo walk check` на обоих дистрибутивах на измеренной геометрии, критерий перехода D-18).
- Решения: D-11, D-12, D-13, D-18, D-20, D-21, D-22, D-26.
- Чекпоинты владельца: одна `checkpoint:human-action` (таблица замеров). CI-only: задача 4.
- Задач: 4.

### 01-16 — Правки походки (условно, W8)

- Поток: S5. Трассер (задача 1): приёмка на измеренной геометрии (`repeats=5 cells=all`, CI) даёт решение «нужны ли правки».
- Файлы: `ros2_ws/src/dog_control/src/gait.cpp`; `ros2_ws/src/dog_control/include/dog_control/gait.hpp`; `ros2_ws/src/dog_control/test/test_gait.cpp`; `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-POST-NUMBERS.md` (new); `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/acceptance/post-numbers/` (new). Если правке нужен новый параметр, добавляются `ros2_ws/src/dog_bringup/config/robot.yaml` и `ros2_ws/src/dog_control/src/locomotion_node.cpp` (значение по умолчанию кода == YAML).
- Правило условности: задачи 2 и 3 выполняются, только если на Jazzy реалистичной модели (`servo_model=real`) не выполнено хотя бы одно из: `flat_A_bwd10` `ratio_min ≥ 0.40` и `max|dyaw5| ≤ 10°` (backward, left, right), `flat_B_bwd10` `ratio_min ≥ 0.40`, без падений, наклон < 20°, корпус не просел. Иначе в SUMMARY пишется «правки не требуются», `01-POST-NUMBERS.md` хранит данные, код не меняется. Если после правок Jazzy real всё ещё ниже 0.40 — `CHECKPOINT REACHED` (решение владельцу), пороги не ослабляются.
- Допустимо (D-17): любая форма траектории переноса в `gait.cpp` (направленная форма переноса при заднем ходе: `p.z = step_heights_[leg] * sin(pi * s)`, целевая точка касания `n + 0.5 * step`, профиль `blend`). Запрещено: общий подъём `step_height`, снижение порогов CI. Ход вперёд (102–113 % команды) и остальные манёвры остаются под `walk_check` и gtest `dog_control` на обоих дистрибутивах; пик `peakServoSpeed` после правки остаётся ≤ `margin * max_speed` (`Locomotion.JointSpeedsFitTheServosAuto`). Улучшать ход назад на неровностях не нужно (D-07).
- Решения: D-17, D-07, D-01, D-11; GAIT-06 (`walk_check` — регрессионный барьер).
- Чекпоинты владельца: нет (эскалация только при провале). CI-only: задачи 1 и 3 (приёмка и `walk_check` на Jazzy и Lyrical); локально gtest (`test_gait`, `test_locomotion`, `test_servo_limits`).
- Задач: 3 (решение по данным; условная правка; условная проверка).

### 01-17 — Итоговая приёмка и порог push-CI (W9)

- Поток: S4. Трассера нет: финальная проверка на уже готовом коде. Правка `--backward-ratio` — последняя правка кода фазы, строго после итоговых данных.
- Файлы: `.github/workflows/ci.yml` (только строка `walk_check --backward-ratio 0.2` в job `simulation`, выражением по `matrix.distro`, с комментарием со ссылкой на замер и `docs/TERRAIN.md`); `docs/TERRAIN.md`; `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-ACCEPTANCE-REPORT.md` (new); `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/acceptance/final/` (new; JSON и `summary.md`).
- Состав: (1) CI-only: итоговый запуск `-f acceptance=true -f repeats=5 -f cells=all -f strict=true` (4 элемента) и отдельно `cells=flat_A_bwd10 repeats=10` на идеальной модели; (2) `01-ACCEPTANCE-REPORT.md`: минимум и медиана по ячейкам, вердикты Jazzy (зачёт: ≥ 40 %, увод ≤ 10° в режиме A, без падений) и Lyrical (ориентир), при провале зачёта — `CHECKPOINT REACHED` владельцу, порог не меняется; (3) `python3 -m dog_gazebo.acceptance_stats threshold …` по данным идеальной модели и правка одной строки `ci.yml` (значение по дистрибутиву; не ниже 0.2, не выше 0.4); (4) CI-only: `git push`, `ci_dispatch.sh` (job `gazebo walk check` на обоих дистрибутивах с новым порогом зелёные), `docs/TERRAIN.md`: таблица результатов и пороги с источником.
- Решения: D-01, D-03, D-05, D-06, D-07, D-15, D-17 (пороги не ослабляются), D-24, D-25.
- Чекпоинты владельца: нет (эскалация при провале). CI-only: задачи 1 и 4.
- Задач: 4.

## Decision coverage map

Каждый план обязан процитировать (по ID, в `must_haves.truths` или `<objective>`) решения из своей строки.

| Decision | Планы |
|----------|-------|
| D-01 | 01-03, 01-11, 01-13, 01-16, 01-17 |
| D-02 | 01-03, 01-11 |
| D-03 | 01-03, 01-11, 01-13, 01-17 |
| D-04 | 01-01, 01-11, 01-13 |
| D-05 | 01-03, 01-13, 01-17 |
| D-06 | 01-03, 01-11, 01-13, 01-17 |
| D-07 | 01-03, 01-11, 01-16, 01-17 |
| D-08 | 01-05, 01-06, 01-09, 01-12, 01-14 |
| D-09 | 01-05, 01-06, 01-09, 01-12, 01-14 |
| D-10 | 01-05, 01-07, 01-09, 01-12, 01-14 |
| D-11 | 01-04, 01-07, 01-08, 01-14, 01-15, 01-16 |
| D-12 | 01-04, 01-07, 01-08, 01-15 |
| D-13 | 01-02, 01-04, 01-08, 01-10, 01-15 |
| D-14 | 01-07, 01-10 |
| D-15 | 01-07, 01-10, 01-13, 01-17 |
| D-16 | 01-02, 01-07, 01-10 |
| D-17 | 01-11, 01-16, 01-17 |
| D-18 | 01-02, 01-07, 01-15 |
| D-19 | 01-07 |
| D-20 | 01-02, 01-04, 01-07, 01-08, 01-10, 01-15 |
| D-21 | 01-02, 01-07, 01-10, 01-15 |
| D-22 | 01-07, 01-15 |
| D-23 | 01-13 (базовый прогон «до»), 01-14 и 01-15 (чекпоинты после независимых волн), весь порядок таблицы |
| D-24 | 01-01, 01-03, 01-11, 01-13, 01-16, 01-17 (каждый CI-план) |
| D-25 | 01-01, 01-08, 01-10, 01-12, 01-13, 01-15, 01-17 (каждый план с push и dispatch) |
| D-26 | 01-07, 01-15 |

## Requirement coverage map

| Requirement | Планы |
|-------------|-------|
| CAL-16 | 01-02 (`body_com_x`, проверки `robot_setup`), 01-07 (лист замеров), 01-10 (`body_com_x` в URDF), 01-15 (ввод чисел, `--check`, `walk_check` в CI) |
| CAL-17 | 01-02 (инвариант трёх скоростей), 01-04, 01-05, 01-06, 01-07 (чек-лист), 01-08, 01-09, 01-12, 01-13 (job `servo-speed`), 01-14 (замер), 01-15 (числа) |
| GAIT-01 | 01-03, 01-11, 01-13, 01-16, 01-17 |
| GAIT-02 | 01-01, 01-03, 01-11, 01-13, 01-17 |
| GAIT-06 | 01-01, 01-04, 01-07 (10 проверок), 01-08, 01-11, 01-13, 01-15, 01-16, 01-17 |
| GAIT-10 | 01-02 (ключи `servo_sim`), 01-07 (описание профиля), 01-10, 01-13 (матрица `real`), 01-17 |

## Probe items ownership

Детерминированный зонд классифицировал все 6 строк как `unclassified` (русский текст требований), все `unresolved`. Ни одна строка не закрывается автоматически и не отбрасывается: каждая получает владельца-план, который вписывает конкретные предикаты в `must_haves.truths` (строка, если проверка явная; объект `{ statement, verification: backstop }`, если возможна только страховка). Равенство «найдено = вписано + помечено допущением» проверяется по этой таблице.

### Шесть нерешённых пунктов зонда

| Requirement | План, автор предикатов | Предикаты (кандидаты для `must_haves.truths`) | Допущения и `backstop` |
|-------------|------------------------|-----------------------------------------------|------------------------|
| CAL-16 | 01-02, 01-07, 01-15 | 01-02: ключ `body_com_x` читается и сохраняется без потери комментариев; `|body_com_x| > hip_x` даёт предупреждение; ошибки несуществующих датчиков не блокируют `save` и `--check`; `knee_ratio_max` совпадает с золотыми значениями C++; предупреждение, если `servo.knee_ratio` меньше вычисленного. 01-07: лист содержит блок D-22 с явным СТОП. 01-15: `--check` без ошибок (достижимость стойки, колено 60–100°, тяга замыкается, массы ≤ 8 %); `walk_check` на измеренной геометрии проходит (CI) | backstop: точность мерок владельца (штангенциркуль, кухонные весы) |
| CAL-17 | 01-05, 01-06, 01-09, 01-12, 01-14, 01-08, 01-02 | 01-05: отказ при медиане опроса > 1.5 мс или p99 > 5 мс; 0x40 не зондируется как INA. 01-09: ток > 2.0 А дольше 50 мс и насыщение PGA (95 %) отпускают выходы; пик вне 0.03–3 А по указанному шунту даёт отказ; ≥ 3 ошибок INA подряд отпускают; чужой активный канал и неверный `PRE_SCALE` дают отказ. 01-12: PWM отпущен при любом выходе (исключение, сигнал, таймаут). 01-06: сетка без насыщения даёт «не насыщено», а не число; расхождение `v_dur` и `v_sat` > 15 % даёт меньшее и флаг; результат не зависит от масштаба шунта. 01-08: период не помещается до 1.5 с — старт с FATAL; живая смена отклоняется в WALK. 01-02: три скорости равны в закоммиченных YAML | backstop: SIGKILL и потеря питания не перехватываются (рука владельца на питании); `[ASSUMED]` пороги правдоподобия; `[ASSUMED]` 100 кГц на H618; ошибка перевода мкс → угол входит в запас ворот |
| GAIT-01 | 01-03, 01-11, 01-13, 01-16, 01-17 | 01-03: повтор с падением, `no_stand` и `error` классифицируются раздельно; вердикт при `n < 5` не зачётный; Lyrical не требует 40 %. 01-11: `dyaw5_deg` считается до выбега; `lie` пропускается при подмножестве манёвров; `no_stand` заменяется до 2·n попыток; домены 80–89 не пересекаются с 41–43 и 60–77. 01-16 и 01-17: увод ≤ 10° только в режиме A | backstop: разброс Gazebo от прогона к прогону; различие Jazzy и Lyrical (физика симулятора), видно только в CI |
| GAIT-02 | 01-03, 01-11, 01-13, 01-17 | 01-03: `derive_push_threshold` на границах 0.2/0.4 и округлении (0.52 → 0.40, 0.45 → 0.35, 0.30 → 0.20); по дистрибутивам. 01-13: входы только через `env:`; `timeout-minutes: 90`; `permissions: contents: read`. 01-17: порог выставляется последним, по данным ≥ 5 повторов, не ниже 0.2 | backstop: оценка времени повтора (`[ASSUMED]` 20–40 с), уточняется пробным запуском |
| GAIT-06 | 01-04, 01-08, 01-16, 01-17 | 01-04: пик на `TrotGait` совпадает с контроллером до 3 знаков; перебор с окном устойчив к немонотонности (167 нарушений на сетке 1 мс); период не короче 0.55 с; сравнения с допуском `1e-9`. 01-08: принятие и отказ `reconfigureGait` покрыты gtest; прежний `JointSpeedsFitTheServos` проходит с `auto_period = false`. 01-16 и 01-17: ход вперёд и остальные манёвры остаются; `walk_check` — все проверки на обоих дистрибутивах | backstop: физические результаты `walk_check` только в CI |
| GAIT-10 | 01-10, 01-13, 01-17 | 01-10: `ideal` байт-в-байт прежний; напряжение зажато в [4.8, 6.6]; `BacklashPlay` (мёртвая зона, реверс, начальное состояние); `DelayLine` по времени симуляции и в порядке поступления; нули `CommandShaper` = прозрачный проход; 12 `friction` у `real` | backstop: `<dynamics>` доходит до SDF (`gz sdf -p`, `[ASSUMED]` CLI в образе); SERVO + трение без дрожи и робот встаёт (`[ASSUMED]`, только CI); эффект профиля различается на Jazzy и Lyrical |

### Запреты (`must_haves.prohibitions`, без `check_*`: помечаются непроверенными, автоматически не снимаются)

| ID | Запрет | Планы-носители |
|----|--------|----------------|
| PR-01 | Не печатать адрес `origin` (токен), не пушить в `main`, не force-push, не открывать PR без команды владельца (D-25) | 01-01 и каждый план с push или dispatch: 01-08, 01-10, 01-12, 01-13, 01-15, 01-16, 01-17 |
| PR-02 | Не запускать Gazebo и симуляции локально, не собирать образы симуляции (D-24, D-04) | 01-01, 01-10, 01-11, 01-13, 01-15, 01-16, 01-17 |
| PR-03 | Не править `ros2_ws/src/dog_hardware/**` (`servo_bus.*`, `servo_driver.*`, `servo_driver_node.cpp`, `power_sensor.*`, `pca9685_probe.cpp`, `CMakeLists.txt`): параллельная Phase 3 | 01-02, 01-04, 01-05, 01-07, 01-08, 01-09, 01-12, 01-15 |
| PR-04 | Не поднимать общий `step_height`, не ослаблять пороги CI (D-17) | 01-13, 01-16, 01-17 |
| PR-05 | Не менять идеальную модель по умолчанию: `servo_model` остаётся `ideal`, URDF и мост без профиля прежние (D-15) | 01-10, 01-13, 01-15 |
| PR-06 | Не менять кинематику и схему ноги молча при несовпадении оси hip, знака или схемы; остановиться и вернуться к владельцу (D-22) | 01-07, 01-15 |
| PR-07 | Не подставлять `${{ inputs.* }}` в `run:`; значения входов только через `env:` (ASVS V5) | 01-01 (шаблоны), 01-13, 01-17 |
| PR-08 | Инструмент замера: не оставлять серву под управлением при любом выходе; не зондировать и не писать 0x40 как INA; одна серва за раз; амплитуда ≤ 30° и центр 1000–1800 мкс; `--shunt-ohm` обязателен без значения по умолчанию; `--yes` не по умолчанию (D-10) | 01-05, 01-09, 01-12, 01-14 |
| PR-09 | Значение `--backward-ratio` в push-CI менять только последней задачей кода после итоговой приёмки; остальные планы строку не трогают (D-05, D-17) | 01-13, 01-16, 01-17 |
| PR-10 | Отложенное не реализовывать: улучшение хода назад на неровностях выше 10 мм (D-07), замер всех 12 серв, скорость под весом корпуса, динамическая просадка от тока, замеры люфта, задержки и трения на роботе | 01-10, 01-14, 01-16, 01-17 |
| PR-11 | Период походки не укорачивать относительно 0.55 с (D-12); `GaitParams::period{0.55}` и умолчание `auto_period = false` не менять до синхронизации умолчаний в 01-15; ненамеренные числа не зашивать в код | 01-04, 01-08, 01-15 |
| PR-12 | Правки `ci.yml` только небольшими аддитивными блоками, каждый в своей задаче; существующие job не переформатировать | 01-13, 01-17 |

## Outline decisions

1. Волны. W1: только 01-01 (ветка и помощники). W2: семь планов с непересекающимися файлами (конфигурация и `robot_setup`; статистика; `servo_limits`; трассер `dog_bench`; анализ скорости; документы). W3: четыре плана, зависящие от W2 (автопериод, защиты `dog_bench`, профиль серв, `walk_check` и `acceptance`). W4: сессия `dog_bench` (зависит от `CMakeLists.txt` плана 01-09). W5: единственный план, меняющий `ci.yml` до конца (01-13), он же базовый прогон. W6-W7: шаги владельца и применение чисел. W8: условные правки походки. W9: итоговая приёмка. Планы одной волны имеют непересекающиеся `files_modified` и не зависят друг от друга; `ci.yml` правят только 01-13 (W5) и 01-17 (W9), 01-17 зависит от 01-13 явно.
2. Чекпоинты владельца. Вся владельческая работа идёт после независимых волн и базового прогона: 01-14 (W6, три `checkpoint:human-action` и утверждение) и первая задача 01-15 (W7, таблица замеров). Зависимость 01-14 от 01-13 искусственная, чтобы чекпоинты не блокировали независимые волны (D-23). Владелец может снимать замеры раньше: лист замеров готов после W2, инструмент `dog_bench` — после W4; формально волна ждёт ответа владельца только в W6 и W7.
3. Базовый прогон «до» — 01-13, задача 4 (W5): идеальная и реальная модели, оба дистрибутива, `repeats=5 cells=all`, плюс `cells=flat_A_bwd10 repeats=10`; выполняется до ввода измеренных чисел (01-15) и до любых правок походки (01-16); `gait.auto_period: false` держит поведение прежним до 01-15.
4. Правка `--backward-ratio` — последняя правка кода фазы: 01-17, задача 3, после итоговой приёмки; значения выводятся по `derive_push_threshold` отдельно для Jazzy и Lyrical (D-05, пределы 0.2–0.4). Порог Lyrical по формуле подтверждён владельцем не был (D-05): в отчёте отметить как допущение.
5. Отклонения от RESEARCH по именам (чтобы не переопределять в планах): исходник CLI — `src/servo_speed_test_main.cpp` (иначе `tools/local_gtest/run.sh` слинкует чужой `main`); флаг `--us-per-deg` вместо условного `--sweep-deg`; чистый разбор аргументов `walk_check` — в новом модуле `walk_check_args.py` (не ломает импорты `walk_check.py`); класс `CommandShaper` добавлен в `servo_profile.py`, чтобы мост остался тонким; `kMaxAutoPeriod = 1.5` — константа ядра, а не ключ YAML; `ServoSpeedModel` объявлена в `servo_limits.hpp` без включения `locomotion.hpp` (forward-declaration), `effectivePeriod` приватна в `locomotion.cpp`; все новые ключи `robot.yaml` добавляются одним планом 01-02.
6. D-12 дословно: период пересчитывается и при старте, и при живой смене `servo.*`, `gait.auto_period`, `gait.min_period`, `gait.period`; в WALK и переходных режимах колбэк возвращает `successful=false` с причиной; gtest покрывает принятие и отказ (RESEARCH Open Question 1, решение владельца). Числа периодов брать из RESEARCH (3.5 → 1.030, 4.0 → 0.8975, 5.0 → 0.720, 6.0 → 0.600, 6.35 → 0.550 с), а не из оценок CONTEXT; `walk_check` на ровном полу даёт 10 проверок.
7. Синхронизация умолчаний. После измерения 01-15 приводит умолчания кода к YAML (`ServoSpeedModel`, `auto_period`, `servo_velocity` в `urdf.py`); gtest из 01-08 задают `auto_period` и `servo` явно, поэтому смена умолчаний их не ломает. `servo_driver.hpp` (`max_joint_speed{6.0}`) — территория Phase 3, не правится: YAML главнее.
8. Phase 3: ничего в `dog_hardware/**` не правится; зажим скорости драйвера в пространстве сустава (`servo_driver.cpp:226`) передаётся записью в `docs/REVIEW.md` (01-07), в Phase 1 отражается только `servo.knee_ratio`. Общие файлы: `servos.yaml` (одна строка, 01-15), `ci.yml` (два плана, небольшие блоки).
9. Дисциплина CI: перед каждым CI-шагом `git push` ветки (вывод через sed); запуск `ci.yml` прогоняет все его job (~15 мин), ждать нужные через `--wait-job`; результаты копируются в `.planning/phases/01-…/acceptance/<метка>/` только как JSON и `summary.md` (логи симулятора остаются в `ros2_ws/build/_ci/` и не коммитятся). Дефекты, найденные CI в файлах уже завершённых планов (например, `sim.launch.py` при первом запуске `real`), допустимо чинить мелкой правкой внутри планов 01-13 (W5) и 01-15 (W7) с записью в SUMMARY.
10. TDD-кандидаты (по `workflow.tdd_mode` решает прогон плана): 01-03, 01-04, 01-06, 01-08 (ядро), 01-09 (`safety`, `ramp`), 01-10 (`BacklashPlay`, `DelayLine`).
11. Обратимость: необратимых (`one-way`) решений в фазе нет (ветка удаляется, YAML откатывается); `costly` отмечены в 01-08 (D-12) и 01-10 (D-15), как в CONTEXT.
12. Размер: 17 планов вместо ориентировочных 10-16: защитный инструмент `dog_bench` разбит на три плана (сквозной `selftest`, защиты и PWM, сессия), а статистика и `servo_limits` вынесены в отдельные TDD-планы по правилу «одна функциональность на TDD-план». `PHASE SPLIT` не рекомендуется (решение владельца: одна фаза, волнами).
13. Детекторы (запущены один раз, для всей фазы). Assumption-delta: `detected: false`, чекпоинт не срабатывает, `promote`/`add-alongside` не решаются. API coverage: `detected: false` (области фазы: I2C-чипы INA219 и PCA9685, GitHub Actions `workflow_dispatch`, внешнего API/SDK нет), `COVERAGE.md` не писался; если детектор на этапе запечатывания сработает по полному тексту планов, записать в `COVERAGE.md` строку `No external API integration: phase touches I2C chips (INA219, PCA9685), Gazebo and CI inputs only.` Schema-gate: файлов ORM-схем нет, задача `[BLOCKING]` не нужна. Package legitimacy: новых пакетов npm/pip/cargo нет (`numpy`, `pytest`, `pyyaml` уже в проекте), задач-чекпоинтов легитимности нет; `T-01-SC` не нужен.
14. Планы не должны повторно решать: пороги и значения предохранителей (2.0 А / 50 мс, 95 % шкалы PGA, правдоподобие 0.03–3 А, 3 ошибки INA, перерасход тика 50 мс, `--max-seconds` 600, амплитуда 25 (макс. 30), центр 1000–1800 мкс, сетку скоростей 1.5…10.0); порог `selftest` (медиана 1.5 мс, p99 5 мс); формулы `speed_factor`/`torque_factor` и номиналы `servo_sim.*`; имена ячеек и правила зачёта; схему JSON 1; домены 80–89; `timeout-minutes: 90`; имена входов и job в `ci.yml`; долю ворот `servo.margin` = 0.8 до утверждения владельцем в 01-14; нижнюю границу периода 0.55 с и верхнюю 1.5 с.

## OUTLINE COMPLETE
