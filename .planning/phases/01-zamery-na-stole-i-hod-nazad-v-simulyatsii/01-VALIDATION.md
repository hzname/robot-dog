---
phase: "1"
slug: "zamery-na-stole-i-hod-nazad-v-simulyatsii"
# status lifecycle: draft (seeded by plan-phase) → validated (set by validate-phase §6)
# audit-milestone §5.5 distinguishes NOT-VALIDATED (draft) from PARTIAL (validated + nyquist_compliant: false) (#2117)
status: draft
nyquist_compliant: false
wave_0_complete: false
created: "2026-09-30"
---

# Phase 1 — Validation Strategy

> Контракт проверки фазы для обратной связи во время исполнения. Источник: `01-RESEARCH.md`, раздел «Validation Architecture».

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | GoogleTest (`ament_add_gtest`, ROS-free ядра `dog_control` и новый `dog_bench`), pytest (Python-модули, `tools/`, `dog_description`, `dog_gazebo`), `launch_testing` (существующий `test_mock_bringup`); проверки в Gazebo (`walk_check`, `acceptance`) — CLI, только в GitHub Actions |
| **Config file** | C++: `ros2_ws/src/dog_control/CMakeLists.txt` (`foreach(t …)`), новый `ros2_ws/src/dog_bench/CMakeLists.txt`; Python: `setup.py` с `extras_require={'test': ['pytest']}` (для нового `dog_gazebo/test` добавить) |
| **Quick run command** | `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q tools/robot_setup/test ros2_ws/src/dog_description/test` и локальная сборка gtest затронутого ROS-free ядра (g++/cmake, обёртка CMake лежит вне репозитория: `colcon` на этой машине нет) |
| **Full suite command** | CI: `colcon test --packages-skip dog_gazebo` (job `build-test`) и `gh workflow run ci.yml --ref gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii` (jobs `build-test`, `simulation`, `terrain`) |
| **Estimated runtime** | быстрые локальные проверки ~30 секунд [ASSUMED]; `build-test` ~3 минуты; один элемент матрицы приёмки ~15-20 минут [ASSUMED, первый запуск с `repeats=1` это проверит] |

---

## Sampling Rate

- **After every task commit:** Run `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q <затронутые каталоги>` и локальную сборку gtest затронутого ядра (десятки секунд)
- **After every plan wave:** Run `gh workflow run ci.yml --ref gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii` (полный CI); длинные симуляции только в GitHub Actions (D-24)
- **Before `/gsd-verify-work`:** полный CI зелёный, результат приёмки (4 элемента матрицы), порог `--backward-ratio` выставлен из данных, чекпоинты владельца выполнены
- **Max feedback latency:** 60 секунд локально; CI-проверки заметно дольше и идут по одной симуляции на раннер

---

## Per-Task Verification Map

Таблицу заполняет планировщик: у каждой задачи должен быть `<automated>` или зависимость от Wave 0. Посевной набор на уровне требований (из RESEARCH.md «Phase Requirements → Test Map»):

| Task ID | Plan | Wave | Requirement | Threat Ref | Secure Behavior | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|------------|-----------------|-----------|-------------------|-------------|--------|
| (планировщик) | — | — | CAL-16 | — | N/A | pytest | `python3 -m pytest -q tools/robot_setup/test` и `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_description/test` | ✅ файлы есть, новые кейсы ❌ W0 | ⬜ pending |
| (планировщик) | — | — | CAL-16 | — | N/A | CLI | `python3 tools/robot_setup/robot_setup.py --check` | ✅ | ⬜ pending |
| (планировщик) | — | — | CAL-17 | — | сессия замера отпускает серву при любом завершении, ток выше порога снимает выходы | gtest | локально g++/cmake/gtest по `dog_bench`; CI `colcon test --packages-select dog_bench` | ❌ W0 | ⬜ pending |
| (планировщик) | — | — | CAL-17 | — | N/A | pytest | `python3 -m pytest -q tools/servo_speed/tests` | ❌ W0 | ⬜ pending |
| (планировщик) | — | — | GAIT-06 | — | N/A | gtest | `./build/test_servo_limits` и `./build/test_locomotion --gtest_filter='Locomotion.JointSpeedsFitTheServos*'` | ❌ W0 (`test_servo_limits.cpp`) | ⬜ pending |
| (планировщик) | — | — | GAIT-06 | — | N/A | Gazebo (CI) | `gh workflow run ci.yml --ref <ветка>` (job `simulation`) | ✅ | ⬜ pending |
| (планировщик) | — | — | GAIT-01, GAIT-02 | — | N/A | pytest | `PYTHONPATH=ros2_ws/src/dog_gazebo python3 -m pytest -q ros2_ws/src/dog_gazebo/test` | ❌ W0 | ⬜ pending |
| (планировщик) | — | — | GAIT-01, GAIT-02 | — | N/A | Gazebo (CI) | `gh workflow run ci.yml --ref <ветка> -f acceptance=true -f repeats=5` | ❌ W0 (job и CLI) | ⬜ pending |
| (планировщик) | — | — | GAIT-10 | — | N/A | pytest | `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_description/test` | ❌ W0 (`test_servo_profile.py`) | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

- [ ] `ros2_ws/src/dog_control/test/test_servo_limits.cpp` — CAL-17, GAIT-06 (+ имя в `foreach(t kinematics gait crawl greet locomotion)`)
- [ ] `ros2_ws/src/dog_description/test/test_servo_profile.py` и кейсы в `test_urdf.py` — GAIT-10, CAL-16
- [ ] `ros2_ws/src/dog_gazebo/test/` (каталог, `test_acceptance_stats.py`, `test_walk_check_args.py`) и `extras_require={'test': ['pytest']}` в `dog_gazebo/setup.py` — GAIT-01, GAIT-02
- [ ] `ros2_ws/src/dog_bench/` (пакет, CMake, `test/test_*.cpp`, `package.xml` format 3) — CAL-17
- [ ] `tools/servo_speed/tests/test_analyze.py` и `synth.py` — CAL-17
- [ ] `tools/robot_setup/test/` кейсы: `body_com_x`, передаточное число колена (золотые значения из C++), согласованность трёх скоростей в YAML, ошибки геометрии несуществующих датчиков — CAL-16, CAL-17
- [ ] `.github/workflows/ci.yml`: `inputs` и job `acceptance` (+ небольшой job тестов `tools/servo_speed`) — GAIT-02
- [ ] Установка фреймворков не нужна: g++, cmake, gtest, pytest, numpy, pyyaml на машине уже есть

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| Замеры на столе по листу `01-MEASUREMENT-SHEET.md` (длины, массы, центр масс по X, четыре длины тяги колена, пределы суставов) | CAL-16 | нужен физический робот и инструменты владельца | владелец присылает таблицу в чат; Claude вносит через `robot_setup --cli`, проверяет `--check`, запускает `walk_check` в CI (D-26) |
| Мини-проверка питания мультиметром: 6.0 В на шине, общая земля, VCC PCA9685 = 3.3 В, конденсатор на V+ | CAL-17 | защита оборудования до первого включения | владелец по листу проверки перед `selftest` (D-10) |
| Маркировка шунта INA219 (R100 или R010) и угол между двумя импульсами транспортиром | CAL-17 | значения нельзя узнать программно | владелец сообщает маркировку и угол; значения идут в параметры `--shunt-ohm` и калибровку µs → рад |
| `selftest` и `run` инструмента замера скорости на роботе (одна серва `lf_thigh_joint`, рука на питании) | CAL-17 | нужен робот и INA219 | `docker compose run --rm robot ros2 run dog_bench …` по листу шагов; результат идёт в анализ `tools/servo_speed` |
| Одобрение измеренной скорости и итоговой доли ворот | CAL-17, GAIT-06 | решение владельца (D-11: одна серва не оценивает разброс) | владелец смотрит график «расстояние между следами от скорости» и подтверждает число |

---

## Validation Sign-Off

- [ ] All tasks have `<automated>` verify or Wave 0 dependencies
- [ ] Sampling continuity: no 3 consecutive tasks without automated verify
- [ ] Wave 0 covers all MISSING references
- [ ] No watch-mode flags
- [ ] Feedback latency < 60s (локально)
- [ ] `nyquist_compliant: true` set in frontmatter

**Approval:** pending
