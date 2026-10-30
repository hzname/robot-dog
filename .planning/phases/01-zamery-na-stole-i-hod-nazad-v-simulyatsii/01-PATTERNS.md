# Phase 1: Замеры на столе и ход назад в симуляции - Pattern Map

**Mapped:** 2026-09-30
**Files analyzed:** 41 (новых и изменяемых; однотипные файлы сгруппированы в одной секции)
**Analogs found:** 39 / 41 (2 без точного аналога, см. «No Analog Found»)

Все пути аналогов проверены: каждый файл возвращает непустой вывод `git ls-files -- <path>` (отслеживается git, зеркал `.gsd/capabilities/*` нет). Номера строк даны по состоянию `main` на 2026-09-30.

**Жёсткие границы фазы (повторяю для планировщика):**
- `ros2_ws/src/dog_hardware/**` (`servo_bus.*`, `servo_driver.*`, `servo_driver_node.cpp`, `power_sensor.*`, `pca9685_probe.cpp`) — **только чтение**. Фаза 3 идёт параллельно и меняет эти файлы. Всё, что нужно от них, копируется в новый пакет `dog_bench`, без `find_package(dog_hardware)` и без линковки с `dog_hardware_core`.
- `.github/workflows/ci.yml` общий с Фазой 3: только малые аддитивные правки отдельными коммитами; порог `--backward-ratio` (строка 94) правится последним отдельным коммитом.
- Идеальная модель серв остаётся по умолчанию (D-15): `build_urdf(..., servo_model='ideal')` обязан давать байт-в-байт прежний вывод.
- Значения по умолчанию в коде == `robot.yaml` (CLAUDE.md). Новые ключи YAML в форме `key: value  # comment`, по одному на строку (`robot_setup` правит регулярками).

## File Classification

| Новый / изменяемый файл | Role | Data Flow | Closest Analog | Match Quality |
|---|---|---|---|---|
| `ros2_ws/src/dog_control/include/dog_control/servo_limits.hpp` | utility (ядро, без ROS) | transform | `dog_control/include/dog_control/gait.hpp` | role-match |
| `ros2_ws/src/dog_control/src/servo_limits.cpp` | utility | transform | `dog_control/src/gait.cpp` | role-match |
| `ros2_ws/src/dog_control/test/test_servo_limits.cpp` | test | transform | `dog_control/test/test_gait.cpp` + `test_locomotion.cpp:147-172` | exact |
| `ros2_ws/src/dog_control/CMakeLists.txt` (правка) | config | — | он сам (строки 19-25, 47-50) | exact |
| `ros2_ws/src/dog_control/include/dog_control/locomotion.hpp`, `src/locomotion.cpp` (правка: эффективный период, переконфигурация походки) | service (ядро) | request-response | они сами (`request()` 80-140, конструктор 50-56) | exact |
| `ros2_ws/src/dog_control/src/locomotion_node.cpp` (правка: `servo.*`, `gait.auto_period`, колбэк параметров, try/catch в `main`) | node | event-driven | `dog_hardware/src/servo_driver_node.cpp` (`onParams`, `main`) + его же `loadParams()` 222-289 | exact (для колбэка) |
| `ros2_ws/src/dog_control/test/test_locomotion.cpp` (правка ворот) | test | transform | он сам (147-172) | exact |
| `ros2_ws/src/dog_bench/package.xml` | config | — | `dog_hardware/package.xml` | exact |
| `ros2_ws/src/dog_bench/CMakeLists.txt` | config | — | `dog_hardware/CMakeLists.txt` | exact |
| `ros2_ws/src/dog_bench/include/dog_bench/i2c_bus.hpp`, `src/i2c_bus.cpp` | utility (железо) | file-I/O (I2C) | `dog_hardware/src/power_sensor.cpp` `I2cRegs` (29-58) + `servo_bus.hpp` `ServoBus` + `MockBus` | role-match |
| `ros2_ws/src/dog_bench/include/dog_bench/ina219_fast.hpp`, `src/ina219_fast.cpp` | utility | streaming (опрос) | `dog_hardware/include/dog_hardware/power_sensor.hpp` namespace `ina` (35-49) + `power_sensor.cpp` 17-24, 67-99 | exact |
| `ros2_ws/src/dog_bench/include/dog_bench/pwm_out.hpp`, `src/pwm_out.cpp` | utility (железо) | file-I/O (I2C) | `dog_hardware/src/servo_bus.cpp` `Pca9685Bus` (read-only, Фаза 3) | role-match |
| `ros2_ws/src/dog_bench/include/dog_bench/ramp.hpp`, `src/ramp.cpp` | utility | transform | `dog_control/include/dog_control/gait.hpp` (чистый генератор) | role-match |
| `ros2_ws/src/dog_bench/include/dog_bench/safety.hpp`, `src/safety.cpp` | service (защита) | event-driven | `dog_hardware/include/dog_hardware/power_sensor.hpp` `PowerGuard` (51-81) + `power_sensor.cpp` 145-175 | exact |
| `ros2_ws/src/dog_bench/include/dog_bench/session.hpp`, `src/session.cpp` | service (ядро сессии, RAII-отпускание) | event-driven | `dog_hardware/src/servo_driver_node.cpp` (`relax_on_exit_`, деструктор) + `servo_bus.cpp` `close()` | partial |
| `ros2_ws/src/dog_bench/src/servo_speed_test.cpp` (`main`) | utility (CLI) | batch | `dog_hardware/src/pca9685_probe.cpp` + `servo_driver_node.cpp` `main` | exact |
| `ros2_ws/src/dog_bench/test/test_{ina219_fast,ramp,safety,session}.cpp` | test | transform | `dog_hardware/test/test_power.cpp` | exact |
| `ros2_ws/src/dog_description/dog_description/servo_profile.py` | utility (чистый Python) | transform | `tools/autocal/robotdog_autocal/servo_model.py` + `urdf.py` 61-82 | role-match |
| `ros2_ws/src/dog_description/dog_description/urdf.py` (правка) | utility (генератор) | transform | он сам (18-36, 71-82, 131, 198-227) | exact |
| `ros2_ws/src/dog_description/test/test_servo_profile.py`, правки `test_urdf.py` | test | transform | `dog_description/test/test_urdf.py` | exact |
| `ros2_ws/src/dog_gazebo/dog_gazebo/joint_command_bridge.py` (правка) | node (sim-only) | event-driven | он сам (1-42) | exact |
| `ros2_ws/src/dog_gazebo/launch/sim.launch.py` (правка) | config (launch) | request-response | он сам (`_setup` 60-147, аргументы 152-189) | exact |
| `ros2_ws/src/dog_gazebo/dog_gazebo/walk_check.py` (правка) | utility (CLI-проверка) | request-response | он сам (157-187, 246-274) | exact |
| `ros2_ws/src/dog_gazebo/dog_gazebo/acceptance.py` | utility (CLI) | batch | `dog_gazebo/dog_gazebo/terrain_sweep.py` | exact |
| `ros2_ws/src/dog_gazebo/dog_gazebo/acceptance_stats.py` | utility (чистый) | transform | `terrain_sweep.never_stood` (23-27) + `tools/autocal/robotdog_autocal/fit.py` (`@dataclass FitResult`) | role-match |
| `ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py`, `test_walk_check_args.py` (+ каталог `test/`) | test | transform | `ros2_ws/src/dog_web/test/test_protocol.py` | role-match |
| `ros2_ws/src/dog_gazebo/setup.py` (правка: `entry_points`, `extras_require`) | config | — | он сам (23-30) + `dog_description/setup.py:19` | exact |
| `tools/robot_setup/robot_setup.py` (правка: `body_com_x`, knee ratio, sensor_checks) | utility | transform | он сам (GROUPS 38-131, validate 263-330, sensor_checks 351) | exact |
| `tools/robot_setup/test/test_robot_setup.py` (правка) | test | transform | он сам | exact |
| `tools/servo_speed/analyze.py`, `synth.py`, `tests/`, `README.md` | utility + test | batch / transform | `tools/autocal/` (`robotdog_autocal/`, `tests/conftest.py`, `README.md`, `requirements.txt`) | role-match |
| `.github/workflows/ci.yml` (правка: inputs, job `acceptance`, job `servo-speed`) | config (CI) | batch | он сам: job `simulation` (68-99), `terrain` (101-212), `autocal` (40-53), `robot-setup` (55-66) | exact |
| `ros2_ws/src/dog_bringup/config/robot.yaml` (правка: `servo:`, `servo_sim:`, `gait.auto_period`, `description.body_com_x`) | config | — | он сам (30-34, 220-235) | exact |
| `ros2_ws/src/dog_bringup/config/servos.yaml` (правка: `max_joint_speed`) | config | — | он сам (28) | exact |
| `docs/SIMULATION.md`, `docs/DEPLOYMENT.md`, `docs/HARDWARE.md`, `docs/REVIEW.md`, `docs/TERRAIN.md`, `README.md` (правки) | docs (русский) | — | они сами | exact |
| `.planning/phases/01-…/01-MEASUREMENT-SHEET.md` | docs (русский) | — | `docs/DEPLOYMENT.md` «Этап 1» (79-254) + `GROUPS` в `robot_setup.py` | role-match |

## Pattern Assignments

### `ros2_ws/src/dog_control/include/dog_control/servo_limits.hpp` + `src/servo_limits.cpp` (utility, transform)

**Analog:** `ros2_ws/src/dog_control/include/dog_control/gait.hpp` и `src/gait.cpp`. Функция принимает `LocomotionParams` и считает по `TrotGait` + IK. Код без rclcpp.

**Заголовок файла** (`gait.hpp` 1-14): блок `//` сверху про систему координат и интерфейс, `#pragma once`, std-заголовки, затем `"dog_control/..."`, `namespace dog_control` на своей строке.
```cpp
// Periodic trot gait generator.
//
// Produces foot positions in the body frame: x/y on the ground plane and a
// lift height above the ground (z >= 0). ...
#pragma once

#include <array>

#include "dog_control/kinematics.hpp"

namespace dog_control
{
```

**Стиль структуры параметров** (`gait.hpp` 23-31; единицы в комментарии, значения по умолчанию в структуре == `robot.yaml`):
```cpp
struct GaitParams
{
  double period{0.55};      // full gait cycle [s]
  double duty{0.65};        // stance fraction of the cycle (0.5 = pure trot)
  ...
};
```
Копировать для `ServoSpeedModel {max_speed{6.0}; margin{0.8}; knee_ratio{1.0};}` (RESEARCH, «API»), `///` на публичных функциях с единицами и поведением при отказе (`minimalPeriod` возвращает `0.0`, если ничего не подходит).

**Константы и анонимный namespace в `.cpp`** (`gait.cpp` 1-16): `kCamelCase` константы, `namespace {` сразу после `namespace dog_control`, закрытие `}  // namespace`.

**Core pattern для `peakServoSpeed`** (по образцу `test_locomotion.cpp` 153-168): прогон `TrotGait::update(kDt=0.02, cmd)` по предельным командам `{0.15,0,0}, {-0.15,0,0}, {0,0.08,0}, {0,0,0.6}, {0.15,0.08,0.6}`, `inverseKinematics` на каждую ногу, разность суставов / dt, колено умножается на `knee_ratio`. Пик считается в пространстве серв: `max(hip, thigh, knee_ratio * calf)`. Сравнения с допуском `1e-9` (пик 5.077 против ворот 5.08 на грани).

**Перебор периода:** не бисекция (167 нарушений монотонности на сетке 1 мс). Копировать схему из RESEARCH «Code Examples → Поиск периода с окном»: шаг 0.0025 с, окно `guard = 0.05` с, нижняя граница `gait.min_period` (0.55), верхняя `max_period` (1.5), `return 0.0`, если не нашли. Ожидаемая таблица для теста: 3.5 → 1.030, 4.0 → 0.8975, 5.0 → 0.720, 6.0 → 0.600, ≥ 6.35 → 0.550.

**Обработка ошибок:** функция возвращает значение (`0.0`), не бросает; бросает вызывающий: узел делает `throw std::runtime_error("<what> '<value>' ...")` и `RCLCPP_FATAL` в `main` (CLAUDE.md, «Error Handling»).

**CMake:** добавить `src/servo_limits.cpp` в `add_library(dog_control_core ...)`, `dog_control/CMakeLists.txt` строки 19-25.

---

### `ros2_ws/src/dog_control/test/test_servo_limits.cpp` (test, transform)

**Analog:** `ros2_ws/src/dog_control/test/test_gait.cpp` (структура) + `test_locomotion.cpp` 147-172 (логика пика).

**Заголовок теста** (`test_gait.cpp` 1-20): `#include <gtest/gtest.h>`, std, `"dog_control/..."`, `using dog_control::X;` по одному, константы и хелперы в анонимном namespace, `constexpr double kDt = 0.02;`.
```cpp
#include <gtest/gtest.h>

#include <cmath>

#include "dog_control/gait.hpp"

using dog_control::BodyVelocity;
...
namespace
{
constexpr double kDt = 0.02;
}  // namespace

TEST(Gait, IdleDoesNotStep)
{
```
Имя набора и теста: `TEST(ServoLimits, MinimalPeriodTable)`, `TEST(ServoLimits, PeakAtShippedGaitIs5p077)` (допуск `EXPECT_NEAR(..., 1e-3)`), `TEST(ServoLimits, PeakMatchesController)` (TrotGait против LocomotionController: совпадение до 3 знаков).

**Регистрация:** в `dog_control/CMakeLists.txt:47` добавить имя в `foreach(t kinematics gait crawl greet locomotion servo_limits)`; линковка `target_link_libraries(test_${t} dog_control_core)` уже в цикле (строки 47-50).

**Локальный запуск:** вне репозитория CMake-обёртка с `add_compile_options(-Wall -Wextra -Wpedantic)` (RESEARCH «Validation Architecture»). В репозиторий её не коммитить.

---

### `ros2_ws/src/dog_control/test/test_locomotion.cpp` (правка ворот `JointSpeedsFitTheServos`)

**Analog:** он сам, строки 147-172.
```cpp
TEST(Locomotion, JointSpeedsFitTheServos)
{
  const LocomotionParams p;  // defaults == robot.yaml
  const double servo_limit = 5.5;  // rad/s, margin below servos.yaml max_joint_speed (6)
  for (const auto & cmd : std::vector<dog_control::BodyVelocity>{
      {0.15, 0.0, 0.0}, {-0.15, 0.0, 0.0}, {0.0, 0.08, 0.0}, {0.0, 0.0, 0.6}, {0.15, 0.08, 0.6}}) {
    ...
    EXPECT_LT(max_speed, servo_limit) << "cmd " << cmd.vx << "," << cmd.vy << "," << cmd.wz;
    EXPECT_EQ(c.unreachableCount(), 0);
```
Что менять: ручной режим (`gait.auto_period == false`) оставить с `5.5` (база прежняя), добавить тест авто-режима: `servo_limit = margin * max_speed` от `ServoSpeedModel`, период берётся из `minimalPeriod`. Не удалять существующий тест до замера (RESEARCH, «Где ядро»). Параметры узла не нужны: тест работает на структуре `LocomotionParams`.

---

### `ros2_ws/src/dog_control/include/dog_control/locomotion.hpp` + `src/locomotion.cpp` (правка: эффективный период и переконфигурация)

**Analog:** они сами.

**Место правки конструктора** (`locomotion.cpp` 50-56): период подставляется до построения `gait_`.
```cpp
LocomotionController::LocomotionController(const LocomotionParams & params)
: p_(params), gait_(params.gait, neutralFeet(params)), crawl_(params.crawl, neutralFeet(params)),
  greet_(params.greet, neutralFeet(params), params.stand_height, params.leg.thigh, params.leg.calf),
  survey_(params.survey)
{
  height_ = p_.lie_height;
```
Решение из Open Question 1 (RESOLVED владельцем): пересчёт **и при старте, и при смене параметра**. Для этого нужен публичный метод контроллера, который принимает новые `ServoSpeedModel`/`gait.period`/`auto_period` и пересоздаёт `gait_ = TrotGait(newParams, neutralFeet(p_))`, **только** в `Mode::PASSIVE | STAND | LYING`; иначе возвращает `false`. Стиль возврата `bool` для отклонённых запросов скопировать с `bool request(const std::string & cmd)` (`locomotion.hpp` 96, `locomotion.cpp` 80-140), `switch (mode_)` с `default` (строки 86-97, 115-130). Метод в заголовке с `///` (единицы, условия отказа), поля `LocomotionParams` (`locomotion.hpp` 39-87) получают новые поля по образцу `double slope_gain{1.0};` (`servo` модель, `bool auto_period{false}`, `double min_period{0.55}`).

**Инвариант:** `GaitParams::period{0.55}` в коде не меняется, `gait.auto_period` по умолчанию `false`, поэтому все существующие тесты и базовые прогоны идентичны.

---

### `ros2_ws/src/dog_control/src/locomotion_node.cpp` (правка проводки)

**Analogs:** сам файл (`loadParams`, 222-289) и `dog_hardware/src/servo_driver_node.cpp` (колбэк параметров, `main` с try/catch). Файл `servo_driver_node.cpp` **не править**.

**Объявление параметров** (`locomotion_node.cpp` 242-245; default берётся из структуры):
```cpp
    p.gait.period = declare_parameter("gait.period", p.gait.period);
    p.gait.duty = declare_parameter("gait.duty", p.gait.duty);
    p.gait.step_height = declare_parameter("gait.step_height", p.gait.step_height);
    p.gait.max_step = declare_parameter("gait.max_step", p.gait.max_step);
```
Копировать для `gait.auto_period`, `gait.min_period`, `servo.max_speed`, `servo.margin`, `servo.knee_ratio`. **Ловушка:** здесь `declare_parameter(name, double)` строгий по типу: целое в YAML (`max_speed: 6`) бросит исключение. Писать в YAML `6.0` или скопировать `declareNumber` (ниже).

**Терпимый к int/double чтец** (`servo_driver_node.cpp` 56-62 и 190-196):
```cpp
double asNumber(const rclcpp::Parameter & p)
{
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(p.as_int());
  }
  return p.as_double();  // throws InvalidParameterTypeException otherwise
}
...
  double declareNumber(const std::string & name, double default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;
    const auto v = declare_parameter(name, rclcpp::ParameterValue(default_value), desc);
    return asNumber(rclcpp::Parameter(name, v));
  }
```

**Колбэк параметров** (`servo_driver_node.cpp` 163-164 регистрация, 217-271 тело): регистрировать **после** `loadParams()` и создания контроллера (строки 61-62), чтобы колбэк не срабатывал на начальных `declare_parameter`.
```cpp
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return onParams(params);});
...
  rcl_interfaces::msg::SetParametersResult onParams(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      ...
      } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
        result.successful = false;
        result.reason = name + ": " + e.what();
        return result;
      }
```
Копировать: `successful=false` + `reason` вместо броска; `RCLCPP_INFO(get_logger(), "calibration %s updated", name.c_str())` как лог смены. Для locomotion: причина отказа при WALK и переходных режимах («period change rejected in mode walk»), принятие и отказ покрыть gtest на уровне контроллера. Член `rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;` рядом с членами (`servo_driver_node.cpp` ~287).

**Старт при невозможном периоде:** `main` сейчас без try/catch (`locomotion_node.cpp` 420-426):
```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dog_control::LocomotionNode>());
  rclcpp::shutdown();
  return 0;
}
```
Заменить на схему `servo_driver_node.cpp` 298-311:
```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<dog_hardware::ServoDriverNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("servo_driver"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
```
(`RCLCPP_FATAL(rclcpp::get_logger("locomotion"), ...)`, `#include <stdexcept>`).

**Обновить блок комментария** сверху файла (1-23): список топиков/параметров обязателен (CLAUDE.md); добавить строку про `servo.*` и `gait.auto_period`, лог вычисленного периода при старте: `RCLCPP_INFO`.

---

### `ros2_ws/src/dog_bench/package.xml` и `CMakeLists.txt` (config)

**Analog:** `ros2_ws/src/dog_hardware/package.xml`, `CMakeLists.txt`. Пакет без rclcpp: только `ament_cmake` + `ament_cmake_gtest`.

**package.xml** (format 3, maintainer hzname, MIT, 2.0.0; `dog_hardware/package.xml` 1-23):
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>dog_bench</name>
  <version>2.0.0</version>
  <description>Bench tools for the robot dog: servo speed test with INA219 current traces.</description>
  <maintainer email="hzname@example.com">hzname</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
Без `<depend>rclcpp</depend>` и без `<depend>dog_hardware</depend>`.

**CMakeLists.txt** (скелет `dog_hardware/CMakeLists.txt` 1-16, 93-101, 115-121, 123-137):
```cmake
cmake_minimum_required(VERSION 3.16)
project(dog_bench LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(dog_bench_core src/...cpp)
target_include_directories(dog_bench_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dog_bench_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(servo_speed_test src/servo_speed_test.cpp)
target_link_libraries(servo_speed_test dog_bench_core)

install(TARGETS dog_bench_core EXPORT export_dog_bench
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS servo_speed_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t ina219_fast ramp safety session)   # как foreach в dog_control/CMakeLists.txt:47
    ament_add_gtest(test_${t} test/test_${t}.cpp)
    target_link_libraries(test_${t} dog_bench_core)
  endforeach()
endif()

ament_export_targets(export_dog_bench HAS_LIBRARY_TARGET)
ament_package()
```
Образ робота соберёт пакет сам (`docker/Dockerfile`: `colcon build --packages-skip dog_gazebo`, перечисления нет). Запуск на роботе: `docker compose run --rm robot ros2 run dog_bench servo_speed_test ...` (как `pca9685_probe`, `docs/DEPLOYMENT.md:346`).

---

### `ros2_ws/src/dog_bench/include/dog_bench/i2c_bus.hpp` + `src/i2c_bus.cpp` (utility, file-I/O)

**Analogs:** `dog_hardware/src/power_sensor.cpp` `I2cRegs` (read-only) и интерфейс с mock `dog_hardware/include/dog_hardware/servo_bus.hpp` (13-38).

**Транзакции через `I2C_RDWR`** (адрес в каждом сообщении, без `I2C_SLAVE`; `power_sensor.cpp` 3-7, 29-58):
```cpp
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
...
  bool read16(uint8_t reg, uint16_t & out) const
  {
    uint8_t buf[2] = {0, 0};
    uint8_t r = reg;
    i2c_msg msgs[2] = {
      {static_cast<uint16_t>(addr_), 0, 1, &r},
      {static_cast<uint16_t>(addr_), I2C_M_RD, 2, buf}};
    i2c_rdwr_ioctl_data data{msgs, 2};
    if (::ioctl(fd_, I2C_RDWR, &data) != 2) {return false;}
    out = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
    return true;
  }
```
Новое для быстрого опроса: чтение шунта **без** записи указателя (один `i2c_msg` с `I2C_M_RD`), указатель INA219 сохраняется между чтениями (RESEARCH Pattern 4); каждый 8-й тик чтение шины с переключением указателя.

**Абстракция для тестов (FakeBus):** интерфейс с виртуальным деструктором и mock с счётчиками записей, как `ServoBus` + `MockBus` (`servo_bus.hpp` 13-38):
```cpp
class ServoBus
{
public:
  virtual ~ServoBus() = default;
  virtual bool setPulseUs(int channel, double us) = 0;
  ...
};
class MockBus : public ServoBus { ... double pulse(int channel) const {return pulses_.at(channel);} int writes() const {return writes_;} ... };
```
В `dog_bench` завести свой `I2cBus` (чтение/запись регистров) и `FakeI2cBus`. Все коды возврата проверять (правило для обоих пакетов, `[[nodiscard]]` у Фазы 3 не затронет свой слой).

**Запрет:** адрес 0x40 никогда не зондировать как INA (`power_sensor.cpp:109`: `if (addr == 0x40) {continue;}  // PCA9685 lives there; never write to it`); адрес INA — обязательный аргумент CLI без значения по умолчанию.

---

### `ros2_ws/src/dog_bench/include/dog_bench/ina219_fast.hpp` + `src/ina219_fast.cpp` (utility, streaming)

**Analog:** `dog_hardware/include/dog_hardware/power_sensor.hpp` 35-49 и `src/power_sensor.cpp` 17-24 (копировать формулы масштабов в собственный `namespace dog_bench::ina`, не включать `power_sensor.hpp`).
```cpp
namespace ina
{
/// INA219: 16 V range, +-320 mV shunt, 128-sample averaging, continuous.
constexpr uint16_t kIna219Config = 0x1FFF;
double ina219ShuntVolts(uint16_t raw);
double ina219BusVolts(uint16_t raw);
...
double ina219ShuntVolts(uint16_t raw) {return static_cast<int16_t>(raw) * 10e-6;}
double ina219BusVolts(uint16_t raw) {return (raw >> 3) * 4e-3;}
```
Новое: `constexpr uint16_t kIna219FastConfig = 0x199F;  // 0.1 Ohm shunt: +-3.2 A, new result every 1.064 ms` и `kIna219Fast80mv = 0x099F` (10 мОм, ±8 А), из RESEARCH «Конфигурация INA219». Регистры калибровки/тока/мощности не использовать: `I = V_shunt / R` на хосте (`power_sensor.cpp` 74-82). Запись конфигурации и read-back по образцу `power_sensor.cpp` 132-136 (`write16(0x00, cfg) && read16(0x00, v)`), но проверять именно быструю конфигурацию.

---

### `ros2_ws/src/dog_bench/include/dog_bench/pwm_out.hpp` + `src/pwm_out.cpp` (utility, file-I/O)

**Analog (read-only, Фаза 3 владеет):** `dog_hardware/src/servo_bus.cpp`, класс `Pca9685Bus`. Копировать формулы, не включать заголовок.

**Константы и расчёт тиков** (`servo_bus.cpp` 46-60, 65-78, 152-155):
```cpp
constexpr uint8_t kMode1 = 0x00;
constexpr uint8_t kLed0OnL = 0x06;
constexpr uint8_t kAllLedOnL = 0xFA;
constexpr uint8_t kPrescale = 0xFE;
constexpr uint8_t kFullOffBit = 0x10;  // in LEDn_OFF_H
...
uint16_t Pca9685Bus::ticksFor(double us, double pwm_hz)
{
  const double period_us = 1e6 / pwm_hz;
  const double ticks = std::round(us / period_us * 4096.0);
  return static_cast<uint16_t>(std::clamp(ticks, 0.0, 4095.0));
}
```
**Отпускание** (`servo_bus.cpp` 175-180): `const uint8_t buf[5] = {kAllLedOnL, 0, 0, 0, kFullOffBit}; return ::write(fd_, buf, 5) == 5;`. Этот же пятибайтный буфер использовать заранее подготовленным в обработчике аварийных сигналов через async-signal-safe `write()`.

**Отличие от `Pca9685Bus::open` (важно):** `open` (строки 80-133) **переписывает конфигурацию**, если чип спит или prescaler другой. Для `dog_bench` нужен режим «проверить, не инициализировать»: читать `MODE1` (не sleep) и `PRE_SCALE == 121` (50 Гц), иначе отказ с подсказкой `pca9685_probe check`; перед стартом прочитать `LEDn_OFF_H` всех 16 каналов и отказаться при активном чужом канале (стек запущен). `Pca9685Bus::close()` **не гасит выходы** (строки 136-141) — деструктор RAII-объекта в `dog_bench` обязан гасить явно.

---

### `ros2_ws/src/dog_bench/include/dog_bench/safety.hpp` + `src/safety.cpp` (service, event-driven)

**Analog:** `PowerGuard` в `dog_hardware/include/dog_hardware/power_sensor.hpp` 51-81 и `src/power_sensor.cpp` 145-175: окна по времени, событие один раз, повторный взвод.
```cpp
struct PowerGuardParams
{
  double overcurrent_a{5.0};     // sustained total servo current [A]
  double overcurrent_time{0.5};  // [s]
  ...
};
class PowerGuard
{
public:
  enum class Event { NONE, OVERCURRENT, UNDERVOLTAGE };
  explicit PowerGuard(PowerGuardParams p) : p_(p) {}
  /// Feed one reading taken at `now` [s]; returns an event once when a
  /// condition has held for its whole window (re-armed after it clears).
  Event update(const PowerReading & r, double now);
...
  if (current_ > p_.overcurrent_a) {
    if (over_since_ < 0.0) {over_since_ = now;}
    if (!over_fired_ && now - over_since_ >= p_.overcurrent_time) {
      over_fired_ = true;
      ev = Event::OVERCURRENT;
    }
  } else {
    over_since_ = -1.0;
    over_fired_ = false;
  }
```
Копировать структуру «параметры + класс с `update(reading, now)` + enum событий». Значения для замера (RESEARCH, Open Question 5 RESOLVED): **2.0 А дольше 50 мс** и жёсткий порог **95 % шкалы PGA** независимо от шунта; ошибки I2C ≥ 3 подряд; перерасход тика > 50 мс; `--max-seconds` 600; проверка правдоподобия пика 0.03-3 А. **Без фильтра по току** (в отличие от `filter_tau` у `PowerGuard`: сглаживание скрыло бы скачок). Шунт `--shunt-ohm` обязательный.

---

### `ros2_ws/src/dog_bench/include/dog_bench/ramp.hpp` + `src/ramp.cpp` (utility, transform)

**Analog:** чистый генератор с `update(dt)` как `TrotGait` (`gait.hpp` 33-55). Параметры рампы в `struct` с единицами: `amp_deg{25.0}` (жёсткий максимум 30), `center_us`, `us_per_rad{541.1}` для 520..2220 мкс/180°, скорость ≤ 10 рад/с, выдержка 0.4 с. Схема скоростей 1.5…10.0 возрастающая (RESEARCH Pattern 4). Исключений не бросать: валидация `std::string validate() const` (пустая строка = OK), как `ServoCalibration::validate` (`servo_driver.hpp` 91-92).

---

### `ros2_ws/src/dog_bench/include/dog_bench/session.hpp` + `src/session.cpp` (service, event-driven)

**Analog (частичный):** `ServoDriverNode` деструктор и `relax_on_exit_` (`servo_driver_node.cpp` ~172-177):
```cpp
  ~ServoDriverNode() override
  {
    if (relax_on_exit_ && driver_) {
      driver_->relax();
    }
  }
```
RAII-объект владеет выходом PWM и гасит все каналы в деструкторе. Поверх: `sigaction` для SIGINT/SIGTERM/SIGHUP/SIGQUIT выставляет `volatile sig_atomic_t`, цикл проверяет флаг каждый тик; для SIGSEGV/SIGABRT/SIGBUS/SIGFPE обработчик делает один `write()` пяти байт `ALL_LED_OFF` в заранее открытый дескриптор и `_exit(3)`. SIGKILL и потеря питания не перехватываются (PCA9685 держит последний импульс): вторая линия защиты — рука на выключателе (D-10). Сессия зависит от `I2cBus` (интерфейс), а не от `/dev/i2c-*`, чтобы gtest проходил на `FakeI2cBus`: гарантированное отпускание при любом выходе, отпускание по превышению тока, отказ при чужом канале.

---

### `ros2_ws/src/dog_bench/src/servo_speed_test.cpp` (CLI `main`, batch)

**Analogs:** `dog_hardware/src/pca9685_probe.cpp` (разбор аргументов, usage, режимы) и `servo_driver_node.cpp` 298-311 (try/catch, код выхода).

**Заголовок с перечнем команд** (`pca9685_probe.cpp` 1-7) и `usage()` с кодом 2 (19-28), разбор `--flag value` в цикле (33-42):
```cpp
int main(int argc, char ** argv)
{
  std::string device = "/dev/i2c-0";
  int address = 0x40;
  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--device" && i + 1 < argc) {device = argv[++i];}
    else if (a == "--address" && i + 1 < argc) {address = std::stoi(argv[++i], nullptr, 0);}
    else {args.push_back(a);}
  }
  if (args.empty()) {return usage();}
```
Диапазонные проверки аргументов с сообщением и кодом 2 (строки 59-62):
```cpp
    if (us < 500 || us > 2500) {
      std::cerr << "pulse must be within 500..2500 us\n";
      return 2;
    }
```
Режимы: `selftest` (только INA, PWM не включается), `dry-run` (печать плана), `run`. Обязательные `--ina-address`, `--shunt-ohm`, `--channel`; `--amp-deg` ≤ 30 (по умолчанию 25), `--center-us` 1000-1800, `--yes` не по умолчанию (подтверждение «питание проверено мультиметром, нога свободна, рука на выключателе? (YES)»). Код возврата ≠ 0 при любой I2C-ошибке; CSV `t_s, stroke_id, direction, cmd_us, shunt_raw, bus_raw` и причина остановки.

---

### `ros2_ws/src/dog_bench/test/test_{ina219_fast,ramp,safety,session}.cpp` (test)

**Analog:** `ros2_ws/src/dog_hardware/test/test_power.cpp` (1-60).

**Структура** (`test_power.cpp` 1-26): `TEST(Набор, Имя)`, эталонные числа с комментарием единиц, недоступный путь без железа:
```cpp
TEST(InaRegisters, Scaling)
{
  // INA219: 10 uV/LSB shunt, bus in bits 15..3 at 4 mV.
  EXPECT_NEAR(ina::ina219ShuntVolts(1000), 0.010, 1e-12);
  EXPECT_NEAR(ina::ina219BusVolts(1500 << 3), 6.0, 1e-12);
  ...
}
TEST(InaProbe, MissingBusReturnsNull)
{
  std::string found;
  auto s = probePowerSensor("/dev/i2c-does-not-exist", {0x41}, "auto", 0.01, found);
  EXPECT_EQ(s, nullptr);
```
Спайки не срабатывают / длительное превышение срабатывает ровно один раз (`test_power.cpp` 42-60: `SpikesDoNotTrip`, `SustainedStallTripsOnce`) — копировать как шаблон для `safety`. Время в тестах задаётся числом (`i * 0.01`), без `sleep` и стенных часов.

---

### `ros2_ws/src/dog_description/dog_description/servo_profile.py` (utility, transform)

**Analog:** `tools/autocal/robotdog_autocal/servo_model.py` (чистый модуль-порт с docstring, `@dataclass`) + приватные хелперы `urdf.py` 61-82.

**Шапка модуля** (`servo_model.py` 1-16): docstring на первой строке — что это и чем проверено; `import math`; `from dataclasses import dataclass`; приватные `_helper` с подчёркиванием.
```python
"""Servo pulse <-> joint angle model, identical to dog_hardware/servo_driver.cpp.

Kept in sync by tests/test_servo_model.py, which checks reference values
produced by the C++ implementation.
"""

import math
from dataclasses import dataclass, field, fields
```
Состав нового модуля (RESEARCH «Pattern 1»): `ServoProfile` (`@dataclass`: `backlash_deg`, `delay_ms`, `friction_nm`, `bus_voltage`, `bus_voltage_ref`, `from_params(dict)` по образцу `ServoCal.from_params`, строки 115-126), `BacklashPlay(width_rad)`, `DelayLine(delay_s)` (очередь по времени симуляции, фальшивые часы в тестах), `speed_factor(V)`, `torque_factor(V)` (зажим V в [4.8, 6.6]). Без `rclpy` и без `yaml`: импортируется в pytest. Комментарии «измерено/источник» для каждого номинала (D-16).

**Код `BacklashPlay` для копирования** (RESEARCH «Мост: люфт»):
```python
class BacklashPlay:
    """Output follows the command only after the play `width` is used up (width in rad)."""
    def __init__(self, width):
        self.half = 0.5 * width
        self.y = None
    def __call__(self, x):
        if self.y is None:
            self.y = x
        elif x - self.y > self.half:
            self.y = x - self.half
        elif self.y - x > self.half:
            self.y = x + self.half
        return self.y
```
Формулы просадки: `speed_factor(V) = 1 + 0.1471 * (V - V_ref)`, `torque_factor(V) = 1 + 0.1212 * (V - V_ref)`.

**Размещение:** `dog_description` (а не `dog_gazebo`), потому что тесты `dog_gazebo` в CI не запускаются (`colcon test --packages-skip dog_gazebo`, `ci.yml:32`), а мост уже импортирует оттуда (`joint_command_bridge.py:10`).

---

### `ros2_ws/src/dog_description/dog_description/urdf.py` (правка: `body_com_x`, `servo_model`, `<dynamics>`, `servo_sim`)

**Analog:** сам файл.

**Конфиг** (18-36): новый ключ в `DEFAULT_DESCRIPTION`, затем `desc.update(params.get('description', {}))` подхватит старые YAML без ключа; `servo_sim` кладётся точно как `desc['sensors']`:
```python
DEFAULT_DESCRIPTION = {
    'body_length': 0.23, ... 'servo_effort': 1.1, 'servo_velocity': 6.0, 'sim_p_gain': 25.0,
    ...
}
def load_config(path):
    ...
    desc = dict(DEFAULT_DESCRIPTION)
    desc.update(params.get('description', {}))
    desc['sensors'] = dict(params.get('sensors', {}))
    return params['geometry'], desc
```
Добавить `'body_com_x': 0.0` в `DEFAULT_DESCRIPTION` и `desc['servo_sim'] = dict(params.get('servo_sim', {}))`: кортеж `(geometry, description)` и все вызывающие не меняются.

**Центр масс корпуса** (строка 113; `_inertial` уже принимает `xyz`, строки 71-77):
```python
        + _inertial(d['body_mass'], _box_inertia(d['body_mass'], bl, bw, bh)) + '</link>')
...
def _inertial(m, ixyz, xyz=(0, 0, 0)):
```
станет `_inertial(d['body_mass'], _box_inertia(...), (d['body_com_x'], 0, 0))`; визуал и коллизия остаются в центре бокса; при `0.0` вывод идентичен прежнему (`0.0000 0.0000 0.0000`).

**Трение и множители скорости/момента** (131, 138, 148, 158, 184, 198-210): сейчас
```python
    eff, vel = d['servo_effort'], d['servo_velocity']
...
            + _limit(d['hip_limits_deg'], eff, vel) + '</joint>')
...
        out.append(_gazebo_extras(namespace, d['sim_p_gain'], d['servo_velocity'], initial))
...
            f'<cmd_max>{vmax}</cmd_max><cmd_min>{-vmax}</cmd_min></plugin></gazebo>')
```
При `servo_model='real'`: `eff` и `vel` умножаются на `torque_factor`/`speed_factor` (для `vmax` тоже), в каждый revolute-`<joint>` добавляется `<dynamics damping="0" friction="f"/>`. При `'ideal'` — ничего из этого, вывод байт-в-байт прежний (тест). Для колена с тягой: `vel/knee_ratio`, `effort*knee_ratio` (Open Question 8 RESOLVED: драйвер не править).

**Сигнатура** (95): `def build_urdf(geometry, description=None, gazebo=False, namespace='dog', initial=None):` → добавить `servo_model='ideal'` последним именованным аргументом.

**Комментарий `sim_p_gain`:** в velocity-режиме параметр не используется. Правится текст комментария в `robot.yaml:232` и фраза в `docs/SIMULATION.md:3`, **значение не удалять** (иначе пропадёт из загрузки `urdf.py:21`).

---

### `ros2_ws/src/dog_description/test/test_servo_profile.py` и правки `test_urdf.py` (test)

**Analog:** `ros2_ws/src/dog_description/test/test_urdf.py` (1-100).

**Структура** (1-12, 62-82): one-line docstring модуля (CLAUDE.md), `import xml.etree.ElementTree as ET`, `from dog_description.urdf import build_urdf, joint_names, load_config`, фиксированный `GEOM`, функции `test_*` без классов, config берётся из реального `robot.yaml`:
```python
"""The generated URDF must agree with the kinematics used by locomotion."""
...
GEOM = {'hip_offset': 0.055, 'thigh': 0.105, 'calf': 0.105, 'hip_x': 0.09, 'hip_y': 0.06}
...
def test_gazebo_extras_and_shipped_config():
    here = os.path.dirname(__file__)
    cfg = os.path.join(here, '..', '..', 'dog_bringup', 'config', 'robot.yaml')
    geometry, description = load_config(cfg)
    urdf = build_urdf(geometry, description, gazebo=True)
    root = ET.fromstring(urdf)
    plugins = root.findall('gazebo/plugin')
    assert sum('JointPositionController' in p.get('name') for p in plugins) == 12
```
Новые кейсы: `body_com_x` → `origin` у `trunk/inertial` (и URDF неизменный по умолчанию); `ideal` без `<dynamics>` и байт-в-байт равен выводу до правки (сравнивать со строкой, построенной с `description` без ключей профиля); `real` → 12 `dynamics friction="0.06"`, `limit velocity` и `cmd_max` пересчитаны; `BacklashPlay` (мёртвая зона, реверс, монотонность, начальное состояние), `DelayLine` (порядок, тайминг на фальшивых часах), факторы просадки 4.8/5.2/6.0/6.6 В. Детерминизм: `np.random.default_rng(0)` как в строке 52. Запуск: `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_description/test`.

---

### `ros2_ws/src/dog_gazebo/dog_gazebo/joint_command_bridge.py` (правка: ступени люфта и лага)

**Analog:** сам файл (1-42). Сейчас пересылка без преобразований:
```python
class JointCommandBridge(Node):
    def __init__(self):
        super().__init__('joint_command_bridge')
        ns = self.declare_parameter('robot_namespace', 'dog').value
        self.pubs = {j: self.create_publisher(Float64, sim_command_topic(ns, j), 10)
                     for j in joint_names()}
        self.create_subscription(JointState, 'joint_commands', self.on_commands, 10)

    def on_commands(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            pub = self.pubs.get(name)
            if pub is not None:
                pub.publish(Float64(data=float(pos)))
```
Правка: параметры `backlash_deg` (0 = выкл), `delay_s` (0 = выкл) через `self.declare_parameter('backlash_deg', 0.0).value`; на каждый сустав свой `BacklashPlay`; `DelayLine` с таймером 500 Гц по времени узла (`use_sim_time` уже передан в `sim.launch.py:106-107`). При нулях поведение **идентично** нынешнему (идеальная модель, D-15). Импорт `from dog_description.servo_profile import BacklashPlay, DelayLine` рядом с `from dog_description.urdf import ...` (строка 10). `main()` (28-38) не менять. Шапка docstring (1-3) обновляется: узел теперь может вносить люфт и задержку.

---

### `ros2_ws/src/dog_gazebo/launch/sim.launch.py` (правка: `servo_model`, `servo_speed`, `servo_delay_ms`)

**Analog:** сам файл; правки только в образец уже существующих аргументов.

**Хелперы `cfg()`/`on()`** (строки 60-62) — использовать, не заводить новые:
```python
def _setup(context):
    cfg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    on = lambda name: cfg(name).lower() in ('1', 'true', 'yes')  # noqa: E731
```
**Переопределение из аргумента** (строки 83, 89-90, образец для `servo_speed`):
```python
    overrides = {'slope.compensation': on('slope_compensation'), 'heading.hold': on('heading_hold')}
    ...
    if cfg('step_height'):
        overrides['gait.step_height'] = float(cfg('step_height'))
```
**Вызов `build_urdf`** (81): `urdf = build_urdf(geometry, description, gazebo=True, namespace=NS, initial=initial)` → добавить `servo_model=cfg('servo_model')`; `description['servo_velocity']` переопределяется, если задан `servo_speed` (физическая скорость, D-13; предполагаемая `servo.max_speed` не трогается).

**Мост** (106-107): `Node(package='dog_gazebo', executable='joint_command_bridge', namespace=NS, parameters=[sim_time])` → добавить словарь `{'backlash_deg': ..., 'delay_s': ...}` (нули при `ideal`).

**Объявление аргумента** (163-164):
```python
        DeclareLaunchArgument('step_height', default_value='',
                              description='override gait.step_height [m] (robot.yaml by default)'),
```
Добавить `DeclareLaunchArgument('servo_model', default_value='ideal', description='ideal | real (backlash, delay, friction, bus voltage)')`, `servo_speed` (пусто = из YAML), `servo_delay_ms`. Аргументы `heading_hold` (161-162) и `slope_compensation` (159-160) для режима B уже есть.

---

### `ros2_ws/src/dog_gazebo/dog_gazebo/walk_check.py` (правка: `--maneuvers`, `--backward-speed`, `dyaw5_deg`, вынос `parse_args`)

**Analog:** сам файл.

**`dyaw5_deg` — ловушка:** `maneuver()` берёт `dyaw` **после** 1.5 с выбега (строки 171-177), то есть за 6.5 с, а критерий «за 5 с».
```python
        tilt = self.spin(seconds, lambda: self.vel.publish(t))
        # release the stick like an operator: zero twist, then coast to a stop
        tilt = max(tilt, self.spin(1.5, lambda: self.vel.publish(Twist())))
        p1 = self.odom.pose.pose.position
        ...
        dyaw = self.yaw_unwrapped - yaw0
```
Записать `dyaw5 = self.yaw_unwrapped - yaw0` сразу после первого `self.spin(seconds, ...)`, до выбега, и добавить `dyaw5_deg=math.degrees(dyaw5)` в `**values` вызова `self.check(...)` (184-187); существующие ключи (`dx`, `dy`, `dyaw_deg`, `ratio`, `tilt_deg`, `z`) не менять: их читают `tools/sim_video/report/*` и `terrain_sweep`.

**Добавление опций** (образец `--backward-ratio`, строки 251-254):
```python
    ap.add_argument('--backward-ratio', type=float,
                    help='... for the backward manoeuvre (default: --min-ratio); 0 = only no fall, '
                    'no tilt, no sagging (backward on uneven ground is the weak manoeuvre, TERRAIN.md)')
```
`--maneuvers` (список подмножества; по умолчанию все; при подмножестве `lie` пропускается), `--backward-speed` (модуль, по умолчанию 0.10; манёвр строки 223: `self.maneuver('backward', -0.10, 0, 0, T, ('x', -0.10 * T))` берёт скорость из аргумента). Команда push-CI `walk_check --backward-ratio 0.2` не меняется.

**Тестируемость:** `import rclpy` и `from geometry_msgs...` стоят на уровне модуля (18-30), pytest без ROS не импортирует файл. Вынести `parse_args(argv)` в функцию и чистые хелперы (выбор манёвров), а импорты `rclpy`/сообщений перенести внутрь `main()`/класса, либо держать `parse_args` в отдельном лёгком модуле. Тест `test_walk_check_args.py` не должен требовать rclpy.

**Итог проверки:** 10 проверок на ровном полу (`stand`, 8 манёвров, `lie`), строка `print('%d/%d passed' ...)` (242) выводит «10/10»; «8/8» в документах устарело (см. секцию docs).

---

### `ros2_ws/src/dog_gazebo/dog_gazebo/acceptance.py` (CLI, batch)

**Analog:** `ros2_ws/src/dog_gazebo/dog_gazebo/terrain_sweep.py` (целиком, 102 строки). Он не импортирует rclpy, поэтому `acceptance.py` импортируется в pytest.

**Импорт переиспользуемого** (Don't Hand-Roll: не писать свой запускатель):
```python
from dog_gazebo.terrain_sweep import never_stood, run_level
```
**Свежая симуляция на повтор и перезапуск при «не встал»** (`terrain_sweep.py` 84-94):
```python
        row = run_level(args.terrain, level, args.seed, args.domain + k % 5, extra,
                        args.launch_arg, keep, sim_log)
        if never_stood(row):
            # before any test: the simulation did not come up whole - once more
            print('robot never left passive - relaunching the simulation once', flush=True)
            row = run_level(args.terrain, level, args.seed, args.domain + k % 5, extra,
                            args.launch_arg, keep, sim_log.replace('.sim.log', '.retry.sim.log'))
            row['relaunched'] = True
```
`run_level(kind, level, seed, domain, extra, launch_args, keep, sim_log)` (30-63): `extra` — хвост командной строки `walk_check` (сюда идут `--maneuvers backward,left,right --backward-speed ...`), `launch_args` — хвост `sim.launch.py` (`heading_hold:=false`, `slope_compensation:=false`, `servo_model:=real`). Сам формирует `ROS_DOMAIN_ID`/`GZ_PARTITION=f'sweep{domain}'` (31), убивает группу процессов (52-63). Для приёмки передавать домен `80 + i % 10` (60-77 заняты job `terrain`, 41-43 launch-тесты).

**CLI-скелет** (`terrain_sweep.py` 66-77, 95-98): `argparse` с `formatter_class=RawDescriptionHelpFormatter` и `description=__doc__`, `--out` обязателен, `--launch-arg action='append'`, `sim_log = f'{os.path.splitext(args.out)[0]}_{level:g}.sim.log'`, запись JSON после каждой строки, конец `sys.exit(0 if ok else 1)`:
```python
        rows.append(row)
        with open(args.out, 'w') as f:
            json.dump(rows, f, indent=1)
    ok = all(r.get('results') and all(x['ok'] for x in r['results']) for r in rows)
    sys.exit(0 if ok else 1)
```
Схема результата `schema: 1` (distro, servo_model, repeats, git_sha, cells, scoring, verdict): RESEARCH «Pattern 3». Статусы повтора `ok | fell | no_stand | error`; `no_stand` не считается падением, добавляется замена до 2·n попыток. Строки `PASS`/`FAIL` с `print(..., flush=True)`; `summary.md` для `$GITHUB_STEP_SUMMARY`. Docstring модуля — как у `terrain_sweep` (пример запуска, что делает, куда пишет лог).

**Правило безопасности:** значения `${{ inputs.* }}` в `run:` не подставлять, только через `env:` (ASVS V5).

---

### `ros2_ws/src/dog_gazebo/dog_gazebo/acceptance_stats.py` (utility, transform)

**Analog:** `terrain_sweep.never_stood` (чистая функция над JSON, строки 23-27) и `@dataclass` для значений из `tools/autocal/robotdog_autocal/fit.py` (`FitResult`).
```python
def never_stood(data):
    """The robot stayed passive: the stand request or the clock never reached
    locomotion (DDS on a busy runner), before any manoeuvre was tried."""
    r = data.get('results') or []
    return bool(r) and r[0]['name'] == 'stand' and not r[0]['ok'] and 'state=passive' in r[0]['detail']
```
Чистый модуль без rclpy/numpy: `statistics.median`, `min` (Don't Hand-Roll). Содержимое: классификация статуса повтора, `summarize(runs)` (n, n_invalid, ratio_min, ratio_median, falls, max_abs_dyaw5_deg, pass, reasons), правила A/B/Lyrical (D-01..D-07), `derive_push_threshold`, сборка JSON схемы 1, markdown-сводка.

**Код порога D-05** (RESEARCH «Code Examples»):
```python
import math

def derive_push_threshold(min_ratio, lo=0.2, hi=0.4, step=0.05, margin=0.8):
    """Push-CI --backward-ratio from the minimum over >= 5 repeats (D-05)."""
    raw = math.floor(round(margin * min_ratio / step, 9)) * step
    return round(min(hi, max(lo, raw)), 2)
# 0.52 -> 0.40 ; 0.45 -> 0.35 ; 0.30 -> 0.20 (floor)
```
Константа высоты корпуса брать не копированием числа, а с комментарием на `walk_check.MIN_BODY_HEIGHT = 0.108` (walk_check.py:52); вердикт с `n < 5` не выдаётся как зачётный. Python 3.12 и 3.14: без нового синтаксиса.

---

### `ros2_ws/src/dog_gazebo/test/` (`test_acceptance_stats.py`, `test_walk_check_args.py`) (test)

**Analog:** `ros2_ws/src/dog_web/test/test_protocol.py` (чистый Python-модуль пакета, pytest, без ROS) и `dog_description/test/test_urdf.py`.
```python
import json

import pytest

from dog_web import protocol

LIM = protocol.Limits()


def test_drive_is_scaled_and_clamped():
    a = protocol.handle_message('{"type":"drive","vx":1,"vy":-0.5,"wz":7}', LIM)
    assert not a.errors
    assert a.twist == pytest.approx((LIM.max_vx, -0.5 * LIM.max_vy, LIM.max_wz))
```
Каталога `test/` у `dog_gazebo` нет: создать (без `__init__.py`, как у `dog_web/test`, `dog_description/test`). Запуск: `PYTHONPATH=ros2_ws/src/dog_gazebo python3 -m pytest -q ros2_ws/src/dog_gazebo/test`. Тесты: min/median, правила A/B/Lyrical, классификация статусов, `derive_push_threshold` на границах 0.2/0.4/округлении (`0.52→0.40`, `0.45→0.35`, `0.30→0.20`), ключи схемы JSON, разбор аргументов `walk_check`, формирование командных строк `acceptance` без запуска. Детерминизм, без сна и стенных часов. **Ловушка:** `colcon test` в CI идёт с `--packages-skip dog_gazebo`, поэтому эти тесты запускать первым шагом job `acceptance` и локально.

---

### `ros2_ws/src/dog_gazebo/setup.py` (правка)

**Analog:** сам файл (23-30) и `ros2_ws/src/dog_description/setup.py:19`.
```python
    entry_points={'console_scripts': [
        'joint_command_bridge = dog_gazebo.joint_command_bridge:main',
        'walk_check = dog_gazebo.walk_check:main',
        'terrain_sweep = dog_gazebo.terrain_sweep:main',
        ...
    ]},
```
добавить `'acceptance = dog_gazebo.acceptance:main',`; `extras_require={'test': ['pytest']},` по образцу `dog_description/setup.py:19` (нужно на Python 3.14). `package.xml` у `dog_gazebo` — добавить `<test_depend>python3-pytest</test_depend>` как у `dog_description/package.xml:33`. Модуль `acceptance_stats` попадёт в `packages=[package_name]` автоматически.

---

### `tools/robot_setup/robot_setup.py` (правка: `body_com_x`, проверка колена, `sensor_checks`)

**Analog:** сам файл.

**Новое поле формы** (кортеж `(id, label, unit, section, key, scale file->form, min, max, help)`, группа `body`, строки 49-58):
```python
        ('foot_radius', 'Радиус стопы', 'мм', 'description', 'foot_radius', 1000, 0, 40,
         'резиновый наконечник; для модели'),
```
Добавить `('body_com_x', 'body_com_x — центр масс корпуса вперёд от центра', 'мм', 'description', 'body_com_x', 1000, -60, 60, 'баланс корпуса без ног на пруте, ±3 мм; «+» вперёд')`. **Ловушка:** `set_nested` (194-216) бросает `KeyError`, если ключа нет в файле, поэтому ключ должен быть в `robot.yaml` (он будет); `load()` (141-) пропустит отсутствующий ключ, и `validate` даст «не заполнено».

**Предупреждение в `validate`** (стиль строк 308-313, текст по-русски): `if abs(com_x) > hip_x: out.append(('warn', '...'))`.

**Проверка сумм масс** (315-319) и **колено** (297-301) уже есть, не дублировать; текст «лучше 60–100°» при пороге `60 <= -q2 <= 110` — не менять.

**Порт четырёхшарнирника для `servo.knee_ratio`:** копировать не из C++, а из `tools/autocal/robotdog_autocal/servo_model.py` `Linkage` (19-90, уже сверён с C++ тестом) или написать минимальный порт рядом с `linkage_closes` (250-261). Тест сравнивает с золотыми значениями из C++ для `servo_arm 15, joint_arm 20, rod 95, axis 95`: скорость сервы в раз больше суставной 1.605 (−30°), 1.427 (−20°), 1.357 (−10°), 1.334 (0°), 1.344 (+10°), 1.392 (+20°), 1.507 (+30°); максимум в рабочем диапазоне колена ≈ 1.40. `--check` предупреждает, если `servo.knee_ratio` меньше вычисленного.

**Ловушка sensor_checks** (351-392): на шаблонных значениях несуществующих датчиков (`gs2`, ToF, лидары) функция даёт **ошибки** `out.append(('error', 'GS2 не достаёт до пола ...'))`, при смене `stand_height`/`hip_x`/`hip_y` они блокируют `save` и `--check` (код 2, его запускает CI job `robot-setup`). Начало функции:
```python
def sensor_checks(g, out, info):
    """Where the sensors meet the floor in the stand pose (all in mm, body frame)."""
    floor = -g['stand_height']
    foot_x, foot_y = g['hip_x'], g['hip_y'] + g['hip_offset']
    if int(g['gs2']):
```
Решение: понизить до `('warn', ...)` при флагах `x_lidar/gs2/tof = 0` (уровень предупреждения при `sensors.*: false`), плюс тест.

**Инвариант трёх скоростей:** в закоммиченных YAML `description.servo_velocity == servo.max_speed == max_joint_speed` (тест в `tools/robot_setup/test`, job `robot-setup`). Phase 2 отступает только аргументом запуска.

---

### `tools/robot_setup/test/test_robot_setup.py` (правка)

**Analog:** сам файл (1-114): fixture `cfg` копирует реальные YAML во временную папку, `import robot_setup as rs  # noqa: E402`, проверка русских подстрок.
```python
@pytest.fixture
def cfg(tmp_path):
    for f in ('robot.yaml', 'servos.yaml'):
        shutil.copy(os.path.join(rs.CONFIG, f), tmp_path / f)
    return str(tmp_path)

def test_validation_catches_what_would_break_the_robot():
    base = rs.load(rs.CONFIG)
    v = dict(base, stand_height=260)  # longer than the leg
    assert any('нога достаёт' in t for lv, t in rs.validate(v)[0] if lv == 'error')
```
Новые кейсы: `body_com_x` загрузка, round-trip без изменений (`test_current_config_is_valid_and_roundtrips_unchanged` уже даст регрессию, когда ключ попадёт в `robot.yaml`), сохранение с комментариями, предупреждение при `|com_x| > hip_x`; knee-ratio против золотых значений; инвариант трёх скоростей; отсутствие ошибок датчиков при смене `stand_height`.

---

### `tools/servo_speed/analyze.py`, `synth.py`, `tests/`, `README.md` (utility + test)

**Analog:** `tools/autocal/` — структура автономного инструмента без ROS.

- Каталог и тесты: `tools/autocal/tests/conftest.py` (1-4) добавляет родительскую папку в `sys.path`:
  ```python
  import os
  import sys

  sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
  ```
- Тест-стиль: `tools/autocal/tests/test_servo_model.py` (эталонные значения из C++ в словаре с комментарием «Reference values printed by ...», `pytest.approx`, `@pytest.mark.parametrize`) и `test_fit.py` (детерминированный шум `np.random.default_rng(0)`).
- Зависимости: `tools/autocal/requirements.txt` (`numpy>=1.24`); matplotlib только внутри функции `--plot`, тесты без него.
- README на русском, формат как `tools/autocal/README.md` и `tools/robot_setup/README.md`: заголовок, блок установки, `bash`-команды, таблица.
- Docstring модулей на английском, первая строка — сводка (CLAUDE.md).

Содержимое (RESEARCH Pattern 4): `analyze.py` — вход CSV `t_s, stroke_id, direction, cmd_us, shunt_raw, bus_raw`, выход JSON (`v_sat`, `v_dur_plateau`, согласие, плато, шумовой пол), порог `ε = max(3·σ_rep, 0.02·I_плато)` (относительные единицы, не зависит от шунта), кросс-проверка длительностью хода (согласие в пределах 15 %, при расхождении меньшее и флаг). `synth.py` — детерминированный (фиксированный seed) генератор: P-контур с ограничением скорости, `I = I0 + a·|θ̇| + b·|θ̈|`, шум 8 мА, квантование 0.1 мА, дрожь старта 0-20 мс. Тесты: излом найден в пределах одной ступени сетки при `v_max ∈ {3.5, 4.5, 6.0, 7.5}`; шум 8-30 мА; **инвариантность к масштабу шунта** (токи ×10, тот же `v_sat`); сетка без насыщения → «не насыщено»; согласие `v_dur` и `v_sat`; рассинхрон старта. Запуск: `python3 -m pytest -q tools/servo_speed/tests`.

---

### `.github/workflows/ci.yml` (правка: inputs, job `acceptance`, job `servo-speed`)

**Analog:** сам файл. Правки маленькие и аддитивные (Фаза 3 правит тот же файл).

**`workflow_dispatch` с inputs** (строки 3-7):
```yaml
on:
  push:
    branches: [main, master]
  pull_request:
  workflow_dispatch:
```
→ `workflow_dispatch:` получает `inputs` (`acceptance: boolean default false`, `repeats: string default '5'`, `cells: choice`). **Ловушка:** у остальных job'ов нет `if`, поэтому ручной запуск ветки прогоняет и их тоже (~15 мин), это допустимо; push в ветку фазы CI не запускает (`branches: [main, master]`); полный CI на ветке — `gh workflow run ci.yml --ref <ветка> -f acceptance=true -f repeats=5`. Новый отдельный workflow-файл не годится (404 для непринятого в `main` файла).

**Каркас нового job** — копировать job `simulation` (68-99): контейнер `osrf/ros:${{ matrix.distro }}-simulation`, `defaults.run.shell: bash`, `working-directory: ros2_ws`, шаг сборки, `timeout-minutes`, `upload-artifact@v4`:
```yaml
  simulation:
    name: gazebo walk check (${{ matrix.distro }})
    runs-on: ubuntu-24.04
    strategy:
      fail-fast: false
      matrix:
        distro: [jazzy, lyrical]
    container:
      image: osrf/ros:${{ matrix.distro }}-simulation
    defaults:
      run:
        shell: bash
        working-directory: ros2_ws
    steps:
      - uses: actions/checkout@v4
      - name: Build
        run: |
          source /opt/ros/${{ matrix.distro }}/setup.bash
          colcon build
```
Добавки для `acceptance`: `if: github.event_name == 'workflow_dispatch' && inputs.acceptance`, `strategy.matrix: {distro: [jazzy, lyrical], servo_model: [ideal, real]}`, `permissions: contents: read`, `timeout-minutes: 90`, `env: REPEATS: ${{ inputs.repeats }}`; первым шагом pytest чистых модулей; затем `ros2 run dog_gazebo acceptance --repeats "$REPEATS" --servo-model ${{ matrix.servo_model }} --domain 80 --out acceptance_${{ matrix.distro }}_${{ matrix.servo_model }}.json`; `actions/upload-artifact@v4` (`if: always()`, образец строки 205-212: `path: ros2_ws/*.json` и `*.sim.log`), вывод в `$GITHUB_STEP_SUMMARY`. Значения входов только через `env:` (не `${{ inputs.* }}` в `run:`). Шаг `gz sdf -p` на сгенерированном URDF и `grep friction` — проверка, что `<dynamics>` доходит до SDF (`[ASSUMED]`: CLI есть в образе).

**Правка порога `--backward-ratio`** (92-94), последним отдельным коммитом:
```yaml
          # backward is the weak manoeuvre (TERRAIN.md): it varies widely run to
          # run; on the flat the default 40 % failed once at 39 %
          ros2 run dog_gazebo walk_check --backward-ratio 0.2
```
Значение — выражением по `matrix.distro` из `derive_push_threshold`; комментарий со ссылкой на замер и `docs/TERRAIN.md` (пороги ослабляются/меняются только с комментарием).

**Job для `tools/servo_speed`** — копировать `robot-setup` (55-66):
```yaml
  robot-setup:
    name: robot parameter form (tools/robot_setup)
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: '3.12'
      - run: pip install pyyaml pytest
      - run: python -m pytest -q tools/robot_setup/test
```
(`pip install numpy pytest`, `python -m pytest -q tools/servo_speed/tests`). Тесты `dog_bench` идут в существующем `build-test` без правки `ci.yml` (`colcon test --packages-skip dog_gazebo`).

---

### `ros2_ws/src/dog_bringup/config/robot.yaml` и `servos.yaml` (config)

**Analog:** сами файлы.

**Формат** (блок `gait`, строки 30-34; `description`, 220-235): ключ в одну строку `key: value  # comment`, колонки комментариев выровнены, единицы в скобках, объяснение неочевидного над ключом:
```yaml
    gait:
      period: 0.55          # full trot cycle [s]
      duty: 0.65            # stance fraction (0.5 = pure trot, rocks ~15 deg in sim)
      step_height: 0.02     # swing apex [m]
      max_step: 0.06        # longest stance stroke [m] -> vx cap 0.17 m/s
...
    description:
      ...
      servo_effort: 1.1     # [N*m] MG996R ~11 kg*cm @ 6 V
      servo_velocity: 6.0   # [rad/s] MG996R 0.14 s/60deg @ 6 V no-load, ~6 loaded
      sim_p_gain: 25.0      # Gazebo joint position loop gain [1/s]
```
Новые ключи (значения по умолчанию в коде должны совпасть):
- `gait.auto_period: false  # ...` (true вместе с измеренными числами), `gait.min_period: 0.55`;
- блок `servo:` (`max_speed: 6.0`, `margin: 0.8`, `knee_ratio: 1.0`) рядом с `gait`;
- блок `servo_sim:` (схема в RESEARCH «Единая схема YAML»): `backlash_deg: 1.5`, `delay_ms: 40.0`, `friction_nm: 0.06`, `bus_voltage: 6.0`, `bus_voltage_ref: 6.0`; **источник каждого числа помечается в комментарии** (D-16: середина 1–2° из `docs/TERRAIN.md`, середина 30–50 мс из GAIT-05, трение ≈ 5 % от момента заклинивания);
- `description.body_com_x: 0.0  # [m] trunk centre of mass forward of the body centre (measured <date>; 0 = at the centre)`.
Вместо пометки «v1: …» писать «измерено <дата>» (CONTEXT, Established Patterns). Поправить комментарий `sim_p_gain` (параметр в velocity-режиме не используется), значение не менять. Не нарушать регулярки `set_nested` (`key: value  # comment`, один ключ в строке, секция `name:` без значения на строке).

**servos.yaml:** единственная правка — `max_joint_speed: 6.0        # slew limit [rad/s]` (строка 28) после замера, по образцу существующего комментария; делать после слияния Фазы 3 или принять тривиальный конфликт.

---

### Документация `docs/*` и `README.md` (правки, русский язык)

**Analogs:** сами файлы. Точки правок (проверены grep):
- «8/8» → «10/10 (stand, 8 манёвров, lie)» в `docs/SIMULATION.md:42`, `docs/DEPLOYMENT.md:244` и `:250`, `docs/TERRAIN.md:80` (таблица результатов, смотреть контекст), `README.md:88`.
- `docs/SIMULATION.md:3`: «позиционный регулятор» → идеальный ограничитель скорости без запаздывания (по исходникам gz-sim: `targetVel = -error / dt`, `p_gain` не используется); добавить раздел про `servo_model:=real` (как каждый эффект действует, что проверяемо только в CI).
- `docs/HARDWARE.md`: ограничение походки ≤ 5.1 рад/с → измеренное после замера.
- `docs/DEPLOYMENT.md`: «Этап 1» — добавить строки про `body_com_x` и четыре длины тяги колена; критерий перехода «10/10».
- `docs/REVIEW.md`: таблица «Открыто» с колонками `# | Проблема | Чем грозит | Предлагаемое решение | Почему так` (строка 23 — заголовок; нумерация продолжает последний пункт): записать пункт «драйвер ограничивает скорость в пространстве сустава (`servo_driver.cpp:226`), для колена с тягой серва быстрее в 1.3–1.6 раза» с пометкой для Фазы 3 (Open Question 8 RESOLVED); открытые вопросы — сюда, не TODO в коде.

---

### `.planning/phases/01-…/01-MEASUREMENT-SHEET.md` (docs)

**Analog:** `docs/DEPLOYMENT.md` «Этап 1» (79-254) + кортежи `GROUPS` в `tools/robot_setup/robot_setup.py` 38-131. Формат: таблица «поле / единица / как мерить / ключ YAML / схема из `docs/img`» (в RESEARCH Pattern 5 готовая таблица: `thigh`, `calf`, `hip_offset`, `knee_direction`, `hip_x/y`, размеры корпуса, массы, пределы суставов, четыре длины тяги колена, `body_com_x`, ось hip и схема ноги для стоп-условия D-22). Схемы: `measure_leg_side.svg`, `measure_leg_rear.svg`, `measure_body_top.svg`, `measure_linkage.svg` (все есть в `docs/img/`). Правила: размеры между центрами осей, тяга при серве на центральном импульсе `pca9685_probe pulse <канал> 1370`; группы «Датчики» и «Восприятие» владельцу не заполнять. Язык русский. Чекпоинты владельца (`checkpoint:human-action`): мультиметр (6.0 В на V+, общая земля, VCC PCA9685 = 3.3 В, конденсатор на V+), маркировка шунта (R100 = 0.1 Ом, R010 = 0.01 Ом), транспортир для µs → радианы, `selftest` и `run`.

## Shared Patterns

### Единый стиль C++ (все новые `.hpp`/`.cpp`/тесты)
**Источник:** `ros2_ws/src/dog_control/include/dog_control/gait.hpp`, `dog_hardware/include/dog_hardware/servo_bus.hpp`.
**Применять к:** всем файлам `dog_control` и `dog_bench`.
- 2 пробела, `#pragma once`, скобки классов/функций/namespace на своей строке, управляющие на той же: `if (x) {`, односторонние тела компактно: `if (h2 < 0) {return;}`, `double phase() const {return phase_;}`.
- `const Vec3 & foot` (пробелы вокруг `&`), `char ** argv`; члены `snake_case_`; константы `kCamelCase`; `enum class` значения `UPPER_SNAKE`; единицы в имени или в конце строки `// [rad/s]`.
- Конструктор: список инициализации с ведущим двоеточием на следующей строке (`locomotion.cpp:51`).
- Закрывающие `}  // namespace dog_control` и `}  // namespace`.
- `-Wall -Wextra -Wpedantic` чисто на Jazzy (GCC 13.3) и Lyrical (GCC 15.2): предупреждения чинить в коде (`static_cast<int>(...)`, убрать неиспользуемые параметры).

### Ядро без ROS + тонкий узел
**Источник:** `dog_control/CMakeLists.txt` 18-38 (`dog_control_core` + `locomotion_node`), `dog_hardware/CMakeLists.txt` 93-113.
**Применять к:** `servo_limits.*` (в ядро), правки `locomotion_node.cpp` (только проводка, параметры, таймеры), `dog_bench_core` (+ `servo_speed_test` как тонкий `main`).
```cmake
add_library(dog_control_core ...)
target_include_directories(dog_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dog_control_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
```

### Ошибки: значение вместо броска, fail-fast на старте
**Источник:** `servo_driver.hpp` 91-92 (`std::string validate() const`), `servo_driver_node.cpp` 298-311 (`main` с `try/catch`, `RCLCPP_FATAL`, код 1), `onParams` (`SetParametersResult` с `reason`).
**Применять к:** `minimalPeriod` (возвращает `0.0`), `locomotion_node` (`throw std::runtime_error(...)` при старте, колбэк параметров без бросков), `dog_bench` (любая I2C-ошибка → отпускание и код ≠ 0).

### Логи и watchdog
**Источник:** CLAUDE.md «Logging»; `locomotion_node.cpp` 130-140 (`RCLCPP_WARN(get_logger(), "cmd_vel timeout (%.2fs) - stopping", ...)`, `command '%s' rejected (mode %s)`).
**Применять к:** новым сообщениям узла (смена периода, отказ смены в режиме WALK). Логировать переходы и отклонённые команды, не каждый тик; в контуре управления `RCLCPP_WARN_THROTTLE`. Каждый новый вход безопасности обязан иметь watchdog (в `dog_bench`: таймаут тика 50 мс, `--max-seconds`).

### Python: стиль и структура
**Источник:** `dog_description/dog_description/urdf.py`, `tools/autocal/robotdog_autocal/servo_model.py`, `ros2_ws/src/dog_gazebo/dog_gazebo/terrain_sweep.py`.
**Применять к:** `servo_profile.py`, `acceptance*.py`, `tools/servo_speed/*`.
- 4 пробела, одинарные кавычки, f-строки, строки до ~120, docstring с тройными `"""` на модуле и публичных классах/функциях (первая строка — сводка), `# ---------- name` разделители секций, `# noqa: E731` для лямбд в переменной, `# noqa: E402` для поздних импортов.
- Приватные хелперы с `_`; константы `UPPER_SNAKE`; `@dataclass` для значений; `__init__.py` пакетов пустой, импортировать модули явно (`from dog_gazebo.terrain_sweep import ...`).
- Комментарии на английском; `docs/` и интерфейс `robot_setup` на русском.
- Совместимость с Python 3.12 и 3.14: без нового синтаксиса.

### CLI-проверки: PASS/FAIL, JSON, код выхода
**Источник:** `walk_check.py` 157-159, 189-243, 274; `terrain_sweep.py` 95-98.
**Применять к:** `acceptance.py`, `walk_check.py` (правки), `tools/servo_speed/analyze.py`.
```python
    def check(self, name, ok, detail, **values):
        self.results.append((name, ok, detail, values))
        print('%-6s %-12s %s' % ('PASS' if ok else 'FAIL', name, detail), flush=True)
...
    sys.exit(code)
```
`print(..., flush=True)`, `--out`/`--trace` JSON, `sys.exit(0 if ok else 1)`; CI полагается на код выхода. Время в манёврах — по меткам симуляции (`sim_time()`), не по стенным часам.

### YAML: регекс-дружелюбный формат
**Источник:** `ros2_ws/src/dog_bringup/config/robot.yaml` 27-34, 220-235; `tools/robot_setup/robot_setup.py` `set_nested` 194-216.
**Применять к:** всем правкам `robot.yaml`/`servos.yaml`: `key: value  # comment`, один ключ в строке, единицы `# [unit]`, объяснение неочевидного значения над ключом, источник числа в комментарии, корневой ключ `/**:` + `ros__parameters`. Значения по умолчанию в коде == YAML; `robot.yaml` — единый источник правды для `dog_control`, `dog_description.urdf.load_config`, `dog_perception.core.robot_config`.

### Тесты: детерминизм, без сна и стенных часов
**Источник:** `dog_control/test/test_gait.cpp` (`kDt = 0.02`), `dog_hardware/test/test_power.cpp` (время — `i * 0.01`), `dog_description/test/test_urdf.py` (`default_rng(0)`).
**Применять к:** всем новым тестам. Launch-тесты берут свой `ROS_DOMAIN_ID` (занято 41-43; CI-симуляции 60-77; приёмка 80-89); в этой фазе новых launch-тестов нет. Тест `dog_gazebo` в `colcon test` не попадает (`--packages-skip dog_gazebo`).

### Запрет параллельных симуляций, только CI
**Источник:** `docs/TERRAIN.md`, CONTEXT D-24.
**Применять к:** всем планам, затрагивающим `walk_check`/`acceptance`: локально допустимы g++/cmake/gtest для ROS-free ядер и pytest для Python/tools; Gazebo, `terrain_sweep`, Docker-образы simulation и полная приёмка запускаются только `gh workflow run ci.yml --ref gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii`. Не печатать URL `origin` (токен).

## No Analog Found

| File | Role | Data Flow | Reason |
|------|------|-----------|--------|
| `ros2_ws/src/dog_bench/src/session.cpp` (обработчик аварийных сигналов: один async-signal-safe `write()` + `_exit(3)`, `sigaction`) | service | event-driven | В репозитории нет обработчиков сигналов: ни `sigaction`, ни `signal()` в C++ (Python `terrain_sweep.run_level` использует `os.killpg` — другой уровень). Брать схему из RESEARCH Pattern 4 «Предохранители»; RAII-часть имеет аналог (`relax_on_exit_`). |
| `ros2_ws/src/dog_bench/src/ramp.cpp` / цикл опроса 1 кГц с метками `CLOCK_MONOTONIC` | utility | streaming (опрос по таймеру) | Нет C++-кода с жёстким циклом опроса вне rclcpp-таймеров (`create_wall_timer` в узлах). Использовать `std::chrono::steady_clock`/`clock_gettime(CLOCK_MONOTONIC)` по RESEARCH; ближайший стиль — `steadySeconds()` в `servo_driver_node.cpp` 64-68. |

## Metadata

**Analog search scope:** `ros2_ws/src/dog_control`, `dog_hardware`, `dog_description`, `dog_gazebo`, `dog_web`, `dog_bringup`; `tools/robot_setup`, `tools/autocal`; `.github/workflows/ci.yml`; `docs/`.
**Files scanned:** ~45 (прочитаны целиком или по нужным диапазонам; `servo_driver_node.cpp`, `servo_bus.cpp`, `power_sensor.*`, `pca9685_probe.cpp`, `walk_check.py`, `terrain_sweep.py`, `urdf.py`, `sim.launch.py`, `joint_command_bridge.py`, `ci.yml`, `robot.yaml`, `servos.yaml`, `robot_setup.py`, тесты).
**Tracked-source gate:** все указанные аналоги возвращают непустой `git ls-files`; зеркал и установочных копий в PATTERNS.md нет.
**Pattern extraction date:** 2026-09-30
