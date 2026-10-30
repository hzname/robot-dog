---
last_mapped_commit: d269bd16b0d0f69f3ab623b9cecc4ba545b70517
last_mapped_at: 2026-09-29
---
# Coding Conventions

**Analysis Date:** 2026-09-29

Scope: the live code in `ros2_ws/src/` (C++17 and Python ROS 2 packages) and `tools/` (standalone Python). `legacy/` is excluded from colcon (`legacy/COLCON_IGNORE`) and is history only: do not copy its patterns. Untracked top-level files (`robot_configurator.py`, `robot_dog_ws/`, `test_servo_config_reader.sh`) follow a different, older style (bilingual docstrings, JSON config, hard-coded `/home/sg/...` paths); do not use them as a style reference.

No formatter or linter is configured or run in CI (no `.clang-format`, `.flake8`, `pyproject.toml`, `ament_lint_*` in any `CMakeLists.txt`). The conventions below are what the code actually follows; keep new code consistent with them. The only enforced quality gate is compiler warnings: every C++ package sets `-Wall -Wextra -Wpedantic` (see `ros2_ws/src/dog_control/CMakeLists.txt`).

## Naming Patterns

**Files:**

- C++ sources/headers: `snake_case.cpp` / `snake_case.hpp`, headers under `include/<package>/` (`ros2_ws/src/dog_control/include/dog_control/kinematics.hpp`). ROS node entry points end in `_node.cpp` (`locomotion_node.cpp`, `servo_driver_node.cpp`); library code has no suffix (`locomotion.cpp`, `servo_driver.cpp`).
- Python modules: `snake_case.py` (`ros2_ws/src/dog_web/dog_web/protocol.py`, `wsserver.py`). Simulation checkers end in `_check.py` (`walk_check.py`, `perception_check.py`, `localization_check.py`).
- Tests: `test_<unit>.cpp` / `test_<unit>.py`, named after the unit (`test_kinematics.cpp`, `test_protocol.py`). Launch/integration tests are named after the scenario (`test_mock_bringup.py`, `test_gamepad_fifo.py`, `test_power_monitor.py`).
- ROS packages: `dog_<domain>` (`dog_control`, `dog_hardware`, `dog_perception`, `dog_teleop`, `dog_web`, `dog_gazebo`, `dog_description`, `dog_bringup`).

**Functions:**

- C++: `camelCase` for free functions and methods (`forwardKinematics`, `inverseKinematics`, `jointToPulseUs`, `setTargets`, `legSide`). Trivial accessors are named for the value, no `get` prefix (`mode()`, `joints()`, `estopActive()`, `enabled(i)`); setters use `set` (`setVelocity`, `setEstop`).
- Python: `snake_case` (`handle_message`, `fit_joint`, `find_limit`). Module-private helpers get a leading underscore (`_num`, `_rot`, `_fk`, `_wrap_deg`).

**Variables:**

- C++ locals and struct fields: `snake_case` (`stand_height`, `cmd_timeout`). Private class data members: `snake_case_` with trailing underscore (`controller_`, `cmd_vel_active_`, `last_guard_vx_` in `ros2_ws/src/dog_control/src/locomotion_node.cpp`). Plain aggregate structs (`LocomotionParams`, `ServoCalibration`) use fields without underscore and in-class default initializers `double hip{0.055};`.
- Put units in the name or a trailing bracketed comment: `offset_deg`, `pulse_min_us`, `servo_arm_mm`, `max_joint_speed{6.0};  // [rad/s] slew limit`. Internal angles are radians; degrees appear only at config/calibration boundaries and are named `_deg`.
- Compile-time constants: `kCamelCase` (`kNumLegs`, `kEps`, `kDt`, `kFloor`, `kJointNames`). Python module constants: `UPPER_SNAKE` (`COMMANDS`, `GUARD_STATES`, `CAL_FIELDS`, `MIN_BODY_HEIGHT`).

**Types:**

- C++ classes/structs/enums: `PascalCase` (`LocomotionController`, `ServoCalibration`, `IkResult`); `enum class` values are `UPPER_SNAKE` (`Mode::STANDING_UP`, `GaitType::CRAWL`). Legacy unscoped enum for leg indices: `LF, RF, LR, RR` (`kinematics.hpp`).
- Python classes: `PascalCase`; prefer `@dataclass` for value objects (`Limits`, `Actions` in `protocol.py`; `FitResult` in `tools/autocal/robotdog_autocal/fit.py`).
- Namespaces: `dog_<domain>` matching the package (`namespace dog_control { ... }  // namespace dog_control`).

**ROS names:**

- Topics/parameters are relative and live under the `dog` namespace (`NS = 'dog'` in `ros2_ws/src/dog_bringup/launch/robot.launch.py`). Parameters are dotted `group.name` (`geometry.thigh`, `stance.stand_height`, `odom.publish`, `<joint>.offset_deg`).
- Joint names: `<leg>_<joint>_joint` with legs `lf, rf, lr, rr` and joints `hip, thigh, calf`.

## Code Style

**Formatting (C++, ament/uncrustify-like, hand-maintained):**

- 2-space indent, no tabs, `#pragma once` in every header (no include guards).
- Function, class, struct and namespace braces on their own line; control-flow braces on the same line: `if (x) {`, `} else {`.
- Single-statement bodies use compact braces on one line: `if (h2 < 0) {return;}`, `double x() const {return x_;}`.
- Reference/pointer spacing: `const Vec3 & foot`, `const std::string & cmd` (space on both sides of `&`), but `char ** argv`.
- Constructor initializer lists start with a leading colon on the next line: `: rclcpp::Node("locomotion", options)`.
- Line length: soft ~100, hard ~120 (only 8 lines exceed 120). Continuation lines indent by 2 (function args) or align under the opening arg.
- `///` for API doc comments in headers, `//` for everything else.

**Formatting (Python):**

- 4-space indent, single quotes for strings (`'stand'`), f-strings for formatting (`f'unknown message type {kind!r}'`); older `%`-formatting is acceptable in logging calls (`self.get_logger().info('web: %s' % actions.command)`).
- Lines run to ~120 characters (over 300 lines exceed that; do not add many more). Two blank lines between top-level definitions; section dividers use `# ---------- name` comments (`ros2_ws/src/dog_perception/dog_perception/core.py`, `# ---- ROS side` in `web_teleop.py`).
- Type hints are used sparingly, on public boundaries only (`def handle_message(text: str, limits: Limits) -> Actions:`); not required everywhere.
- Lambda assigned to a name is flagged with `# noqa: E731` (`robot.launch.py`); late imports after `sys.path` tweaks use `# noqa: E402` (`tools/robot_setup/test/test_robot_setup.py`). This shows flake8 rules are followed by hand.

**Linting:**

- None automated. `-Wall -Wextra -Wpedantic` must stay clean for C++. Fix warnings in code (explicit `static_cast<int>(...)`, unused parameters removed) rather than disabling them.

## Import / Include Organization

**C++ order (blank line between groups):**

1. Own header for a `.cpp` (`#include "dog_control/kinematics.hpp"`)
2. C++ standard library, alphabetical (`<algorithm>`, `<array>`, `<cmath>`)
3. Project and ROS headers in quotes, alphabetical (`"dog_control/locomotion.hpp"`, `"rclcpp/rclcpp.hpp"`, `"sensor_msgs/msg/imu.hpp"`)

Tests put `<gtest/gtest.h>` first, then standard headers, then project headers (`ros2_ws/src/dog_control/test/test_kinematics.cpp`). Prefer `using dog_control::Vec3;` declarations in tests; `using namespace dog_hardware;` also appears in tests only. Never use `using namespace` in headers.

**Python order (blank line between groups):**

1. Standard library
2. Third-party (`numpy`, `yaml`, `pytest`) and ROS (`rclpy`, `geometry_msgs`, ...)
3. First-party (`from dog_web import protocol`; relative `from .servo_model import ServoCal` inside `tools/autocal/robotdog_autocal/`)

**Path aliases:** none. `tools/` code reaches ROS packages by `sys.path.insert(0, ...)` relative to `__file__` (`tools/autocal/tests/conftest.py`, `tools/autocal/tests/test_client.py`).

## Error Handling

**C++ core libraries (ROS-independent, `*_core` libs):**

- Validation returns a value instead of throwing: `std::string validate() const` returns an empty string when OK and an error message otherwise (`ServoCalibration::validate` in `ros2_ws/src/dog_hardware/include/dog_hardware/servo_driver.hpp`); bool-returning mutators for rejected requests (`bool request(const std::string & cmd)`, `bool setCalibration(...)`).
- Out-of-range math is clamped and flagged, never NaN: `IkResult{q, reachable}` in `kinematics.hpp`; `jointToPulseUs(..., bool * clamped)`.
- Constructors of parsers throw `std::invalid_argument` with the offending spec in the message (`ros2_ws/src/dog_hardware/src/imu_sensor.cpp`, `ros2_ws/src/dog_perception/src/core.cpp`).

**C++ nodes:**

- Fail fast at startup with `throw std::runtime_error("<what> '<value>' (use a or b)")` for bad config (unknown backend, invalid calibration). `main()` wraps `rclcpp::spin` in `try/catch (const std::exception &)`, logs with `RCLCPP_FATAL`, and returns exit code 1 (`ros2_ws/src/dog_hardware/src/servo_driver_node.cpp`, `power_monitor_node.cpp`).
- Runtime parameter callbacks return `rcl_interfaces::msg::SetParametersResult` with `successful=false` and a `reason` instead of throwing.
- Bad/stale input at runtime is dropped and logged, not fatal: undersized arrays are ignored (`if (msg->data.size() < 2) {return;}`), stale command/guard/terrain inputs are cleared after a timeout (`cmd_vel_timeout`, `guard_timeout` in `locomotion_node.cpp`). Every safety input has a watchdog: new inputs must too.
- Numbers from YAML may be int or double: read parameters through a tolerant helper (`asNumber`, `declareNumber` in `servo_driver_node.cpp`).

**Python:**

- Message handlers never raise to the caller: `protocol.handle_message` catches `(ValueError, json.JSONDecodeError)` and returns `Actions(errors=[...])`; the server answers `{"type": "error", ...}` (`ros2_ws/src/dog_web/dog_web/protocol.py`, `web_teleop.py`).
- Input validation raises `ValueError` with a message naming the field: `raise ValueError(f'"{key}" must be a finite number')`. Reject `bool` where a number is expected (`isinstance(v, bool)` check in `_num`).
- Async timeouts raise `TimeoutError('no answer from the robot')` (`dog_web/calibration.py`).
- CLI checkers (`walk_check.py`, `terrain_sweep.py`, `perception_check.py`) report `PASS`/`FAIL` lines with `print(..., flush=True)` and end with `sys.exit(0 if ok else 1)`; CI relies on the exit code.

## Logging

**Framework:** ROS logging only in nodes; `print` only in CLI tools.

- C++: `RCLCPP_INFO/WARN/FATAL(get_logger(), "fmt %s", x.c_str())`; use `RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, ...)` for anything in the control loop.
- Python nodes: `self.get_logger().info/warning/error(...)`.
- Log state transitions and rejected commands (`"command '%s' rejected (mode %s)"`), not every tick. Safety events are `WARN` (`"E-STOP ENGAGED"`, `"guard: hazard ahead - forward motion stopped"`).

## Comments

**When to Comment:**

- Comments explain WHY and record measured/derived numbers, units and frames, not what the line does: `// Tuned in Gazebo with servo speed capped at 6 rad/s (see docs/SIMULATION.md)` in `ros2_ws/src/dog_bringup/config/robot.yaml`.
- Every header/module starts with a block comment describing the frame conventions, state machine or topic interface (`kinematics.hpp` joint sign conventions; `locomotion_node.cpp` subscribes/publishes list; `protocol.py` message grammar). Keep node topic lists in that header comment up to date when adding a topic.
- Reference docs by path: `(see docs/CALIBRATION.md)`, `TERRAIN.md`, `DEPLOYMENT.md`.
- Language: code comments, identifiers, log messages and docstrings are English. Long-form docs under `docs/` and the operator-facing UI text of `tools/robot_setup/robot_setup.py` (form labels, validation messages) are Russian; assert on Russian substrings in `tools/robot_setup/test/test_robot_setup.py` accordingly.
- No TODO/FIXME/HACK markers exist in `ros2_ws/` or `tools/`; open issues are tracked in `docs/REVIEW.md` instead. Follow that: write the issue there, not as a TODO.

**Docstrings:**

- Python modules and public classes/functions have triple-double-quote docstrings, first line a summary (`"""Fitting servo calibration from (pulse, measured joint angle) samples."""`). Test modules have a one-line docstring stating what is verified end to end.
- C++ `///` on public API in headers, with units and failure behaviour (`/// Validates the fields; returns an empty string when OK.`).

## Function Design

**Size:** Node classes are long but split into `loadParams()`, `tick()`, `publishX()`; pure logic lives in a ROS-independent core class or module that is unit-tested (`LocomotionController`, `ServoDriver`, `dog_web.protocol`, `dog_perception.core`).

**Parameters:** Pass structs by `const &` (`const LocomotionParams &`); config as parameter structs with defaults inline (`GaitParams`, `CrawlParams`, `DriverParams`). Time is passed in explicitly (`update(double now)`, `update(dt)`), never read from a clock inside core logic, so tests can drive it deterministically.

**Return Values:** Prefer small result structs (`IkResult`), `bool` for accepted/rejected, `std::optional` for "maybe nothing to publish" (`out.twist.has_value()` in `dog_teleop`). Python: dataclasses or plain tuples with documented order.

## Module Design

**Exports:**

- Pattern per C++ package: a `<package>_core` static/shared library (no rclcpp) + thin `*_node.cpp` executables + gtests linking the core. Add new logic to the core, keep nodes to wiring, parameters and timers (`ros2_ws/src/dog_control/CMakeLists.txt`).
- Node parameters are declared with their default taken from the params struct: `p.leg.thigh = declare_parameter("geometry.thigh", p.leg.thigh);`. Defaults in code and in `ros2_ws/src/dog_bringup/config/robot.yaml` must agree; `robot.yaml` is the single source of truth for geometry (also read by `dog_description.urdf.load_config` and `dog_perception.core.robot_config`).
- Python packages: `__init__.py` is empty; import modules explicitly (`from dog_web import protocol`).

**Barrel Files:** Not used.

**Cross-language twins:** Where logic exists twice, a test keeps them in sync and says so in the module docstring: `dog_perception/core.py` (numpy) mirrors `dog_perception/src/core.cpp`; `tools/autocal/robotdog_autocal/servo_model.py` mirrors `dog_hardware/src/servo_driver.cpp` (`tools/autocal/tests/test_servo_model.py` checks reference values); `dog_description/urdf.py` mirrors `dog_control/src/kinematics.cpp` (`test_urdf.py`). Changing a formula means changing both sides and both tests.

## Configuration Files

- ROS parameter YAML uses the wildcard node key and `ros__parameters` (`/**:` in `robot.yaml`, `/**/servo_driver:` in `servos.yaml`), inline-flow maps for per-joint rows aligned in columns, and `# [unit]` trailing comments. Explain non-obvious values in a comment above the key.
- `tools/robot_setup/robot_setup.py` rewrites values in place and keeps comments/layout: keep YAML edits regex-friendly (`key: value  # comment`, one key per line).
- Launch files: `ros2_ws/src/dog_bringup/launch/robot.launch.py` builds actions in an `OpaqueFunction` `_setup(context)`, using small local helpers `cfg(name)` / `on(name)` for launch arguments; each ROS node is given `namespace=NS`.
- `package.xml` is format 3; maintainer `hzname`, license MIT, version `2.0.0` across packages.

---

*Convention analysis: 2026-09-29*
