---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 05
subsystem: bench-tooling
tags: [ina219, i2c, servo-speed, selftest, gtest, cpp17, cli]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 01
    provides: "tools/local_gtest/run.sh (colcon-free g++ -Werror gtest runner, dog_bench is one of the two allowed packages); phase branch with upstream"
provides:
  - "ros2_ws/src/dog_bench: ament_cmake package with no ROS dependency; targets dog_bench_core (export_dog_bench) and servo_speed_test"
  - "dog_bench::I2cBus / LinuxI2cBus (one ioctl(I2C_RDWR) per transaction, the slave address in every message) / FakeI2cBus (pointer model, counters, dropWrites, accumulating failRange, transaction hook)"
  - "dog_bench::ina: kIna219Fast320mv = 0x199F, kIna219Fast80mv = 0x099F, isSafeAddress 0x41..0x4f, shuntVolts/busVolts/fullScaleShuntVolts/configForShunt"
  - "Ina219Fast: configure reads before it writes and checks the read-back; readShunt reuses the register pointer; readBus; currentA on the host"
  - "runSelftest + SelftestResult: 1 ms polling, median/p99/max, 1.5 ms and 5 ms cutoffs, 3 consecutive errors abort; monotonicSeconds/sleepSeconds"
  - "servo_speed_test CLI: selftest | dry-run | run; strict arguments; exit codes 0/1/2 (3 reserved for plan 01-12)"
affects: [01-09 (fail-safes and PWM grow on I2cBus/FakeI2cBus/read8/writeBytes and CMakeLists), 01-12 (measurement session, dry-run/run, exit code 3), 01-14 (owner runs selftest on the robot), phase 01 verification]

# Tech tracking
tech-stack:
  added: [gtest (local, through tools/local_gtest/run.sh), ament_cmake + ament_cmake_gtest]
  patterns:
    - "Core + thin entry split: dog_bench_core with no ROS, the CLI in servo_speed_test_main.cpp (excluded from local gtest builds by the _main.cpp rule)"
    - "Scale formulas copied into dog_bench::ina; the driver package headers are never included (D-09)"
    - "Address safety before any bus access: an unsafe address returns false with zero transactions"
    - "Deterministic tests: fake clock injected through the bus transaction hook, FakeI2cBus counters, no sleeps and no wall time"

key-files:
  created:
    - ros2_ws/src/dog_bench/package.xml
    - ros2_ws/src/dog_bench/CMakeLists.txt
    - ros2_ws/src/dog_bench/include/dog_bench/i2c_bus.hpp
    - ros2_ws/src/dog_bench/src/i2c_bus.cpp
    - ros2_ws/src/dog_bench/include/dog_bench/ina219_fast.hpp
    - ros2_ws/src/dog_bench/src/ina219_fast.cpp
    - ros2_ws/src/dog_bench/include/dog_bench/selftest.hpp
    - ros2_ws/src/dog_bench/src/selftest.cpp
    - ros2_ws/src/dog_bench/src/servo_speed_test_main.cpp
    - ros2_ws/src/dog_bench/test/test_ina219_fast.cpp
    - ros2_ws/src/dog_bench/test/test_selftest.cpp
  modified: []

key-decisions:
  - "FakeI2cBus::failRange accumulates ranges (fix 7a457e2): the planned Selftest tests register two disjoint single failures with two calls, so replacing a previous range would silently drop the first"
  - "runSelftest measures intervals between tick starts and judges median <= 1.5 ms and p99 (rank ceil(0.99 n) - 1) <= 5 ms; three consecutive failed transactions abort and the statistics still cover the intervals collected so far"
  - "The CLI validates every argument before opening the bus, and --help wins over all other checks; dry-run and run pass validation and then refuse with exit 2 (no PWM code is linked, the servos stay off)"
  - "0x40 (PCA9685) and any address outside 0x41..0x4f never touch the bus: isSafeAddress guards configure/readShunt/readBus and the CLI message names the PCA9685"

patterns-established:
  - "Register pointer reuse: readShunt is a bare read while the pointer sits on the shunt register; after readBus or a failure the pointer is set again"
  - "Strict CLI number parsing: whole-string strtol/strtod, finite doubles, /dev/i2c-<digits> whitelist, all before any device access"

requirements-completed: [CAL-17]

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "Fast INA219 layer and fake bus: 6 Ina219Fast gtests (register fields, scaling, write+read-back, 0x40 refusal without transactions, bad device refusal, register pointer reuse) pass under g++ -Werror"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_bench/test/test_ina219_fast.cpp#Ina219Fast.* (6 tests); tools/local_gtest/run.sh dog_bench ina219_fast"
        status: pass
    human_judgment: false
  - id: D2
    description: "runSelftest polling-rate check: 5 Selftest gtests on fake clocks (fast bus accepted, slow median, p99 stalls, consecutive errors, too few samples)"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_bench/test/test_selftest.cpp#Selftest.* (5 tests); tools/local_gtest/run.sh dog_bench selftest"
        status: pass
    human_judgment: false
  - id: D3
    description: "CLI servo_speed_test: builds from all sources with -Werror, 20 invalid argument sets exit 2, --help exits 0 and mentions selftest, 0x40 names the PCA9685, a missing bus (/dev/i2c-99) exits 1, no PWM code in the package"
    requirement: "CAL-17"
    verification:
      - kind: other
        ref: "g++ -std=c++17 -O1 -Wall -Wextra -Wpedantic -Werror src/*.cpp; 20 bad sets -> exit 2; --help | grep selftest; 0x40 stderr has PCA9685; /dev/i2c-99 -> exit 1; grep -rniE 'setPulse|PRE_SCALE|ALL_LED|LED0_ON|0xFA' -> empty"
        status: pass
    human_judgment: false
  - id: D4
    description: "CMake registration of the new sources and targets under colcon (GCC 13.3 Jazzy / 15.2 Lyrical)"
    verification: []
    human_judgment: true
    rationale: "No colcon/ros2 locally (D-24): only the textual CMake registration and the g++ -Werror build of the same sources are proven here; the colcon build of this package is checked in the plan 01-12 wave (and the GCC 15 side by the phase-branch CI)"
  - id: D5
    description: "Backstop (plan must_haves, verification: backstop): the real polling rate on the H618 at the 100 kHz bus is not observable locally; the bus speed is configured nowhere"
    verification: []
    human_judgment: true
    rationale: "Depends on the robot hardware and its I2C_TIMEOUT behaviour (STATE.md); the owner runs servo_speed_test selftest on the Banana Pi (plan 01-14), which is exactly what the selftest exists for"

# Actuals (#2632)
actuals:
  tokens: 12383
  tasks: 3
  commits: 4
plan_head_before: 4236ba0dd59f70fadfdc1e19818e128e533d5779

# Metrics
duration: 10 min
completed: 2026-10-05
status: complete
---

# Phase 1 Plan 05: Dog bench — fast INA219 and the servo_speed_test selftest path Summary

**New ROS-free `dog_bench` package: `ioctl(I2C_RDWR)` bus with a fake twin, fast INA219 (0x199F / 0x099F, pointer reuse, read-back configure), a 1 kHz selftest with 1.5 ms / 5 ms cutoffs and a CLI whose 20 wrong argument sets all exit 2 — 11 gtests, all green under g++ -Werror.**

## Performance

- **Duration:** 10 min
- **Started:** 2026-10-05T09:23:02Z (first recorded clock reading)
- **Completed:** 2026-10-05T09:33:08Z
- **Tasks:** 3
- **Files modified:** 11 created (plus the four .planning close-out files)

## Accomplishments

- `dog_bench` package (ament_cmake only, no `rclcpp`, no `dog_hardware`): builds locally with `-Wall -Wextra -Wpedantic -Werror`, sources listed one per line so 01-09/01-12 can add rows.
- `i2c_bus`: `LinuxI2cBus` does exactly one `ioctl(I2C_RDWR)` per transaction with the slave address inside every `i2c_msg` (no separate address ioctl); `readBare16` reads without moving the pointer; `writeBytes` caps at 32 bytes; open failures leave `ok() == false` with `strerror` text. `FakeI2cBus` models the INA219 register pointer, counts transactions / writes / pointer reads / bare reads, and injects dropped writes and failing transaction ranges (used by the deterministic Selftest tests).
- `ina219_fast`: `0x199F` (PG /8, ±320 mV, 12-bit, continuous shunt+bus, 1.064 ms per result) and `0x099F` (PG /2, ±8 A on 10 mΩ) decoded field by field in a gtest; `configForShunt` accepts only 0.1 Ω and 0.01 Ω (±5 %) and never guesses; `Ina219Fast::configure` reads before it writes, refuses the PCA9685 address and reset-bit configs without a single transaction, and accepts only an exact read-back; `readShunt` reuses the register pointer, any failure drops it.
- `runSelftest`: 1 ms tick, shunt every tick, bus every 8th; median / p99 (rank `ceil(0.99 n) − 1`) / max over the intervals between tick starts; verdict FAIL on median > 1.5 ms or p99 > 5 ms with the 400 kHz advice (docs/REVIEW.md item 12) in the reason; 3 consecutive failed transactions abort and still report the partial statistics; fewer than 100 samples is refused before any bus access.
- `servo_speed_test` CLI: one closed path `LinuxI2cBus -> Ina219Fast -> runSelftest -> verdict -> exit code`; `--ina-address` and `--shunt-ohm` mandatory with no defaults, `/dev/i2c-<digits>` only, strict whole-string number parsing (nan/inf rejected); 0x40 answers "is the PCA9685 and is never probed as an INA"; `dry-run`/`run` validate everything and then refuse with exit 2 — the package contains no PWM code, so a servo cannot be driven by this build.
- TDD red steps behaved as designed: task 1 red = runner refused (no sources yet), task 2 red = missing-header compile error; both turned green with the implementations.

## Task Commits

Each task was committed atomically:

1. **Task 1: dog_bench skeleton, fake I2C bus and fast INA219** - `14d337c` (feat)
2. *(deviation fix, found while writing task 2 tests)* **FakeI2cBus::failRange accumulates ranges** - `7a457e2` (fix)
3. **Task 2: runSelftest polling-rate check on fake clocks** - `6cdff07` (feat)
4. **Task 3 (tracer): servo_speed_test selftest CLI** - `4182c3a` (feat)

**Plan metadata:** `docs(01-05): complete dog bench servo speed selftest plan` (this commit: SUMMARY + STATE + ROADMAP + REQUIREMENTS)

## Files Created/Modified

- `ros2_ws/src/dog_bench/package.xml` - format 3, ament_cmake + ament_cmake_gtest only, no `depend` elements
- `ros2_ws/src/dog_bench/CMakeLists.txt` - C++17, dog_bench_core (`i2c_bus.cpp`, `ina219_fast.cpp`, `selftest.cpp`), `servo_speed_test` executable installed to `lib/dog_bench`, `foreach(t ina219_fast selftest)` gtests, `export_dog_bench`
- `ros2_ws/src/dog_bench/include/dog_bench/i2c_bus.hpp` - `I2cBus`, `LinuxI2cBus`, `FakeI2cBus`
- `ros2_ws/src/dog_bench/src/i2c_bus.cpp` - `I2C_RDWR` transactions, big-endian reads, fake bus implementation
- `ros2_ws/src/dog_bench/include/dog_bench/ina219_fast.hpp` - `dog_bench::ina` constants and scale helpers; `Ina219Fast`
- `ros2_ws/src/dog_bench/src/ina219_fast.cpp` - field decode, `configForShunt`, guarded configure/readShunt/readBus
- `ros2_ws/src/dog_bench/include/dog_bench/selftest.hpp` - thresholds, `SelftestResult`, `ClockFn`/`SleepFn`, `runSelftest`, `monotonicSeconds`/`sleepSeconds`
- `ros2_ws/src/dog_bench/src/selftest.cpp` - the polling loop, statistics, reason texts
- `ros2_ws/src/dog_bench/src/servo_speed_test_main.cpp` - CLI parsing, validation, selftest mode, exit codes
- `ros2_ws/src/dog_bench/test/test_ina219_fast.cpp` - six Ina219Fast tests
- `ros2_ws/src/dog_bench/test/test_selftest.cpp` - five Selftest tests on the fake clock

## Decisions Made

- The `failRange` contract is "accumulate": a call adds a failing range, ranges expire by transaction number (drives the two disjoint single failures of the planned Selftest test and keeps the pointer test unchanged).
- `p99` uses the exact planned rank `ceil(0.99 n) − 1`; even medians average the two middle intervals; the abort path still computes and reports statistics over the intervals collected so far.
- `dry-run` and `run` are deliberately unreachable in this build (exit 2) and the CLI says so; exit code 3 is only named in the help text, reserved for plan 01-12.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] `FakeI2cBus::failRange` replaced the previous range instead of accumulating it**
- **Found during:** Task 2 (writing `test_selftest.cpp` — the planned `CountsConsecutiveErrors` case registers `failRange(first + 10, 1)` and `failRange(first + 40, 1)`, two disjoint single failures)
- **Issue:** the task 1 implementation stored one range; the second call would silently drop the first, so only one of the two planned single failures could ever fire and the test would count 1 error instead of 2.
- **Fix:** ranges are kept in a `std::vector<std::pair<int, int>>`; `failRange` appends (zero-count is a no-op) and `fails()` scans all ranges.
- **Files modified:** `ros2_ws/src/dog_bench/include/dog_bench/i2c_bus.hpp`, `ros2_ws/src/dog_bench/src/i2c_bus.cpp`
- **Verification:** `tools/local_gtest/run.sh dog_bench ina219_fast` still 6/6; the Selftest suite (which needs two disjoint failures) passes 5/5.
- **Committed in:** 7a457e2

**2. [Rule 1 - Bug] `--help` exited 2 when no mode argument was present**
- **Found during:** Task 3, first run of the task's verify command (`$B --help` returned rc 2, `grep -q selftest` saw nothing)
- **Issue:** `parseArguments` validated the positional mode before `run()` could act on the `--help` short-circuit, so the help request died as "missing mode".
- **Fix:** `parseArguments` returns right after its loop when `--help` is set (`--help wins over everything else`); `run()` then prints the usage to stdout and returns 0.
- **Files modified:** `ros2_ws/src/dog_bench/src/servo_speed_test_main.cpp`
- **Verification:** `$B --help` exits 0 and prints the usage containing `selftest`; the full 20-set verify command passes end to end.
- **Committed in:** 4182c3a

---

**Total deviations:** 2 auto-fixed (2 bugs).
**Impact on plan:** both were required for the plan's own tests/verify to pass; no scope creep — only the plan's own files were touched, and no behavior outside the plan's acceptance criteria changed.

## Issues Encountered

- The TDD red steps behaved exactly as designed (task 1: the runner refused with "no core sources"; task 2: `dog_bench/selftest.hpp: No such file or directory`), then green after implementation — no re-planning needed.
- No `colcon`/`ros2` locally (D-24): the local compiler is GCC 14.2 while CI uses 13.3 (Jazzy) and 15.2 (Lyrical); the colcon build of the package is checked in the plan 01-12 wave (coverage D4).
- The no-bus path was exercised through `/dev/i2c-99`; the real selftest run against the sensor is the owner's task in plan 01-14 (coverage D5).

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Plan 01-09 can build the ramp, fail-safes and PWM on `I2cBus`/`FakeI2cBus` (`read8`/`writeBytes` are already there for the PCA9685) without touching this plan's headers; plan 01-12 adds `session.cpp` and can claim exit code 3; plan 01-14 runs `servo_speed_test selftest` on the robot.
- `CAL-17` stays open in REQUIREMENTS.md: `requirements.ready-ids` reports 0/1 ready (sibling plans declaring CAL-17 have no summary yet), so the shared-ID gate correctly deferred completion.
- Backstops carried: the real 1 kHz polling on the H618 (owner's selftest, 01-14) and the colcon build on GCC 13/15 (01-12 wave / phase CI).

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-05*

## Self-Check: PASSED

- All 11 key-files.created exist on disk (checked with `[ -f ]`).
- All four commits exist: `14d337c`, `7a457e2`, `6cdff07`, `4182c3a`.
- Both gtests pass (`[  PASSED  ] 6 tests.` / `[  PASSED  ] 5 tests.`), the 20-set CLI verify exits 0, `dog_hardware` and `.github` untouched.
