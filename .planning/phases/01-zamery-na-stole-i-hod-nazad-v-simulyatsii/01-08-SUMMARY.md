---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 08
subsystem: control
tags: [gait-period, servo-limits, auto-period, parameters, gtest, cpp17]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 02
    provides: "frozen key names gait.auto_period/gait.min_period and servo.{max_speed,margin,knee_ratio} (D-12, D-13, D-20)"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 04
    provides: "servo_limits core: ServoSpeedModel, minimalPeriod, kMaxAutoPeriod, kSpeedTolerance and the pinned period table"
provides:
  - "LocomotionController: the auto period at construction (D-12) and reconfigureGait() for live changes in PASSIVE, STAND, LYING; 0.0 -> std::runtime_error, a refusal keeps the gait"
  - "locomotion_node: gait.auto_period/gait.min_period/servo.* parameters, atomic set-parameters callback with reasons, RCLCPP_FATAL + exit code 1 when no period fits"
  - "six new Locomotion gtests (26 -> 29); pinnedParams() in both dog_control test files keeps the algorithm tests independent of the 01-15 defaults sync (OI-1)"
affects: [01-15 (defaults sync), 01-16/01-17 (first robot runs), phase 01 verification]

# Actuals (#2632) - pairs with the plan's estimate
actuals:
  tokens: 7168
  tasks: 4
  commits: 3
plan_head_before: d6af2a724760e9e144b36d26e9892e8a85f9b3d9

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "A live parameter change is applied atomically as one reconfigureGait() call; the mode gate is checked before any computation, so a refused change costs nothing"
    - "The core turns 'no period fits' into std::runtime_error; the node turns it into RCLCPP_FATAL + exit 1 at start and successful=false + reason in the callback"

key-files:
  created: []
  modified:
    - ros2_ws/src/dog_control/include/dog_control/locomotion.hpp
    - ros2_ws/src/dog_control/src/locomotion.cpp
    - ros2_ws/src/dog_control/src/locomotion_node.cpp
    - ros2_ws/src/dog_control/test/test_locomotion.cpp
    - ros2_ws/src/dog_control/test/test_servo_limits.cpp

key-decisions:
  - "The period is computed at start and on live changes, but only in PASSIVE, STAND, LYING; WALK and transitions answer successful=false with 'period change rejected in mode <mode>' (D-12, T-01-08-01)"
  - "reconfigureGait keeps the per-leg step heights, so a guard-raised swing is not reset by a rebuild; standing joints are unchanged to 1e-9"
  - "No period up to kMaxAutoPeriod (1.5 s) is a startup refusal (FATAL, exit 1) and a refused live change; the controller never walks on an overload (T-01-08-02)"
  - "pinnedParams() (v1 values, auto_period=false, min_period 0.55, servo {6.0, 0.8, 1.0}) backs the six new tests; the old JointSpeedsFitTheServos stays the shipped-defaults gate, byte-identical to main (OI-1)"

patterns-established:
  - "asNumber/declareNumber copied from servo_driver_node.cpp so the new numeric keys tolerate int and double from YAML or `ros2 param set` (dog_hardware untouched, PR-03)"

requirements-completed: [CAL-17, GAIT-06]

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "The controller with auto_period builds the trot with the computed period (6.0 rad/s, margin 0.8 -> 0.6000 s) and walks it (phase gain dt/0.6); a manual gait.period stays an explicit override"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_locomotion.cpp#Locomotion.AutoPeriodAtStart"
        status: pass
    human_judgment: false
  - id: D2
    description: "The gates: every row of the pinned period table (3.5..7.0 rad/s) holds the five extreme commands' peak at or below margin*max_speed + 1e-9 with 0 unreachable; the knee_ratio 1.388 row grows the period past 0.55 and fits 6.4; no-fit and degenerate inputs throw std::runtime_error"
    requirement: "GAIT-06"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_locomotion.cpp#Locomotion.JointSpeedsFitTheServosAuto"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_locomotion.cpp#Locomotion.NoFitThrows"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_servo_limits.cpp#ServoLimits (9 tests, pinnedParams() extended by exactly the three lines)"
        status: pass
    human_judgment: false
  - id: D3
    description: "reconfigureGait accepts a live period change in PASSIVE, STAND, LYING (keeps the joints and the guard's swing heights; the phase then grows with the new period) and refuses in WALK and every transition without side effects"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_locomotion.cpp#Locomotion.ReconfigureAcceptedWhenStanding"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_locomotion.cpp#Locomotion.ReconfigureRejectedWhenWalking"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_locomotion.cpp#Locomotion.ReconfigureNoFitKeepsTheGait"
        status: pass
    human_judgment: false
  - id: D4
    description: "locomotion_node wiring compiles and the whole package passes colcon test on Jazzy and Lyrical in CI (205 tests, 0 errors, 0 failures, 0 skipped; zero warning lines from dog_control/)"
    verification:
      - kind: integration
        ref: "CI run 37354974119, jobs 'build + test (jazzy)' and 'build + test (lyrical)', headSha 6e9105f"
        status: pass
    human_judgment: false
  - id: D5
    description: "The callback's live behaviour on the robot (ros2 param set acceptance/rejection, minimalPeriod time on the Banana Pi) and the correspondence of the assumed number to the real MG996R"
    verification: []
    human_judgment: true
    rationale: "Plan backstops: rclcpp is not available locally (D-24), so the wiring is confirmed by the CI build + code reading; the Pi timing and the live param set are checked at the first robot run (01-16/01-17), the servo number by the bench measurement (01-14)"

# Metrics
duration: 34min
completed: 2026-10-05
status: complete
---

# Phase 1 Plan 08: Auto gait period and live reconfiguration Summary

**The gait period is computed from the assumed servo speed (0.6000 s at 6.0 rad/s on the pinned v1 set), set at construction and on live parameter changes only in PASSIVE/STAND/LYING, never shorter than min_period; the peak stays under margin*max_speed on every pinned table row (D-11, D-12, D-13, D-20), and a period that does not fit up to 1.5 s refuses to start (FATAL, exit 1).**

## Performance

- **Duration:** 34 min
- **Started:** 2026-10-05T18:08:28Z
- **Completed:** 2026-10-05T18:42:41Z
- **Tasks:** 4
- **Files modified:** 5

## Accomplishments
- `LocomotionParams` carries `auto_period`/`min_period`/`servo`; the constructor computes the effective period (`minimalPeriod` up to `kMaxAutoPeriod`) and builds the trot with it; a manual `gait.period` stays an explicit override.
- Six new gtests on the pinned v1 reference set: `AutoPeriodAtStart` (0.6000 s, phase gain dt/0.6), `JointSpeedsFitTheServosAuto` (the whole period table + the knee_ratio 1.388 row), `NoFitThrows`, and the three `reconfigureGait` tests; the old `JointSpeedsFitTheServos` remains byte-identical to main (the shipped-defaults gate).
- `reconfigureGait` rebuilds the trot only in PASSIVE/STAND/LYING, keeps the guard's per-leg step heights, and refuses without side effects in WALK and transitions; the phase advance with the new period is asserted.
- `locomotion_node`: `gait.auto_period`/`gait.min_period`/`servo.*` declared (int/double tolerant), the callback applies the merged set atomically with reasons ("period change rejected in mode walk", "no gait period up to 1.5 s fits ..."), `main` catches the controller exception as FATAL + exit 1.
- CI on the phase branch: all 8 jobs of run 37354974119 success on Jazzy and Lyrical; `colcon test-result` 205 tests, 0 errors, 0 failures, 0 skipped; zero warning lines from `dog_control/`.

## Task Commits

Each task was committed atomically:

1. **Task 1 (tracer): auto period in LocomotionController and the servo gates (D-11, D-12, D-13, D-20)** - `2db475e` (feat)
2. **Task 2: reconfigureGait for live period changes in PASSIVE, STAND, LYING (D-12)** - `83645a8` (feat)
3. **Task 3: node wiring - gait.auto_period, servo.* parameters and the set-parameters callback (D-12, D-13, D-20)** - `6e9105f` (feat)

**Plan metadata:** `docs(01-08): complete auto gait period plan` (this commit: SUMMARY + STATE + ROADMAP + WINDOWS)

_Note: task 4 (CI on Jazzy and Lyrical) produced no file changes - it pushed the phase branch and dispatched ci.yml (RUN_ID 37354974119); no fix commits were needed._

## Files Created/Modified
- `ros2_ws/src/dog_control/include/dog_control/locomotion.hpp` - `auto_period`/`min_period`/`servo` fields; `gaitPeriod()`; `gaitReconfigurable()`; `reconfigureGait()`; the period note in the header
- `ros2_ws/src/dog_control/src/locomotion.cpp` - `effectivePeriod`/`gaitParamsFor` (throws on no fit); the constructor uses the computed period; `reconfigureGait` (mode gate first, step heights preserved)
- `ros2_ws/src/dog_control/src/locomotion_node.cpp` - `gait.auto_period`/`gait.min_period`/`servo.*` parameters, `asNumber`/`declareNumber`, the `onParams` callback, the period log, `main` with FATAL + exit 1
- `ros2_ws/src/dog_control/test/test_locomotion.cpp` - `pinnedParams()`, `kExtremeCommands`, `measureServoSpeeds`, `phaseAdvance`; six new tests
- `ros2_ws/src/dog_control/test/test_servo_limits.cpp` - `pinnedParams()` gains exactly the three lines `auto_period`/`min_period`/`servo` (nothing else changed)

## Decisions Made
- Live changes are one atomic `reconfigureGait()` call after the mode gate; the node answers `successful=false` with a reason instead of throwing (T-01-08-03).
- The rebuilt trot keeps the per-leg step heights so a guard-raised swing is not reset; standing joints are unchanged to 1e-9.
- `pinnedParams()` carries the three new fields so the 01-15 defaults sync cannot move the algorithm tests; the old gate test is untouched by design (OI-1).

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - verification command] Task 3 verify #2's CMake/package.xml clause compared to main**
- **Found during:** Task 3 (node wiring)
- **Issue:** the clause `git diff --quiet main -- .../CMakeLists.txt .../package.xml` cannot pass on the phase branch: dependency plan 01-04 registered `servo_limits` in CMakeLists.txt (commit 68885c2), so the file legitimately differs from main before task 3 started. The clause's intent (its fails_when: "CMakeLists.txt or package.xml edited") is "this plan must not edit them".
- **Fix:** ran the equivalent check against the plan base d6af2a7 (`git diff --quiet d6af2a7 -- CMakeLists.txt package.xml` -> exit 0, both files untouched by plan 01-08); every other clause of verify #2 passes unchanged.
- **Files modified:** none (verification only)
- **Verification:** `git diff --stat main -- CMakeLists.txt` = 3 insertions/2 deletions from 68885c2 only; vs d6af2a7 empty; dog_hardware, CMakeLists.txt and package.xml untouched by this plan.
- **Committed in:** n/a (documented here; WINDOWS.md entry 2 records it, marked fixed)

**2. [Rule 3 - blocking] gh panicked in background shells because of an injected placeholder GITHUB_TOKEN**
- **Found during:** Task 4 (CI dispatch)
- **Issue:** background shells carry a 13-character placeholder `GITHUB_TOKEN`; gh 2.67.0 prefers it over its own config and panics (nil dereference in `FindWorkflow`) when resolving `--workflow ci.yml`, so `ci_dispatch.sh` could not list runs.
- **Fix:** ran the prescribed dispatch command with `env -u GITHUB_TOKEN` in front (script and arguments unchanged); gh then uses its own authenticated config (the credential-helper setup of plan 01-01).
- **Files modified:** none (environment only)
- **Verification:** `env -u GITHUB_TOKEN gh run list --workflow ci.yml ...` -> rc=0 in a background shell; the dispatch then completed with exit 0.
- **Committed in:** n/a

---

**Total deviations:** 2 handled (1 verification-base correction, 1 environment workaround). No source or test behaviour was changed by either; every other automated check and acceptance criterion of the plan passes as written.
**Impact on plan:** none on the deliverable.

## Issues Encountered
- The first `ReconfigureRejectedWhenWalking` run asserted 0.6000 s for the accepted "same call" at servo {5.0, 0.8, 1.0}; the computed period is 0.7200. Fixed in the test before the task commit (caught by the task's own verification loop).
- `ci_dispatch.sh` prints "incomplete status ... retrying" every 30 s while the run is in progress (gh returns an empty `conclusion` string, which shifts the tab-read); benign - the script still waits for the run and reported exit 0 with both awaited jobs success. `ci_dispatch.sh` is plan 01-01's file and was not modified.

## User Setup Required
None - no external service configuration required.

## Next Phase Readiness
- Plans 01-09..01-17 remain; 01-15 will sync the shipped defaults (robot.yaml auto_period flip) without moving the pinned test numbers.
- CAL-17 and GAIT-06 stay open in REQUIREMENTS.md: both are declared by sibling plans without summaries (`requirements.ready-ids`: 0/2 ready), so the shared-ID gate defers completion as designed.
- Backstops carried by the plan (not resolvable here): the Pi timing of `minimalPeriod` in the callback and the live param-set behaviour (first robot run, 01-16/01-17); the servo number correspondence (bench measurement, 01-14).

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-05*

## Self-Check: PASSED

All five modified files and all three task commits (2db475e, 83645a8, 6e9105f) exist; the plan's verifications (locomotion 29 tests, servo_limits 9 tests, six binary suites green under gtest -Werror, CI run 37354974119 build + test success on Jazzy and Lyrical) were run and shown PASS.
