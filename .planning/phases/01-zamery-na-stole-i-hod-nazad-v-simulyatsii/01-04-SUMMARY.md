---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 04
subsystem: control
tags: [servo-limits, trot-gait, inverse-kinematics, gait-period, gtest, cpp17]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 01
    provides: "tools/local_gtest/run.sh (colcon-free g++ -Werror gtest runner); phase branch with upstream"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 02
    provides: "frozen Phase 1 key names: gait.auto_period/min_period, servo.{max_speed,margin,knee_ratio} (D-13, D-20)"
provides:
  - "ros2_ws/src/dog_control/include/dog_control/servo_limits.hpp + src/servo_limits.cpp: ServoSpeedModel, PeakSpeed, peakServoSpeed, minimalPeriod, kMaxAutoPeriod, kPeriodScanStep, kPeriodGuard, kSpeedTolerance; ROS-free, pure, never throwing"
  - "ros2_ws/src/dog_control/test/test_servo_limits.cpp: nine ServoLimits gtests on the pinnedParams()/model() reference set (name and values shared with plan 01-08)"
  - "CMakeLists.txt registration: src/servo_limits.cpp in dog_control_core, servo_limits in the test foreach"
affects: [01-08 (wires the functions into LocomotionController/locomotion_node), 01-15 (syncs the shipped defaults; the algorithm tests must not move), phase 01 verification, phase 2 (servo speed sweep)]

# Actuals (#2632) - pairs with the plan's estimate
actuals:
  tokens: 5430
  tasks: 3
  commits: 3
plan_head_before: aee4276818bc60cb8ec34a6c26ce4901edd455e6

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Core functions stay pure and stateless (no ROS, no allocation, no exceptions): the caller (plan 01-08) can recompute at start and on parameter change (D-12, D-13)"
    - "Algorithm tests run on pinnedParams() - the v1 robot as shipped before Phase 1 spelled out explicitly - so the shipped-defaults sync of plan 01-15 cannot redden them (OI-1)"
    - "The windowed scan over a quantised period grid replaces bisection: the peak is not monotonic in the period (20 ms tick quantises the step phase)"
    - "The duplicated neutralFeet formula is pinned by a controller tie test (PeakMatchesController) plus a deliberate red-step break"

key-files:
  created:
    - ros2_ws/src/dog_control/include/dog_control/servo_limits.hpp
    - ros2_ws/src/dog_control/src/servo_limits.cpp
    - ros2_ws/src/dog_control/test/test_servo_limits.cpp
  modified:
    - ros2_ws/src/dog_control/CMakeLists.txt
    - .planning/STATE.md
    - .planning/ROADMAP.md
    - .planning/REQUIREMENTS.md

key-decisions:
  - "The pinned v1 reference set (pinnedParams in the test, same name and values as plan 01-08) keeps the fixed numbers (5.077, the period table) independent of the code defaults that plan 01-15 syncs with the measured robot"
  - "The period table uses the guard window values: 4.0 -> 0.9050 and 6.35 -> 0.5575 (the first fits without the window are 0.8975 and 0.550 but their neighbours within 0.05 s violate the gate); exactly 0.55 s is returned from 6.4 rad/s (D-11 consequence)"
  - "minimalPeriod returns 0.0 for every degenerate input (NaN, inf, margin*max_speed <= 0, knee_ratio <= 0, min_period below the TrotGait clamp of 0.1 s, max_period < min_period) and for no-fit; the node of plan 01-08 turns 0.0 into a refusal, not into a walking overload"

requirements-completed: [CAL-17, GAIT-06]

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "peakServoSpeed: peak servo-space speed of the trot at the five extreme commands (5.077 rad/s on the knee at the pinned v1 gait, period 0.55 s); knee_ratio scales only the knee (1.388 -> knee leads at 1.388*5.0771; 0.5 -> thigh leads at 4.232)"
    requirement: "GAIT-06"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_servo_limits.cpp#ServoLimits.PeakAtShippedGaitIs5p077"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_servo_limits.cpp#ServoLimits.KneeRatioScalesOnlyTheKnee"
        status: pass
    human_judgment: false
  - id: D2
    description: "minimalPeriod: windowed grid scan min_period + k * 0.0025 s, a period fits only when it and every grid period up to +0.05 s fit margin*max_speed (+1e-9); table 3.5->1.0300, 4.0->0.9050, 5.0->0.7200, 6.0->0.6000, 6.35->0.5575, 6.4->0.5500, 7.0->0.5500; never below min_period; degenerate inputs -> exactly 0.0"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_servo_limits.cpp#ServoLimits.MinimalPeriodTable"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_servo_limits.cpp#ServoLimits.PeriodNeverBelowMinimum"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_servo_limits.cpp#ServoLimits.NoFitReturnsZero"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_servo_limits.cpp#ServoLimits.WindowSkipsFragileFirstFit"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_servo_limits.cpp#ServoLimits.MinimalPeriodMatchesBruteForce"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_servo_limits.cpp#ServoLimits.ToleranceIsOneNanoUnit"
        status: pass
    human_judgment: false
  - id: D3
    description: "GAIT-06 tie: the model peak matches the real LocomotionController (maximum over the five extreme commands) to 5e-4 on the five reference periods and on two extra sets with foot offsets (about 4.737) and different hip_x/stand_height (about 4.226); red step executed: with the foot_offset_y term sabotaged the test failed on the offset set only"
    requirement: "GAIT-06"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_control/test/test_servo_limits.cpp#ServoLimits.PeakMatchesController"
        status: pass
    human_judgment: false
  - id: D4
    description: "CMake registration: src/servo_limits.cpp in dog_control_core and servo_limits in the test foreach; all six dog_control gtests build and pass locally under g++ -Werror"
    verification:
      - kind: other
        ref: "grep -F 'src/servo_limits.cpp' CMakeLists.txt; grep -F 'foreach(t kinematics gait crawl greet locomotion servo_limits)'; tools/local_gtest/run.sh dog_control {kinematics,gait,crawl,greet,locomotion,servo_limits} -> all rc=0, [  PASSED  ] lines"
        status: pass
    human_judgment: true
    rationale: "colcon is not available locally (D-24): only the textual registration and the g++ -Werror build of the same sources are proven here; the first colcon build on GCC 13/15 (CI, plan 01-08 wave) is what confirms the CMake edit"
  - id: D5
    description: "Plan backstops: the number corresponds to the real MG996R servo and the minimalPeriod runtime on the Banana Pi are not observable in this environment"
    verification: []
    human_judgment: true
    rationale: "Plan backstops: the bench measurement (01-14) and the CI walk_check show the real correspondence; the first node run (01-08) shows the Pi runtime (PC worst case measured here about 2 s for the full brute-force test, minimalPeriod itself tens of ms)"

# Metrics
duration: 8min
completed: 2026-10-05
status: complete
---

# Phase 1 Plan 04: Servo-speed limits (peak speed and minimal period) Summary

**ROS-free `servo_limits` core: peak 5.077 rad/s on the knee at the pinned v1 trot (D-20), a windowed period scan (3.5 -> 1.0300 ... 7.0 -> 0.5500, exactly 0.55 from 6.4 rad/s), tolerated at 1e-9 on both sides, and the model peak matched to the real `LocomotionController` at 5e-4 on three explicit parameter sets (GAIT-06).**

## Performance

- **Duration:** 8 min
- **Started:** 2026-10-05T09:10:39Z
- **Completed:** 2026-10-05T09:18:48Z
- **Tasks:** 3
- **Files modified:** 4 (3 created, 1 modified; plus the .planning close-out files)

## Accomplishments
- `peakServoSpeed(p, s)`: the JointSpeedsFitTheServos procedure on the real `TrotGait` + `inverseKinematics` (50 Hz, 1 s ramp, 4 s window, five extreme commands, combined first), strict maximum in servo space with the knee scaled by `knee_ratio`; bit-faithful to the controller's flat-ground path (same neutral feet, same ramp, same tick counts).
- `minimalPeriod(p, s, min_period, max_period)`: one-pass scan over the integer grid `min_period + k * kPeriodScanStep`; a period is accepted only when it and every grid period up to `kPeriodGuard` above it (clipped at `max_period`) fit `margin * max_speed + kSpeedTolerance`; returns 0.0 for no-fit and every degenerate input. The planned table reproduced exactly on GCC 14.2: 3.5 -> 1.0300, 4.0 -> 0.9050, 5.0 -> 0.7200, 6.0 -> 0.6000, 6.35 -> 0.5575, 6.4 -> 0.5500, 7.0 -> 0.5500.
- Nine `ServoLimits` gtests on the pinned reference set (`pinnedParams()` / `model()`, the same names plan 01-08 will reuse): window table, lower bound, no-fit, fragile-first-fit, an independent brute-force cross-check (1e-12), the 1-nano-unit tolerance from both sides, and the controller tie.
- TDD red/green evidence: task 1 and 2 verifications failed before the header/implementation existed (compile errors), then passed; task 3's red step sabotaged the duplicated `foot_offset_y` term, the test failed on the offset set (and only there), and passed again after the restore (the broken variant was never committed).
- CMake registration with the plan's exact shape: `src/servo_limits.cpp)` closing on the new last line of the library list and `servo_limits` appended to the test `foreach` (numstat vs main: 3 added, 2 removed).

## Task Commits

Each task was committed atomically:

1. **Task 1 (tracer): peakServoSpeed on the shipped trot (D-20, D-13)** - `75fa83d` (feat)
2. **Task 2: minimalPeriod scan with the guard window (D-11, D-12)** - `303c3bb` (feat)
3. **Task 3: CMake registration and PeakMatchesController (GAIT-06)** - `68885c2` (feat)

**Plan metadata:** `docs(01-04): complete servo-speed limits plan` (this commit: SUMMARY + STATE + ROADMAP + REQUIREMENTS)

## Files Created/Modified
- `ros2_ws/src/dog_control/include/dog_control/servo_limits.hpp` - forward-declared `LocomotionParams` (no includes), `kSpeedTolerance`, `kMaxAutoPeriod`, `kPeriodScanStep`, `kPeriodGuard`, `ServoSpeedModel`, `PeakSpeed`, `peakServoSpeed`, `minimalPeriod`
- `ros2_ws/src/dog_control/src/servo_limits.cpp` - neutralFeet/solveFeet/approach/extremeCommands/peakForCommand/fits in an anonymous namespace; no ROS, no state, no allocation
- `ros2_ws/src/dog_control/test/test_servo_limits.cpp` - pinnedParams()/model() reference set, peakAt, extremeCommandsOf, controllerPeak; nine tests
- `ros2_ws/src/dog_control/CMakeLists.txt` - `src/servo_limits.cpp` in `dog_control_core`, `servo_limits` in `foreach(t ...)`

## Decisions Made
- The algorithm tests never read shipped struct defaults: `pinnedParams()` spells out the v1 geometry/gait/max_velocity, `model()` spells out margin 0.8 and knee_ratio 1.0, so plan 01-15's defaults sync cannot redden them (OI-1).
- The pinned period table carries the guard-window values (0.9050, 0.5575 - flagged in the plan): the plan's own "first fit without the window" numbers (0.8975, 0.550) were not used because their neighbours within 0.05 s violate the gate.
- Degenerate inputs return 0.0 (never NaN or an exception), including `margin * max_speed <= 0` checked early; plan 01-08 maps 0.0 to a startup refusal.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking / literal ambiguity] CMake closing bracket placement**
- **Found during:** Task 3 (CMake registration)
- **Issue:** the task action says the closing bracket of `add_library(dog_control_core ...)` "goes to a new last line"; its own third verify command bounds the whole file diff at `<= 3 added / <= 2 removed` lines vs main. Putting the bracket alone on its own line yields 4 added / 2 removed and would fail that check; keeping it at the end of the new last line (`src/servo_limits.cpp)`) yields exactly 3 / 2.
- **Fix:** implemented the form that satisfies all of the task's greps and the numeric bound: `src/servo_limits.cpp)` closes the list on the new last line.
- **Files modified:** ros2_ws/src/dog_control/CMakeLists.txt
- **Verification:** `git diff --numstat main -- CMakeLists.txt` -> `3 2`; greps for both strings pass.
- **Committed in:** 68885c2

**2. [Note - must_haves closure] `allowed <= 0.0` in the early guard**
- **Found during:** Task 2 (minimalPeriod implementation)
- **Issue:** the action text lists the early return as "allowed or knee_ratio not finite or not positive" (attachable either way); the must_haves truth says `margin` not greater than 0 must give 0.0. With `margin = 0` and zero commands the scan alone would return `min_period` (peak 0 fits a zero gate plus tolerance).
- **Fix:** `allowed <= 0.0` is checked explicitly in the early guard, so the must_haves wording holds in every corner; no tested outcome changes (margin 0.0 already returned 0.0 through the scan for the pinned velocities).
- **Files modified:** ros2_ws/src/dog_control/src/servo_limits.cpp
- **Verification:** `ServoLimits.NoFitReturnsZero` and the full suite pass.
- **Committed in:** 303c3bb

---

**Total deviations:** 1 ambiguity resolved against the plan's own numeric check (CMake), 1 stated-input closure per must_haves (`allowed <= 0.0`). No auto-fix touched behavior outside the plan's own acceptance criteria.
**Impact on plan:** none on the tested behaviour; every automated check and acceptance criterion of the plan passes.

## Issues Encountered
- None. The red/green steps behaved exactly as designed (two compile-fail red steps, one deliberate formula-break red step failing precisely on the offset set). The pinned numbers (5.077; 4.737; 4.226; the seven-row period table) reproduced on GCC 14.2 as planned, so no re-planning was needed.

## User Setup Required
None - no external service configuration required.

## Next Phase Readiness
- Plan 01-08 can include `servo_limits.hpp` (no cycle: the header only forward-declares `LocomotionParams`) and wire `period = minimalPeriod(p, p.servo, p.min_period, kMaxAutoPeriod)`, turning 0.0 into a startup refusal; plan 01-15 syncs the shipped defaults without touching the pinned test numbers.
- Backstops carried by the plan (not resolvable here): the correspondence of the number to the real MG996R servo (bench measurement, 01-14; CI walk_check) and the `minimalPeriod` runtime on the Banana Pi (first node run, 01-08; PC side measured: the heaviest test here, the brute-force cross-check, runs in ~1.3 s).
- CAL-17 and GAIT-06 stay open in REQUIREMENTS.md: both IDs are declared by sibling plans without summaries yet (`requirements.ready-ids`: 0/2 ready), so the shared-ID gate defers completion as designed.
- No simulations, `colcon` or `ros2` were run (D-24); the colcon/GCC 13 and 15 build of these files is checked in CI / the plan 01-08 wave.

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-05*

## Self-Check: PASSED
