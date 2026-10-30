---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 15
subsystem: config-contract
tags: [yaml-contract, code-defaults, guards, robot_setup, cal-16, cal-17, reduced-scope]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 02
    provides: "the Phase 1 config contract in robot.yaml (gait.auto_period, servo.*, servo_sim.*, description.body_com_x) and the first guards in test_yaml_contract.py"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 10
    provides: "test_profile_defaults_match_robot_yaml (the Python side of the same rule) and the pinned ideal-model golden"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 14
    provides: "01-SERVO-SPEED-RESULT.md, whose contract lines are PENDING: there is no measured servo speed to enter"
provides:
  - "Two guards that the C++ defaults equal robot.yaml: LegGeometry, LocomotionParams (geometry, stance, auto_period), ServoSpeedModel, and DEFAULT_DESCRIPTION via ast"
  - "A checked statement that the code defaults and the YAML agree today, so the synchronization part of the phase carries no number changes"
affects: [01-16, phase 02 (any YAML change now fails CI until the C++ default follows)]

# Actuals (#2632)
actuals:
  tokens: 900
  tasks: 1
  commits: 1

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "A config contract is guarded on both sides: the Python default is compared with the YAML by test_profile_defaults_match_robot_yaml, the C++ defaults by parsing the headers (struct body + field default)"
    - "The C++ side is read with a brace-matching struct extractor and a field regex over double|float|int|bool name{...}, so no C++ compiler is needed in the pytest job"

key-files:
  modified:
    - tools/robot_setup/test/test_yaml_contract.py

key-decisions:
  - "Plan 01-15 is closed with the synchronization only (owner's decision 2026-10-26): no measured numbers are entered, gait.auto_period stays false"
  - "The owner's table of 2026-10-06 in 01-MEASUREMENTS-RAW.md is not applied: the owner closed the plan without entering the numbers; the leg has not been rebuilt since the measurement, so the table stays a valid record"
  - "The measured 12.065 us/deg and the resulting ~141 deg travel of the 520..2220 us window are accepted as the configuration of this servo, not carried as an open question"

patterns-established:
  - "When a plan's numbers are not available, the plan can still close on the part that does not depend on them, with the unmet requirements left explicitly open"

requirements-completed: []

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "Guards that the code defaults equal robot.yaml: geometry, stance heights, foot offsets, servo model numbers, auto_period and DEFAULT_DESCRIPTION"
    requirement: "CAL-16"
    verification:
      - kind: unit
        ref: "python3 -m pytest -q tools/robot_setup/test -> 62 passed (60 before, +2 guards)"
        status: pass
      - kind: unit
        ref: "negative check: CONFIG pointed at a copy with geometry.hip_offset 0.045 -> AssertionError ('hip', 'geometry.hip_offset'), so the guard bites"
        status: pass
    human_judgment: false
  - id: D2
    description: "The measured geometry, masses and servo speed entered into the config"
    verification: []
    human_judgment: true
    rationale: "Not delivered by the owner's decision: the plan was closed without entering the 2026-10-06 table, and the servo speed measurement was rejected (01-14). CAL-16 and CAL-17 stay open; whether to enter the existing table, retake it or leave the config as it is belongs to the owner."

# Metrics
duration: 40min
completed: 2026-10-26
status: complete
---

# Phase 1 Plan 15: code defaults stay equal to robot.yaml (reduced scope)

**The synchronization half of the plan is done: two pytest guards now fail whenever the C++ defaults (LegGeometry, LocomotionParams, ServoSpeedModel) or DEFAULT_DESCRIPTION drift from robot.yaml — the defaults already match, so no number changed, and the measurement half of the plan is closed unfilled by the owner's decision.**

## Performance

- **Duration:** 40 min
- **Started:** 2026-10-26T12:35Z
- **Completed:** 2026-10-26T13:15Z
- **Tasks:** 1 of 4 (the other three need numbers that do not exist)
- **Files modified:** 1 code file, 5 planning files

## Accomplishments

- **Guards (task 2/3, the part that does not need numbers):** `tools/robot_setup/test/test_yaml_contract.py` gains `_read`, `_cpp_struct` (brace matching, line comments stripped), `_cpp_field` (default of `double|float|int|bool name{...}`), `_default_description` (AST, no package import) and two tests:
  - `test_cpp_defaults_match_geometry_in_robot_yaml` — `LegGeometry` {hip, thigh, calf} against `geometry.*`; `hip_x`, `hip_y`, the four stance heights and both foot offsets of `LocomotionParams` against `geometry.*` / `stance.*`; `DEFAULT_DESCRIPTION` against the `description` block, keys and values.
  - `test_cpp_defaults_match_servo_model_and_auto_period` — `max_speed`, `margin`, `knee_ratio` of `ServoSpeedModel` against `servo.*`; `auto_period` against `gait.auto_period`.
- **Both green today, both bite on drift:** values parsed are hip 0.055 / thigh 0.105 / calf 0.105, hip_x 0.09 / hip_y 0.06, heights 0.15 / 0.08 / 0.10 / 0.18, servo 6.0 / 0.8 / 1.0, `auto_period` false, 15 `DEFAULT_DESCRIPTION` keys — all equal to the YAML. A copy with `geometry.hip_offset` 0.045 makes the guard report `('hip', 'geometry.hip_offset')`.
- **Checks:** `tools/robot_setup/test` 62 passed (60 before), `dog_description` 22 passed, `robot_setup --check` clean.
- **Record:** the owner's table of 2026-10-06 (`01-MEASUREMENTS-RAW.md`) is committed as history with a header that it is not applied; the plan closes without touching a single YAML value.

## Task Commits

1. **Synchronization guards** - `26e8fe4` (test)
2. **Plan close (SUMMARY, raw-measurement note, STATE, ROADMAP, REQUIREMENTS)** - this commit (docs)

## Files Created/Modified

- `tools/robot_setup/test/test_yaml_contract.py` - the two guards and their helpers
- `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-MEASUREMENTS-RAW.md` - committed with a note that the numbers belong to the previous leg and are not applied
- `.planning/STATE.md`, `.planning/ROADMAP.md`, `.planning/REQUIREMENTS.md` - plan 15 closed with the reduced scope, CAL-16/CAL-17 still open

## Decisions Made

- **Synchronization only.** The owner closed the plan on 2026-10-26 with the wording "01-15 закрой, [оставь] синхронизацию": the guards are the deliverable, the numbers are not entered.
- **The 2026-10-06 table stays a record, not input.** It carries the geometry of the leg that is still on the robot (thigh 100, calf 111, hip_offset 35 mm) and has not been rebuilt since; the owner closed the plan without entering the numbers, and this plan does not decide the geometry.
- **The servo scale is a fact, not a question.** The measured 12.065 us/deg and the ~141 deg travel of the 520..2220 us window are recorded as the configuration of these servos; no window change is proposed.

## Deviations from Plan

- **Tasks 1-4 of the plan are not executed as written.** Task 1's data exists (`01-MEASUREMENTS-RAW.md`, 2026-10-06) but the owner closed the plan without entering it; task 2 therefore has nothing to write; task 3 has no servo speed (01-14 verdict `unusable`); task 4 (CI on measured numbers) would check nothing new. The owner accepted this and closed the plan.
- **The plan's mask check and CI dispatch were not used:** no `dog_control` test was edited and no YAML number changed, so there is nothing for either to verify.

## Issues Encountered

- The measured table sat uncommitted since 2026-10-06, which is why the plan looked executable. It is committed now with a note that the owner closed the plan without applying it, so the next reviewer sees the provenance and the decision.

## User Setup Required

None.

## Next Phase Readiness

- **01-16 is unblocked:** it works on the simulator and the baseline, not on the measured numbers.
- **CAL-16 and CAL-17 stay open:** the geometry table is recorded but not entered (owner's decision), and the servo speed measurement was rejected; both need the owner's disposition.
- **A YAML change now fails CI until the matching C++ default follows** — that is the point of the guards.

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-26*
