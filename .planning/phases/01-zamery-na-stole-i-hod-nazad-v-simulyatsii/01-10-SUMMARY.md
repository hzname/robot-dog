---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 10
subsystem: simulation
tags: [servo-profile, urdf, gazebo, backlash, delay, knee-ratio, body-com, pytest, ci]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 02
    provides: "robot.yaml servo.* and servo_sim.* blocks and description.body_com_x (the Phase 1 config contract this plan consumes without redefining)"
provides:
  - "dog_description.servo_profile: ServoProfile.from_params, speed_factor/torque_factor over the MG996R 4.8-6.6 V range, BacklashPlay, DelayLine, CommandShaper, parse_override, bridge_settings"
  - "build_urdf(..., servo_model='ideal'|'real') with SERVO_MODELS, joint friction, bus-voltage factors and the knee rod ratio; CLI --servo-model; body_com_x in the trunk inertia"
  - "joint_command_bridge backlash_deg/delay_s through CommandShaper; sim.launch.py arguments servo_model, servo_speed, servo_delay_ms"
affects: [01-13, 01-17, phase 2 (GAIT-05 varies the servo_sim reference numbers)]

# Actuals (#2632) — pairs with the plan's estimate
actuals:
  tokens: 10617
  tasks: 4
  commits: 4
plan_head_before: 20e62dc904eb13784e45ebfbab6440b7b8e06fe1

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Pure-stdlib profile module in dog_description so its tests ride the existing dog_description pytest job on both distros (dog_gazebo is skipped in CI)"
    - "Ideal-model byte guard: PINNED_DESC + SHA-256 of the two URDF strings, taken before the edit and independent of DEFAULT_DESCRIPTION"
    - "Bridge parameters carry the profile: zeros reproduce the old same-call forwarding, the drain timer exists only for a non-zero delay"

key-files:
  created:
    - ros2_ws/src/dog_description/dog_description/servo_profile.py
    - ros2_ws/src/dog_description/test/test_servo_profile.py
  modified:
    - ros2_ws/src/dog_description/dog_description/urdf.py
    - ros2_ws/src/dog_description/test/test_urdf.py
    - ros2_ws/src/dog_gazebo/dog_gazebo/joint_command_bridge.py
    - ros2_ws/src/dog_gazebo/launch/sim.launch.py

key-decisions:
  - "servo_profile.py is pure stdlib (math/dataclasses/collections) in dog_description; the bridge and the launch file only import it, so all new tests run in the existing dog_description pytest job on Jazzy and Lyrical (no ci.yml change, D-24)"
  - "The ideal guard is PINNED_DESC + GOLDEN_SHA256 (sha256 of the plain and Gazebo URDF strings, taken on the untouched tree); the pin does not follow DEFAULT_DESCRIPTION, so plan 01-15 can sync code defaults without reddening it (D-15)"
  - "With zeros the bridge forwards each position in the same call with the previous order and numbers and creates no timer; the 500 Hz drain timer exists only when delay_s > 0 (D-15)"
  - "knee_ratio k: the knee joint gets velocity/k and effort*k, rounded once from the exact numbers; the calf cmd_max follows the joint's own velocity (D-20)"
  - "servo_speed overrides only description.servo_velocity in the URDF (physical speed); the overrides dict keeps no servo.* keys and servo.max_speed is never touched (D-13)"

patterns-established:
  - "Profile numbers validated with ValueError naming the field before any Gazebo run (T-01-10-01); XML carries only formatted numbers"
  - "CI-only verification of the launch/bridge wiring: py_compile locally, both jobs on both distros in GitHub Actions (D-24)"

requirements-completed: [CAL-16, GAIT-10]

# Coverage metadata (#1602) — one entry per shipped deliverable
coverage:
  - id: D1
    description: "ServoProfile with the servo_sim reference defaults and from_params (missing keys keep defaults, extra keys ignored, bad values rejected with the key name, D-16)"
    requirement: "GAIT-10"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_profile_from_params"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_profile_defaults_match_robot_yaml"
        status: pass
    human_judgment: false
  - id: D2
    description: "Bus-voltage factors speed_factor/torque_factor: 4.8/5.2/6.0/6.6 V values, clamp to [4.8, 6.6], v_ref inside the range, non-finite/bool/string rejected (D-14)"
    requirement: "GAIT-10"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_speed_and_torque_factors"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_voltage_clamped"
        status: pass
    human_judgment: false
  - id: D3
    description: "build_urdf(servo_model='real') on the shipped robot.yaml: 12 revolute joints with dynamics damping 0 / friction 0.06, effort/velocity from the factors, the knee scaled by knee_ratio, calf cmd_max following the joint (D-14, D-20)"
    requirement: "GAIT-10"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_real_urdf_has_12_friction_joints"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_real_urdf_scales_by_voltage_and_knee_ratio"
        status: pass
      - kind: integration
        ref: "GitHub Actions run 37362699921 'build + test (jazzy|lyrical)': 246 tests, 0 failures (servo_profile.py built on 3.12 and 3.14)"
        status: pass
    human_judgment: false
  - id: D4
    description: "BacklashPlay (dead zone, reversal, first command, zero-width passthrough, non-finite state-safe), DelayLine (order, simulation-time release, backwards clock), CommandShaper passthrough at zeros, bridge_settings and parse_override (D-14, D-15)"
    requirement: "GAIT-10"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_backlash_dead_zone_and_reversal"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_delay_line_orders_by_sim_time"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_shaper_zero_is_passthrough"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_shaper_applies_backlash_then_delay"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_bridge_settings"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_servo_profile.py#test_parse_override"
        status: pass
    human_judgment: false
  - id: D5
    description: "body_com_x shifts the trunk inertial origin along x (0.012 / -0.02), zero keeps the previous bytes, visual and collision stay centred, NaN/inf/bool rejected on both models (D-21, CAL-16)"
    requirement: "CAL-16"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_urdf.py#test_body_com_x_moves_trunk_inertial"
        status: pass
    human_judgment: false
  - id: D6
    description: "The ideal model stays byte for byte the default: golden SHA-256 guard green and the Gazebo walk check unchanged on both distros (D-15)"
    requirement: "GAIT-10"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_description/test/test_urdf.py#test_ideal_urdf_unchanged"
        status: pass
      - kind: integration
        ref: "GitHub Actions run 37362699921 'gazebo walk check (jazzy|lyrical)': all 10 checks PASS on both distros (backward 59% / 71% of command)"
        status: pass
    human_judgment: false
  - id: D7
    description: "Bridge and launch wiring: backlash_deg/delay_s parameters, CommandShaper import, one create_timer, servo_model/servo_speed/servo_delay_ms arguments, bridge settings passed in, LogInfo for a delay on ideal (D-13, D-14, D-25)"
    requirement: "GAIT-10"
    verification:
      - kind: other
        ref: "python3 -m py_compile joint_command_bridge.py sim.launch.py -> rc 0"
        status: pass
      - kind: integration
        ref: "GitHub Actions run 37362699921 'build + test (jazzy|lyrical)' and 'gazebo walk check (jazzy|lyrical)': both success; ci_dispatch.sh --run-id exited 0"
        status: pass
    human_judgment: false
  - id: D8
    description: "servo_model:=real is ready for its first Gazebo run (01-13): the profile's physical behaviour (friction vs the SERVO constraint, backlash/delay on the walk) is only observable in a simulation"
    verification: []
    human_judgment: true
    rationale: "No local simulation is allowed (D-24); the real-model acceptance runs in plans 01-13 and 01-17. This plan ships the code path and its unit tests only."

# Metrics
duration: 58 min
completed: 2026-10-05
status: complete
---

# Phase 1 Plan 10: Realistic servo profile (servo_model:=real) and body_com_x Summary

**`servo_model:=real` ships: a pure-stdlib servo profile (bus-voltage factors, friction, knee rod ratio, backlash, command delay) through the URDF, the joint bridge and sim.launch.py, with the ideal model pinned byte-for-byte by SHA-256 and CI green on both distros.**

## Performance

- **Duration:** 58 min
- **Started:** 2026-10-05T19:12:14Z
- **Completed:** 2026-10-05T20:10Z
- **Tasks:** 4
- **Files modified:** 6 (plus the .planning close-out files)

## Accomplishments

- `dog_description/servo_profile.py` (new, pure stdlib): `ServoProfile.from_params` (defaults equal the shipped `servo_sim` block, D-16), `speed_factor`/`torque_factor` (MG996R 4.8–6.6 V two-point slopes, clamp, v_ref inside the range), `BacklashPlay`, `DelayLine` (released on the caller's clock), `CommandShaper`, `parse_override`, `bridge_settings`.
- `build_urdf(..., servo_model='ideal'|'real')` + CLI `--servo-model`: `real` writes `<dynamics damping="0" friction="0.06"/>` on all 12 revolute joints, scales effort/velocity by the bus-voltage factors, divides the knee velocity / multiplies its effort by `knee_ratio` (D-20) and gives the calf its own `cmd_max`; unknown models are rejected; `load_config` now carries the `servo_sim` and `servo` blocks.
- Ideal guard: `PINNED_DESC` + `_ideal_cases()` + `GOLDEN_SHA256` (plain and Gazebo URDF strings hashed on the untouched tree); `test_ideal_urdf_unchanged` proves D-15 byte-for-byte, including with the new keys present.
- `body_com_x` (D-21, CAL-16) shifts the trunk inertial origin along x; a missing key and an explicit 0.0 keep the previous bytes; NaN/inf/bool rejected on both models.
- `joint_command_bridge`: `backlash_deg` and `delay_s` parameters through `CommandShaper`; with zeros the same-call forwarding, order and numbers as before and no timer; the 500 Hz drain timer exists only when `delay_s > 0`.
- `sim.launch.py`: arguments `servo_model` (default `ideal`), `servo_speed` (physical speed only, D-13) and `servo_delay_ms`; bridge settings from `bridge_settings`; `LogInfo` when a delay is passed on the ideal model.
- CI (run 37362699921, branch `gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii`): `gazebo walk check` and `build + test` **success on both Jazzy and Lyrical**; `ci_dispatch.sh --run-id` exited 0. Walk check: all 10 checks PASS on both distros (backward 59 % / 71 % of command — the ideal path did not change); build + test: 246 tests, 0 failures on both, `21 passed` for dog_description.

## Task Commits

Each task was committed atomically:

1. **Task 1 (tracer): ServoProfile, factors, build_urdf(servo_model='real'), ideal guard** - `3d27952` (feat)
2. **Task 2: BacklashPlay, DelayLine, CommandShaper, bridge_settings, parse_override** - `140d726` (feat)
3. **Task 3: body_com_x in the trunk inertia** - `a3b9318` (feat)
4. **Task 4: bridge CommandShaper + sim.launch.py arguments, CI** - `074558b` (feat)

**Plan metadata:** `docs(01-10): complete servo profile plan` (this commit: SUMMARY + STATE + ROADMAP + REQUIREMENTS + deferred-items)

## Files Created/Modified

- `ros2_ws/src/dog_description/dog_description/servo_profile.py` - new profile/shaper module (constants, `_number`, `_non_negative`, factors, `ServoProfile`, `BacklashPlay`, `DelayLine`, `CommandShaper`, `parse_override`, `bridge_settings`)
- `ros2_ws/src/dog_description/test/test_servo_profile.py` - new: 12 tests for the profile, shaper and bridge selection
- `ros2_ws/src/dog_description/dog_description/urdf.py` - `servo_model`, `SERVO_MODELS`, `load_config` servo blocks, real profile (dynamics, factors, knee), `knee_vmax` in `_gazebo_extras`, `body_com_x`, `--servo-model`
- `ros2_ws/src/dog_description/test/test_urdf.py` - `PINNED_DESC`, `_ideal_cases`, `GOLDEN_SHA256`, 5 new tests (guard, real URDF, CLI, load_config, body_com_x)
- `ros2_ws/src/dog_gazebo/dog_gazebo/joint_command_bridge.py` - `backlash_deg`/`delay_s`, `CommandShaper`, `on_timer`/`_drain`/`_now`, `TICK_S`
- `ros2_ws/src/dog_gazebo/launch/sim.launch.py` - `servo_model`/`servo_speed`/`servo_delay_ms` arguments, profile wiring into URDF and bridge, ideal-delay LogInfo

## Decisions Made

- The profile lives in `dog_description` as pure stdlib so its tests run in the existing pytest job on both distros (dog_gazebo is skipped in CI).
- The ideal guard pins literal strings via SHA-256 instead of following `DEFAULT_DESCRIPTION`, so plan 01-15's default sync cannot redden it.
- Zeros (ideal) keep the old bridge behaviour exactly: same-call forwarding, no timer; the drain timer exists only for a non-zero delay.
- Knee rod drive: joint velocity = servo speed / k, joint effort = servo effort × k, rounded once from the exact numbers; the calf `cmd_max` follows the joint (D-20).
- `servo_speed` overrides only `description.servo_velocity`; `servo.max_speed` and `max_joint_speed` stay untouched (D-13).

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered

- **GitHub Actions cancelled queued jobs (~15 min without a runner), twice.** Attempt 1: `build + test (lyrical)` was cancelled while queued (the two walk checks and the jazzy build stayed green). Fixed with `gh run rerun --failed` (attempt 2): `build + test (lyrical)` passed; `ci_dispatch.sh --run-id 37362699921` then exited 0 with all four awaited jobs success. Attempt 2 also saw `robot image (arm64)` and `robot parameter form` cancelled after the same ~15 min wait — both outside this plan's awaited set (robot-setup passed in the 18:18 run and its files are untouched here). Logged to `deferred-items.md`.
- **`ci_dispatch.sh` state-parse quirk:** during an in-progress run its TSV state line shifts fields (empty `conclusion`), so it prints `warning: incomplete status` until the run concludes, and after a rerun it can decide from the pre-rerun attempt. Not fixed (tool from plan 01-01, outside this plan's files); logged to `deferred-items.md`.
- Local interpreter: every suite/acceptance command ran with `/usr/bin/python3` (3.13.5, pytest 9.0.3, numpy 2.4.3), like the earlier plans; the bare `python3` is the Hermes toolchain interpreter without pytest/numpy. No repository change.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Plans 01-13 and 01-17 can run `servo_model:=real` (and vary `servo_speed` / `servo_delay_ms`) through `sim.launch.py`, and 01-13 can generate the real URDF for its `gz sdf -p` check with `generate_urdf --servo-model real`.
- `CAL-16` and `GAIT-10` stay unchecked in REQUIREMENTS.md: the `requirements.ready-ids` gate reports 0/2 ready because sibling plans (01-15 for CAL-16; 01-13, 01-17 for GAIT-10) have not produced summaries yet.
- The real profile's physical behaviour (friction with the SERVO constraint, backlash/delay on the walk) is unverified until 01-13/01-17 — coverage D8 is explicitly human/judgment-dependent.

## Self-Check: PASSED

- Files exist: `servo_profile.py`, `test_servo_profile.py` (FOUND), plus the four modified files staged in their task commits.
- Commits exist: `3d27952`, `140d726`, `a3b9318`, `074558b` (all FOUND in `git log --all`).
- Plan-level verification: `pytest ros2_ws/src/dog_description/test` -> 21 passed; `py_compile` -> rc 0; `git diff --name-only main...HEAD -- ros2_ws/src/dog_hardware .github/workflows/ci.yml` -> empty; CI run 37362699921: `gazebo walk check` and `build + test` success on Jazzy and Lyrical; `ci_dispatch.sh` exited 0.

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-05*
