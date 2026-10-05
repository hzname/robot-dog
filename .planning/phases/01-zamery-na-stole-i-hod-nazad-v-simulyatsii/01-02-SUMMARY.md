---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 02
subsystem: configuration
tags: [yaml, ros2-parameters, robot_setup, pytest, four-bar-linkage, contract-tests]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 01
    provides: "phase branch (gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii) and CI helpers; this plan itself runs only local pytest (D-24)"
provides:
  - "ros2_ws/src/dog_bringup/config/robot.yaml: the Phase 1 key contract (gait.auto_period/min_period, servo.{max_speed,margin,knee_ratio}, servo_sim.{backlash_deg,delay_ms,friction_nm,bus_voltage,bus_voltage_ref}, description.body_com_x) with a source and unit comment for every new number"
  - "tools/robot_setup/test/test_yaml_contract.py: key/type contract, the three-servo-speeds invariant (D-13) and the comment-with-unit rule; runs in the existing robot-setup CI job with no ci.yml change"
  - "tools/robot_setup/robot_setup.py: body_com_x form field with the beyond-hip warning; SENSORS_ON_ROBOT so simulated-only sensor geometry is a warning; knee_ratio_max() port of the driver Linkage and the servo.knee_ratio check warning"
affects: [01-04, 01-08, 01-10, 01-15, phase 01 verification, phase 2 (servo profile around the servo_sim reference values)]

# Actuals (#2632) - pairs with the plan's estimate
actuals:
  tokens: 6478
  tasks: 4
  commits: 4

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "One plan fixes the YAML key names; consuming plans never redefine them (D-13 names frozen here)"
    - "Config contract pinned by local pytest in tools/robot_setup/test, which the existing robot-setup CI job already runs (no ci.yml edit)"
    - "Driver Linkage math ported standalone into robot_setup (stdlib + PyYAML only, no tools/autocal import) and pinned against C++ golden values"

key-files:
  created:
    - tools/robot_setup/test/test_yaml_contract.py
  modified:
    - ros2_ws/src/dog_bringup/config/robot.yaml
    - tools/robot_setup/robot_setup.py
    - tools/robot_setup/test/test_robot_setup.py

key-decisions:
  - "The Phase 1 config contract is fixed by plan 01-02: robot.yaml carries gait.auto_period/min_period, servo.*, servo_sim.* (reference values with a source and unit each) and description.body_com_x; plans 01-04, 01-08, 01-10 and 01-15 consume these names without redefining them"
  - "The three servo speeds (description.servo_velocity, servo.max_speed, servos.yaml max_joint_speed) are pinned as one number by tools/robot_setup/test/test_yaml_contract.py; a divergence fails the existing robot-setup job (D-13)"
  - "robot_setup keeps measurement entry unblocked: SENSORS_ON_ROBOT = frozenset() downgrades simulated-only sensor geometry errors to warnings (D-18), and knee_ratio_max (the driver Linkage, C++ golden values) makes --check warn when robot.yaml servo.knee_ratio is below the computed ratio (D-20)"

requirements-completed: [CAL-16, CAL-17, GAIT-10]

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "robot.yaml carries the Phase 1 contract keys with the current behaviour, a source comment and a unit for every new number (auto_period false, servos.yaml untouched, sim_p_gain comment fixed)"
    requirement: "CAL-16"
    verification:
      - kind: unit
        ref: "tools/robot_setup/test/test_yaml_contract.py#test_contract_keys_exist_with_the_right_types"
        status: pass
      - kind: unit
        ref: "tools/robot_setup/test/test_yaml_contract.py#test_three_servo_speeds_are_one_number"
        status: pass
      - kind: unit
        ref: "tools/robot_setup/test/test_yaml_contract.py#test_servo_blocks_carry_a_comment_with_a_unit"
        status: pass
      - kind: other
        ref: "python3 tools/robot_setup/robot_setup.py --check -> rc 0"
        status: pass
      - kind: other
        ref: "git diff -U0 e480bb3^ e480bb3 -- robot.yaml -> exactly one removed line (sim_p_gain comment)"
        status: pass
    human_judgment: false
  - id: D2
    description: "tools/robot_setup/test/test_yaml_contract.py is a standalone contract test (no robot_setup import) picked up by the existing robot-setup CI job"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "python3 -m pytest -q tools/robot_setup/test/test_yaml_contract.py -> 16 passed"
        status: pass
      - kind: other
        ref: "grep -n 'pytest -q tools/robot_setup/test' .github/workflows/ci.yml -> line 64 (no ci.yml change)"
        status: pass
    human_judgment: false
  - id: D3
    description: "body_com_x goes through the whole tool: load, validate (range error, beyond-hip warning), save keeping comments and a one-line diff, and the --cli path writing 0.0125 (D-21, D-26)"
    requirement: "CAL-16"
    verification:
      - kind: unit
        ref: "tools/robot_setup/test/test_robot_setup.py#test_body_com_x_is_loaded_saved_and_keeps_comments"
        status: pass
      - kind: unit
        ref: "tools/robot_setup/test/test_robot_setup.py#test_cli_enters_body_com_x_end_to_end"
        status: pass
      - kind: unit
        ref: "tools/robot_setup/test/test_robot_setup.py#test_body_com_x_out_of_range_is_an_error"
        status: pass
      - kind: unit
        ref: "tools/robot_setup/test/test_robot_setup.py#test_body_com_x_beyond_the_hip_axes_warns"
        status: pass
      - kind: other
        ref: "python3 -c \"...rs.FIELDS['body_com_x']...\" -> description body_com_x 1000 -60 60"
        status: pass
    human_judgment: false
  - id: D4
    description: "Sensor geometry errors of simulated-only sensors do not block save or --check (a GS2 warning remains); SENSORS_ON_ROBOT restores the error and a flag of 0 still skips the check (D-18)"
    requirement: "CAL-16"
    verification:
      - kind: unit
        ref: "tools/robot_setup/test/test_robot_setup.py#test_sensor_errors_do_not_block_save_and_check_while_the_sensors_are_not_on_the_robot"
        status: pass
      - kind: unit
        ref: "tools/robot_setup/test/test_robot_setup.py#test_sensor_errors_block_for_sensors_on_the_robot"
        status: pass
      - kind: unit
        ref: "tools/robot_setup/test/test_robot_setup.py#test_sensor_checks_are_skipped_when_the_flag_is_off"
        status: pass
    human_judgment: false
  - id: D5
    description: "knee_ratio_max computes the four-bar knee speed ratio like the driver Linkage (C++ golden values), and --check warns when robot.yaml servo.knee_ratio is lower (D-20)"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "tools/robot_setup/test/test_robot_setup.py#test_knee_ratio_matches_the_cpp_linkage"
        status: pass
      - kind: unit
        ref: "tools/robot_setup/test/test_robot_setup.py#test_knee_ratio_warning_in_validate_and_check"
        status: pass
      - kind: other
        ref: "python3 -c \"print(round(rs.knee_ratio_max(15, 20, 95, 95), 2))\" -> 1.39"
        status: pass
    human_judgment: false
  - id: D6
    description: "Claim that the new keys keep the current behaviour (auto_period off; nodes build without reading the undeclared keys, D-12, D-15)"
    verification: []
    human_judgment: true
    rationale: "Not locally provable: no colcon/ros2/Gazebo in this environment (D-24). The round-trip test, --check and the URDF tests pass on the new YAML, but the no-behaviour-change claim is only observable in the phase-branch CI run / a simulation; the phase verifier should confirm there."

# Metrics
duration: 8 min
completed: 2026-10-05
status: complete
---

# Phase 1 Plan 02: Config contract and robot_setup checks Summary

**Phase 1 config keys frozen in robot.yaml (11 new keys, every number with a source comment and unit), pinned by a new standalone contract test, plus body_com_x end-to-end, non-blocking simulated-only sensor checks and a knee-ratio port of the driver Linkage in robot_setup.**

## Performance

- **Duration:** 8 min
- **Started:** 2026-10-05T08:41:26Z
- **Completed:** 2026-10-05T08:49:51Z
- **Tasks:** 4
- **Files modified:** 4 (plus the .planning close-out files)

## Accomplishments
- `robot.yaml`: `gait.auto_period: false`, `gait.min_period: 0.55`, the `servo` block (`max_speed` 6.0, `margin` 0.8, `knee_ratio` 1.0), the `servo_sim` reference block (backlash 1.5 deg, delay 40 ms, friction 0.06 N*m, bus 6.0 V / ref 6.0 V) and `description.body_com_x: 0.0`; the `sim_p_gain` comment now says it is unused in the velocity-command mode; existing values and all other files untouched.
- `tools/robot_setup/test/test_yaml_contract.py`: standalone contract test (key types, three-servo-speeds invariant per D-13, margin/knee-ratio bounds, min_period in seconds, physical servo_sim numbers, comment-with-unit rule) — caught by the existing `robot-setup` CI job without touching `ci.yml`.
- `body_com_x` runs the full tool path (YAML -> load -> form -> validate -> render/save -> YAML) including the `--cli` route writing 0.0125 m (D-26), with a warning when the centre of mass is beyond the hip axes; saving keeps comments and changes one line.
- `SENSORS_ON_ROBOT = frozenset()` (docs/HEAD.md: no perception sensors on the robot) downgrades GS2/ToF/lidar geometry errors to visible warnings, so a new stand_height or hip geometry no longer blocks `save`/`--check`; adding an identifier restores the error (D-18).
- `knee_ratio_max` ports the driver four-bar (servo_model / servo_driver.hpp) with C++ golden values (1.605…1.334…1.507 with a 0.003 tolerance; 15/20/95/95 mm gives 1.388 in the knee walk range); `--check` and the web form warn when `servo.knee_ratio` under-states it (D-20).

## Task Commits

Each task was committed atomically:

1. **Task 1: contract keys in robot.yaml and test_yaml_contract.py** - `e480bb3` (feat)
2. **Task 2: body_com_x end-to-end through robot_setup** - `3f7e1c9` (feat)
3. **Task 3: simulated-only sensor errors do not block measurement entry** - `8ec1d49` (feat)
4. **Task 4: knee_ratio_max port and --check warning** - `50cb5bf` (feat)

**Plan metadata:** `docs(01-02): complete config contract and robot_setup plan` (this commit: SUMMARY + STATE + ROADMAP)

## Files Created/Modified
- `ros2_ws/src/dog_bringup/config/robot.yaml` - the Phase 1 contract keys with comments/sources; `sim_p_gain` comment fixed; nothing else changed (one removed line vs the previous commit)
- `tools/robot_setup/robot_setup.py` - `body_com_x` form field and beyond-hip warning; `SENSORS_ON_ROBOT` + `_sensor_flag`; `KNEE_CENTER_DEG`, the Linkage port helpers, `knee_ratio_max`, `servo_knee_ratio` in load/validate, web-form pickup in `do_POST`
- `tools/robot_setup/test/test_robot_setup.py` - 11 new tests for body_com_x, sensor checks and knee ratio (36 tests in the file, 42 with the contract file)
- `tools/robot_setup/test/test_yaml_contract.py` - new contract test (16 cases)

## Decisions Made
- Names and formats of the Phase 1 keys are frozen by this plan; consuming plans (01-04, 01-08, 01-10, 01-15) must not redefine them.
- The three servo speeds are kept equal by a contract test in the existing robot-setup job, not by a new CI job.
- Measurement entry stays unblocked: simulated-only sensor geometry is a warning (`SENSORS_ON_ROBOT` empty until a sensor is really installed), and `servo.knee_ratio` under-statement is a warning, never a block.

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered
- The session `python3` is the Hermes toolchain interpreter (3.14, no pytest); every suite and acceptance command was run with `/usr/bin/python3` (Python 3.13.5, pytest 9.0.3, PyYAML 6.0.3), which is the machine's system python. No repository change; CI keeps `python -m pytest` as is.
- Simulations and CI were not run (D-24): the plan's verification is local pytest plus `robot_setup --check`; the new contract test will run in the next phase-branch CI dispatch in the existing `robot-setup` job.

## User Setup Required
None - no external service configuration required.

## Next Phase Readiness
- Ready for the next plans of wave 2/3: 01-04, 01-08, 01-10 and 01-15 can rely on the frozen key names, the `service` contract test and `robot_setup` behaviour.
- `CAL-16`, `CAL-17` and `GAIT-10` stay unchecked in REQUIREMENTS.md until the sibling plans that also declare them finish (`requirements.ready-ids` gate); this plan alone does not complete them.
- `servo.margin` 0.8 and all `servo_sim` numbers are reference values: the owner's bench measurement (plan 01-14 area) and Phase 2 vary around them (D-11, D-16).
- The no-behaviour-change claim for the new keys is observable only in the phase-branch CI / simulation (no local ROS, D-24) — flagged as coverage D6.

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-05*

## Self-Check: PASSED
