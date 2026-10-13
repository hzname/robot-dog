---
phase: quick-261013-e4x
plan: 01
subsystem: servo calibration / knee limits
tags: [servos, knee, coupling, cad-limits, calibration, servo_pose]
status: complete
requirements: [CAL-01]
commits: 4
plan_head_before: 8baade8ee212866a40186ed44f21f9d1cab42971
actuals:
  tokens: 14600    # chars/4 over the text diff plan_head_before..HEAD (png not counted)
  tasks: 3
  commits: 4       # measured: git rev-list --count plan_head_before..HEAD
key-files:
  modified:
    - ros2_ws/src/dog_bringup/config/servos.yaml
    - ros2_ws/src/dog_bringup/config/robot.yaml
    - ros2_ws/src/dog_description/dog_description/urdf.py
    - ros2_ws/src/dog_description/test/test_urdf.py
    - tools/robot_setup/robot_setup.py
    - tools/robot_setup/test/test_yaml_contract.py
    - tools/robot_setup/test/test_robot_setup.py
    - ros2_ws/src/dog_hardware/test/test_servo_driver.cpp
    - tools/servo_pose/servo_pose_server.py
    - tools/servo_pose/servo_pose_apply.py
    - tools/servo_pose/test_pose_api.py
    - docs/CALIBRATION.md
    - docs/DEPLOYMENT.md
    - docs/REVIEW.md
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/deferred-items.md
  created:
    - docs/img/thigh_knee_limits.png
---

# Quick 261013-e4x: knee limits from the thigh x knee CAD map

The knee is coupled 1:1 to the thigh on all four legs with a servo window on s = thigh + knee of -80..+15 deg and a
knee limit of -140..-40 deg, in `servos.yaml`, `robot.yaml` and the `urdf.py` default, pinned by contract and driver
tests. The numbers are CAD results (Fusion 360, SPT5435LV stand-in), not robot measurements.

## What was done

- **servos.yaml**: four `*_calf_joint` rows with `coupled_to: <leg>_thigh_joint`, `coupling: 1.0`, `servo_arm_mm: 0.0`,
  `offset_deg -32.5`, `range_deg 95.0`, `min_deg -140.0`, `max_deg -40.0`; pulses shifted per direction so the centre
  pulse 1370 us stays the mount pose s = -45: left 1039.5/1936.5 us, right 803.5/1700.5 us. Header comments explain the
  coupling and the window.
- **robot.yaml / urdf.py**: `calf_limits_deg` [-140.0, -40.0] (comment with the origin of the number); `DEFAULT_DESCRIPTION`
  synchronised. `PINNED_DESC` in the golden-hash tests is untouched (D-15).
- **robot_setup.py**: the "coupled knee needs servo_arm_mm > 0" warning is replaced with an info message.
- **Tests**: 4 contract tests in `test_yaml_contract.py` (coupling, limits equal robot.yaml, window inside CAD stops and
  covering the gait envelope, mount pose = 1370 us), 2 `robot_setup` tests, 1 `test_urdf` default-agreement test,
  7 `CoupledKnee` gtests (direction +1 and -1, window edges, q1 100 / q2 -60 clamped, knee limit before the sum,
  12-pose CAD envelope, ServoDriver clamp count).
- **servo_pose**: page and `servo_pose_apply.py` skip rows with `coupled_to` and report them as skipped;
  `FALLBACK_BOUNDS['calf']` synced; mock test updated to the new apply contract.
- **Docs**: chart `docs/img/thigh_knee_limits.png` (copied as is), new CALIBRATION.md section with mechanism, figure,
  table, written numbers, calibration recipe and conflicts; DEPLOYMENT.md D-20 correction; REVIEW.md rows 29-32;
  deferred-items.md section.

## Commits

| Task | Commit | Message |
|------|--------|---------|
| 1 | ed3c692 | feat(quick-261013-e4x): coupled knee limits from the CAD map |
| 2 | e32cf1d | test(quick-261013-e4x): coupled knee driver tests, servo_pose skips coupled rows |
| 3 | 246c7b5 | docs(quick-261013-e4x): coupled knee region, calibration recipe, open conflicts |
| follow-up | fb852fc | test(quick-261013-e4x): say how to update the mount pose constant after calibration |

## Verification (what was actually run locally)

| Check | Result |
|-------|--------|
| `python3 -m pytest -q tools/robot_setup/test` | 60 passed (42 existing + 18 new incl. parametrised) |
| `PYTHONPATH=ros2_ws/src/dog_description python3 -m pytest -q ros2_ws/src/dog_description/test` | 22 passed (21 + 1) |
| `python3 tools/robot_setup/robot_setup.py --check` | exit 0, no `внимание` line |
| `g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror` + gtest on `test_servo_driver.cpp`, `servo_driver.cpp`, `servo_bus.cpp` | built without warnings, 24 tests passed (17 existing + 7 new) |
| `servo_pose_apply.py` dry run on the shipped servos.yaml with a trim on channel 2 | no change arrow, four calf rows listed as skipped |
| `tools/servo_pose/run_mock_test.py` | exactly one FAIL: the known pre-existing `/img/assembly_pose.svg отдаётся как svg`; the new apply checks are OK; temporary files removed |

Built locally with g++/gtest. **Not run:** `colcon build` / `colcon test` for Jazzy and Lyrical (no ROS here), the
Gazebo CI checks (`walk_check`, `terrain_sweep`, ...) after the URDF limit change (no push done), everything on the
real robot.

## Envelope observation (report only, no controller change)

A throwaway program outside the repo linked the ROS-free `dog_control` core and drove `LocomotionController`
(parameters of `pinnedParams()`, which equal the shipped `robot.yaml`; repeated with calf 111 and 116.6 mm).
Min/max of q2 and s = thigh + calf in degrees:

| Scenario (105/105) | q2 | s | In region (q2 -140..-40, s -80..+15) |
|---|---|---|---|
| stand-up, stand, lie (0.08 m) | -135.2 .. -88.8 | -67.6 .. -44.4 | yes (q2 margin 4.8 deg at lying) |
| min height 0.10 / max height 0.18 | -135.2 .. -88.8 / -135.2 .. -62.0 | -67.6 .. -44.4 / -67.6 .. -31.0 | yes |
| max roll + pitch | -135.2 .. -52.0 | -75.7 .. -38.7 | yes |
| walk +-vx, +-vy, wz, vx+vy+wz (max speeds) | -135.2 .. -80.5 | -67.6 .. -28.5 | yes |
| walk at min height | -135.2 .. -88.8 | -78.3 .. -36.3 | yes, s margin only 1.7 deg |
| walk at max height | -135.2 .. -59.6 | -67.6 .. -19.4 | yes |
| survey | -135.2 .. -73.1 | -67.6 .. -25.2 | yes |
| crawl forward on flat ground (vx 0.05..0.15) | -96.0 .. -39.9 | -67.7 .. +11.8 | **q2 0.1 deg over -40** (111: -41.4, 116.6: -48.8, inside) |
| crawl vy 0.08 / wz 0.6 / back -vx | -113.6 .. -0.4 / -115.7 .. -0.4 / -96.0 .. -12.5 | -71.5 .. +1.1 / -89.1 .. +30.9 / -69.5 .. -6.0 | **outside** |
| greet | -135.2 .. -50.4 | -121.4 .. -16.9 (front legs; rear legs -90.0) | **s outside** (-87 hard stop, -80 window) |

Meaning: on the shipped config stand, walk, lie, survey fit the window; `crawl` (without a terrain profile, flat
ground only) and `greet` leave it, so the Gazebo URDF limit -140..-40 can saturate in CI sim checks that use them
(`terrain_sweep` steps/stairs use the crawl). Recorded in REVIEW rows 29 and 31; no controller number was changed.

## Deviations from Plan

**1. [Rule 2 - Missing critical functionality] Mount-pose test constant explained**
- **Found during:** Task 3 (writing the calibration recipe).
- **Issue:** the contract test pins "1370 us = s -45". After a measured s0 != -45 and either recipe (shift or shrink) it
  fails, with no hint what to change; a red CI after a correct calibration would push someone to relax the window check.
- **Fix:** a comment on `MOUNT_S_DEG` and a hint in the assertion message ("put the measured s0 here"); the recipe in
  CALIBRATION.md states it (with s0 in the constant the test holds for both recipes).
- **Files modified:** `tools/robot_setup/test/test_yaml_contract.py`
- **Commit:** fb852fc (separate commit, the file belongs to Task 1)

Otherwise the plan was executed as written.

## Open items for the owner

- **D-20 contradiction:** 01-CONTEXT.md D-20 says the knee rod rests on the thigh and `coupled_to` is not needed; the
  CAD shows the knee servo on the body, so `coupled_to` is implemented as the task input required. 01-CONTEXT.md was not
  edited; the correction is in DEPLOYMENT.md and deferred-items.md and waits for the owner's confirmation on the robot.
- **Measure s0 at 1370 us** on the first calibration (the CAD pose gives about -47, the shipped config assumes -45);
  recipe in CALIBRATION.md.
- `greet` and the crawl at full vy/wz leave the region (REVIEW 29, 31); `thigh_limits_deg` min -45 is below the CAD
  stop q1 <= -35 (REVIEW 32, unchanged by instruction); the servo speed of a coupled joint is counted on the joint
  angle only (REVIEW 26, 32).
- Supports body version (`lie_height` about 0.100 only with supports) is recorded in deferred-items.md; geometry stays
  plan 01-15.

## Known Stubs

None.

## Threat Flags

None: no new endpoints, auth paths or trust boundaries beyond the plan's threat model.

## Self-Check: PASSED

- Files exist: `docs/img/thigh_knee_limits.png` (identical to the task directory copy, `cmp`), `docs/CALIBRATION.md`
  section, REVIEW rows 29-32, deferred-items.md section.
- Commits exist: ed3c692, e32cf1d, 246c7b5, fb852fc on `gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii`.
- Repo clean of check leftovers: no `servos_apply_test.yaml*`, `servo_pose_trim.json`; untracked owner files untouched.
