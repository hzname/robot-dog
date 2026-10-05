---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 11
subsystem: testing
tags: [python, pytest, acceptance, cli, argparse, gazebo, ci, statistics]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 03
    provides: "acceptance_stats: CELLS, run_record, build_result, render_summary, schema 1, MIN_REPEATS (the only scoring rules)"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 10
    provides: "servo_model:=real launch argument that cell_launch_args appends"
provides:
  - "ros2_ws/src/dog_gazebo/dog_gazebo/walk_check_args.py: FLAT_MANEUVERS, SLOPE_MANEUVERS, DEFAULT_BACKWARD_SPEED, MAX_BACKWARD_SPEED, parse_maneuvers, parse_backward_speed, build_parser, parse_args, maneuver_plan, lie_wanted"
  - "walk_check: --maneuvers, --backward-speed, dyaw5_deg (drift over the commanded seconds, before the 1.5 s coast); the default 10-check routine and the push-CI call are unchanged"
  - "acceptance CLI: cell_launch_args, cell_walk_check_args, plan_runs, domain_for, seed_for, parse_args, execute, _flush, main; fresh run_level per repeat, one relaunch on never_stood, no_stand replaced up to 2*n, error not replaced; schema 1 JSON + summary after every repeat; console script 'acceptance'"
affects: [01-13 (CI acceptance job), 01-17 (push-CI threshold), phase 01 verification]

# Actuals (#2632) - pairs with the plan's estimate (70000 tokens, 3 tasks)
actuals:
  tokens: 14886
  tasks: 3
  commits: 3
plan_head_before: 3127cf93477b8078dd6d549cdd89eb9b119c80ca

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "walk_check keeps its ROS side; the pure argument/table layer moved to walk_check_args (argparse + math only) so pytest runs it on 3.12 and 3.14 without rclpy (D-01, D-02)"
    - "acceptance.py orchestrates only: one fresh terrain_sweep.run_level per repeat, one attempt at a time (D-24); every attempt is translated by acceptance_stats.run_record and flushed as schema 1 JSON + markdown after the repeat"
    - "Cells carry mode and speed: mode B adds heading_hold:=false slope_compensation:=false, walk_check runs with --min-ratio 0 --backward-ratio 0 and no threshold exists in acceptance.py (D-17)"

key-files:
  created:
    - ros2_ws/src/dog_gazebo/dog_gazebo/walk_check_args.py
    - ros2_ws/src/dog_gazebo/test/test_walk_check_args.py
    - ros2_ws/src/dog_gazebo/dog_gazebo/acceptance.py
    - ros2_ws/src/dog_gazebo/test/test_acceptance_cli.py
  modified:
    - ros2_ws/src/dog_gazebo/dog_gazebo/walk_check.py
    - ros2_ws/src/dog_gazebo/setup.py
    - .planning/STATE.md
    - .planning/ROADMAP.md
    - .planning/REQUIREMENTS.md

key-decisions:
  - "Plan 01-11: walk_check_args is pure (argparse/math) and maneuver_plan keeps the old run() table bit for bit, slope's -0.14 * T included; dyaw5_deg is recorded right after the commanded spin, before the 1.5 s coast, so the 5 s drift criterion is measurable (D-01, D-02, GAIT-01)"
  - "Plan 01-11: acceptance.py only orchestrates - fresh run_level per repeat, one relaunch on never_stood (.retry.sim.log), no_stand replaced up to 2*n attempts, error not replaced (it takes a repeat slot, so a systematic failure does not double the job time); records go through run_record only (D-03, D-24)"
  - "Plan 01-11: mode B cells run with heading_hold:=false slope_compensation:=false and every cell passes --min-ratio 0 --backward-ratio 0; no threshold lives in acceptance.py (grep gate + test) and the verdict is acceptance_stats' (D-06, D-07, D-17)"
  - "Plan 01-11: --dry-run prints every run_level launch (domain=80+i%10, the launch and walk_check tails); the tests call the CLI only with --dry-run and a fake runner, the real run is the plan 01-13 CI job (D-04, D-24)"

patterns-established:
  - "Pure-layer split: ROS-free argument/table/planning modules importable by pytest; the CLI shell wires them to run_level"
  - "Incremental artifacts: schema 1 JSON and the summary are written atomically (tmp + os.replace) after every repeat, so an interrupted job keeps what it measured"

requirements-completed: [GAIT-01, GAIT-02, GAIT-06]

# Coverage metadata (#1602) - one entry per shipped deliverable
coverage:
  - id: D1
    description: "walk_check --maneuvers/--backward-speed and dyaw5_deg: subset in routine order, lie skipped for a subset, slope flags ignored with one note line, drift recorded before the coast"
    requirement: "GAIT-01"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_walk_check_args.py#test_tracer_backward_005_dyaw5"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_walk_check_args.py#test_default_plan_is_golden, #test_plan_subset_and_speed, #test_lie_wanted, #test_run_full_routine_is_ten_checks, #test_run_subset_skips_lie, #test_run_slope_flags_note, #test_maneuver_skip_record_has_no_dyaw5"
        status: pass
    human_judgment: false
  - id: D2
    description: "acceptance CLI: cells/plan/--dry-run plus the repeat runner - fresh run_level per repeat, relaunch on never_stood, no_stand replacement bound, error not replaced, incremental schema 1 JSON + summary, exit codes 0/1/2"
    requirement: "GAIT-02"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_acceptance_cli.py#test_flat_a_five_repeats_to_verdict, #test_strict_reads_verdict, #test_runner_arguments, #test_incremental_flush, #test_relaunch_once, #test_no_stand_replaced_up_to_2n, #test_error_is_not_replaced, #test_fell_counts_as_valid, #test_local_warning, #test_invalid_distro_never_starts"
        status: pass
      - kind: integration
        ref: "PYTHONPATH=ros2_ws/src/dog_gazebo python3 -m dog_gazebo.acceptance --dry-run --distro lyrical --servo-model real --cells all --repeats 5 -> 25 DRY-RUN lines"
        status: pass
    human_judgment: false
  - id: D3
    description: "Packaging: console_scripts 'acceptance = dog_gazebo.acceptance:main' after terrain_sweep; python3 -m dog_gazebo.acceptance works"
    requirement: "GAIT-02"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_acceptance_cli.py#test_packaging_declares_acceptance_entry, #test_module_entry_point"
        status: pass
    human_judgment: false
  - id: D4
    description: "Backstop: the Gazebo run-to-run spread and the Jazzy/Lyrical physics difference are not removed here; the CLI reports wall_s and min/median so the plan 01-13 CI run can show the real numbers"
    verification: []
    human_judgment: true
    rationale: "Not locally provable without Gazebo (D-24); the first real repeats=1 run (plan 01-13) gives the true repeat time and spread"

# Metrics
duration: 6 min
completed: 2026-10-05
status: complete
---

# Phase 1 Plan 11: Acceptance CLI and walk_check maneuver flags Summary

**walk_check_args (pure) with `--maneuvers`/`--backward-speed` and `dyaw5_deg` before the coast, and the acceptance CLI that repeats the D-03 cells with a fresh simulation each time, one relaunch on never_stood, no_stand replacements up to 2*n, and schema 1 JSON plus the summary flushed after every repeat.**

## Performance

- **Duration:** 6 min
- **Started:** 2026-10-05T20:16:07Z
- **Completed:** 2026-10-05T20:22:13Z
- **Tasks:** 3
- **Files modified:** 6 (4 created, 2 modified; plus the .planning close-out files)

## Accomplishments
- `walk_check_args.py`: the pure argument and table layer - `parse_maneuvers` (names from `FLAT_MANEUVERS`, routine order, unknown/duplicate/empty rejected), `parse_backward_speed` (finite, 0 < v <= 0.5, the negative case says "give the speed magnitude"), `maneuver_plan` (the old `run()` table bit for bit; slope ignores both flags and keeps the literal `-0.14 * T`), `lie_wanted` (slope and the full routine only).
- `walk_check.py`: `run()` loops over `maneuver_plan`, one `note:` line when the flags are given on slope, the lie block under `lie_wanted`; `dyaw5 = self.yaw_unwrapped - yaw0` sits between the commanded spin and the 1.5 s coast, `dyaw5_deg` is added next to `dyaw_deg`; `MIN_BODY_HEIGHT = 0.108` and `worst_tilt > 60.0` untouched, the default routine still prints `10/10 passed`.
- `acceptance.py`: `--dry-run` prints every `run_level` launch (cell, repeat, terrain, seed, domain `80 + i % 10`, launch args, walk_check tail) and the planned count; `execute()` runs one fresh simulation per repeat, relaunches once on `never_stood` with `.retry.sim.log`, replaces `no_stand` up to 2*n attempts, treats `error` as a taken slot (no time doubling), prints one line per attempt and flushes the schema 1 JSON + `render_summary` markdown after every repeat via tmp + `os.replace`; `main()` warns outside GitHub Actions, prints `FAIL <reason>` / `PASS verdict` and returns 0 (artifacts), 1 (a cell with n == 0, or `--strict` without a pass) or 2 (argparse).
- `setup.py`: the `acceptance` console script after `terrain_sweep`; `python3 -m dog_gazebo.acceptance --dry-run` works as the module entry point.
- 57 new pytest cases (23 + 34), all without ROS or Gazebo: the whole `ros2_ws/src/dog_gazebo/test` directory is 125 passed together with the 01-03 statistics suite.

## Task Commits

Each task was committed atomically:

1. **Task 1: tracer - walk_check_args, --maneuvers/--backward-speed, dyaw5_deg** - `29f1778` (feat)
2. **Task 2: acceptance CLI pure layer - cells, plan, --dry-run** - `342a5d3` (feat)
3. **Task 3: acceptance repeat runner, incremental JSON/summary, console script** - `fcb8685` (feat)

**Plan metadata:** `docs(01-11): complete acceptance CLI plan` (this commit: SUMMARY + STATE + ROADMAP + REQUIREMENTS)

## Files Created/Modified
- `ros2_ws/src/dog_gazebo/dog_gazebo/walk_check_args.py` - pure argparse/math module: constants, `parse_maneuvers`, `parse_backward_speed`, `build_parser`, `parse_args`, `maneuver_plan`, `lie_wanted`
- `ros2_ws/src/dog_gazebo/test/test_walk_check_args.py` - 23 cases on stub rclpy: tracer `dyaw5_deg` 8.0 vs `dyaw_deg` 11.0, parse surface, golden flat/slope tables, subset/speed, `lie_wanted`, `run()` through a fake simulation (10/10, 2/2, slope note), skipped record has no `dyaw5_deg`, module purity
- `ros2_ws/src/dog_gazebo/dog_gazebo/acceptance.py` - CLI: constants, cell argument builders, plan/domain/seed, `parse_args` with choices and post-checks, `_flush`, `execute`, `main`
- `ros2_ws/src/dog_gazebo/test/test_acceptance_cli.py` - 34 cases: parse defaults/rejects, builder lists and the walk_check_args round trip, 15-record plan, domains, dry-run output, verdict flow, `--strict`, runner arguments, incremental flush, relaunch, replacement bounds, local warning, purity, packaging, entry point
- `ros2_ws/src/dog_gazebo/dog_gazebo/walk_check.py` - imports the new module, new constructor parameters, `dyaw5_deg`, table loop, slope note, `main()` via `parse_args`; constants untouched
- `ros2_ws/src/dog_gazebo/setup.py` - one console_scripts line

## Decisions Made
- The argument/table layer is pure so pytest runs it without rclpy; `maneuver_plan` reproduces the old `run()` table exactly, so the default routine, `terrain_sweep` and the push-CI command are bit-compatible.
- `dyaw5_deg` is the drift over the commanded seconds only (the GAIT-01 "5 s" criterion); `dyaw_deg` keeps its old meaning (including the coast).
- The acceptance CLI never scores: `--min-ratio 0 --backward-ratio 0` on walk_check and no threshold constant in `acceptance.py`; the verdict is `acceptance_stats`' (D-17).
- A `no_stand` attempt is replaced (up to 2*n attempts per cell); an `error` is not replaced and takes one of the n repeat slots - a systematic failure does not double the job time.
- Tests touch the CLI only via `--dry-run` and a fake runner; simulations, `colcon` and `ros2` were never run locally (D-24) - the real run is plan 01-13's CI job.

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered
- The session `python3` is the Hermes toolchain interpreter (3.14, no pytest/numpy), so every suite and CLI check ran with `/usr/bin/python3` (Python 3.13.5, pytest 9.0.3), as in plans 01-02 and 01-03. No repository change; CI keeps `python -m pytest`.

## User Setup Required
None - no external service configuration required.

## Next Phase Readiness
- Plan 01-13 (CI acceptance job) can call `ros2 run dog_gazebo acceptance --repeats N --cells NAME --servo-model M --distro D --domain 80 [--strict]`; defaults write `acceptance_<distro>_<servo_model>.json` and `..._summary.md` into the working directory (ros2_ws).
- Plan 01-17 consumes `acceptance_stats threshold` on the result file; `push_threshold` is already inside every flushed JSON.
- GAIT-01, GAIT-02 and GAIT-06 stay open in REQUIREMENTS.md until the sibling plans that also declare them finish (the `requirements.ready-ids` gate).
- Backstop: the real Gazebo spread, the Jazzy vs Lyrical difference and the true repeat `wall_s` are only observable in the plan 01-13 CI run (D-24).

## Self-Check: PASSED
