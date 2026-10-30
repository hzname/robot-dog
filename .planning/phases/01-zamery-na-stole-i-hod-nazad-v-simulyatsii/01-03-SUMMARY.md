---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 03
subsystem: testing
tags: [python, pytest, statistics, acceptance, json-schema, cli, push-threshold]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 01
    provides: "phase branch gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii with upstream; this plan itself runs only local pytest (D-24)"
provides:
  - "ros2_ws/src/dog_gazebo/dog_gazebo/acceptance_stats.py: CELLS, classify_run, run_record, summarize_cell, derive_push_threshold, push_threshold_for, build_result, render_summary, main; JSON schema 1; CLI 'python3 -m dog_gazebo.acceptance_stats threshold'"
  - "ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py: 68 pytest cases without ROS or Gazebo (rules D-01/D-05/D-06/D-07 at their boundaries, schema keys, CLI codes)"
  - "dog_gazebo packaging: setup.py extras_require={'test': ['pytest']} and package.xml <test_depend>python3-pytest</test_depend> (needed on Python 3.14)"
affects: [01-11, 01-13, 01-17, phase 01 verification, phase 2 (per-distro threshold data)]

# Actuals (#2632) - pairs with the plan's estimate
actuals:
  tokens: 12136
  tasks: 3
  commits: 3
plan_head_before: 9a5d3ef372cf5ae1af1f37e5438f57a252aaf824

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Statistics and scoring rules live in a pure-stdlib module (no rclpy/numpy) so pytest runs them on Python 3.12 and 3.14 without ROS (D-24)"
    - "The walk_check --trace JSON is translated into runs[] in exactly one place (run_record); every rule consumes runs[] records only"
    - "Schema 1 and the GITHUB_STEP_SUMMARY markdown are deterministic and sanitised; the CLI prints one threshold line and writes nothing"

key-files:
  created:
    - ros2_ws/src/dog_gazebo/dog_gazebo/acceptance_stats.py
    - ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py
  modified:
    - ros2_ws/src/dog_gazebo/setup.py
    - ros2_ws/src/dog_gazebo/package.xml
    - .planning/STATE.md
    - .planning/ROADMAP.md
    - .planning/REQUIREMENTS.md

key-decisions:
  - "acceptance_stats stays pure stdlib on top of terrain_sweep.never_stood; test_constants_match_walk_check pins MIN_BODY_HEIGHT 0.108 and FALL_TILT_DEG 60.0 to walk_check.py, and the acceptance constants (MIN_RATIO 0.40, MAX_DYAW5_DEG 10.0, MAX_TILT_DEG 20.0, MIN_REPEATS 5, floor 0.2) are never retuned (D-01, D-03, D-07)"
  - "run_record is the single translator of walk_check --trace JSON into runs[]; a fall in the last manoeuvre is caught by tilt_deg > 60.0 without the skipped-record flag, and missing data (dyaw5, backward ratio, tilt_deg, z) is always a failure reason, never a skipped check"
  - "D-05 thresholds are per distro, ideal model only, from cell flat_A_bwd10 with n >= 5; the threshold CLI exits 0 (derived) / 1 (not derivable) / 2 (unreadable file, schema not 1, unknown distro) and writes nothing to disk"

requirements-completed: [GAIT-01, GAIT-02]

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "acceptance rules: CELLS (D-03), classify_run ok/fell/no_stand/error, run_record (the walk_check translator), summarize_cell min/median over valid repeats with D-01/D-06/D-07 reasons, pass None below 5 repeats and for report cells"
    requirement: "GAIT-01"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py#test_tracer_five_repeats_to_threshold"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py#test_rule_a_jazzy_boundaries"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py#test_lyrical"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py#test_terrain_cells"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py#test_fewer_than_five, #test_missing_data_fails, #test_falls_no_stand_error_counted_separately"
        status: pass
    human_judgment: false
  - id: D2
    description: "D-05 push threshold: derive_push_threshold (clamp 0.2..0.4, 0.4375 -> 0.35 on the step boundary), push_threshold_for (per distro, ideal model, n >= 5), threshold CLI with exit codes 0/1/2 and no disk writes"
    requirement: "GAIT-02"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py#test_derive_push_threshold_examples, #test_push_threshold_for"
        status: pass
      - kind: integration
        ref: "PYTHONPATH=ros2_ws/src/dog_gazebo python3 -m dog_gazebo.acceptance_stats threshold r.json -> 'distro=jazzy min_ratio=0.520 push_threshold=0.40', exit 0"
        status: pass
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py#test_main_threshold_bad_input, #test_module_entry_point"
        status: pass
    human_judgment: false
  - id: D3
    description: "Schema 1 assembly and report: build_result (nine keys, CELLS order, scored-cells-only verdict) and render_summary (deterministic sanitised markdown for $GITHUB_STEP_SUMMARY)"
    requirement: "GAIT-01"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py#test_build_result_schema_keys, #test_verdict_aggregates_scored_cells, #test_render_summary_content, #test_render_summary_sanitizes"
        status: pass
    human_judgment: false
  - id: D4
    description: "Packaging: setup.py extras_require={'test': ['pytest']} and package.xml python3-pytest test_depend; console_scripts untouched (the acceptance entry arrives in 01-11)"
    requirement: "GAIT-02"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py#test_packaging_declares_pytest"
        status: pass
      - kind: other
        ref: "grep -F \"extras_require={'test': ['pytest']},\" setup.py; grep -F '<test_depend>python3-pytest</test_depend>' package.xml; ast/xml parse OK; git diff shows one added line in setup.py"
        status: pass
    human_judgment: false
  - id: D5
    description: "Backstop: Gazebo run-to-run spread and the Jazzy (Harmonic) vs Lyrical (Jetty) physics difference are not removed by the statistics; the real numbers only appear in the plan 01-13 CI acceptance run"
    verification: []
    human_judgment: true
    rationale: "Not locally provable without Gazebo (D-24); the module surfaces min/median and wall_s so the CI acceptance job (01-13) can show the actual spread"

# Metrics
duration: 6 min
completed: 2026-10-05
status: complete
---

# Phase 1 Plan 03: Acceptance statistics module Summary

**Pure-stdlib `acceptance_stats` with the D-01/D-06/D-07 rules, min and median over >= 5 repeats, JSON schema 1, a sanitised GITHUB_STEP_SUMMARY renderer, the D-05 push threshold and its threshold CLI — 68 pytest cases that run without ROS.**

## Performance

- **Duration:** 6 min
- **Started:** 2026-10-05T08:56:13Z
- **Completed:** 2026-10-05T09:02:26Z
- **Tasks:** 3
- **Files modified:** 4 (plus the .planning close-out files)

## Accomplishments
- `acceptance_stats.py` core: the D-03 `CELLS` table in order, `classify_run` (ok | fell | no_stand | error; a fall in the last manoeuvre is caught by `tilt_deg > 60.0` without the skipped-record flag; `never_stood` matches terrain_sweep), `run_record` (the only `walk_check --trace` -> `runs[]` translator; string/bool/NaN become None), `summarize_cell` (n/n_invalid/falls, minimum and median over valid repeats, D-01/D-06/D-07 reasons where missing data fails, `pass=None` below 5 valid repeats and for report cells).
- `derive_push_threshold` (D-05): `clamp(floor(0.8 * min), 0.2, 0.4)` with the `round(..., 9)` guard (0.4375 -> 0.35, 0.499 -> 0.35); NaN/inf/str/bool raise ValueError.
- Schema 1: `push_threshold_for` (per distro, ideal model only, n >= 5), `build_result` (nine keys, cells in CELLS order with `heading_hold`, verdict over scored cells only, `'no scored cells'`), `render_summary` (deterministic English markdown: percentages, `wall_s median`, PASS/FAIL/INSUFFICIENT/report, D-05 threshold line, reference-distro note; `git_sha` and free text sanitised).
- `main` + module entry point: `python3 -m dog_gazebo.acceptance_stats threshold RESULT.json [...]` prints `distro=<d> min_ratio=<m> push_threshold=<v>` per file; exit 0/1/2; stderr `cannot derive…` and the fall warning; only `json.load`, no eval/pickle; nothing written to disk.
- `setup.py`/`package.xml` declare pytest for Python 3.14; `entry_points` intentionally unchanged (plan 01-11 adds the `acceptance` line).

## Task Commits

Each task was committed atomically:

1. **Task 1: tracer + core (classification, min/median, rules, D-05 threshold)** - `c19aced` (feat)
2. **Task 2: schema 1 (push_threshold_for, build_result, verdict, render_summary)** - `a1de5ad` (feat)
3. **Task 3: threshold CLI, extras_require, test_depend** - `39fb28e` (feat)

**Plan metadata:** `docs(01-03): complete acceptance statistics module plan` (this commit: SUMMARY + STATE + ROADMAP + REQUIREMENTS)

## Files Created/Modified
- `ros2_ws/src/dog_gazebo/dog_gazebo/acceptance_stats.py` - statistics, rules, schema 1, markdown renderer and the threshold CLI; pure stdlib (json, math, re, statistics, never_stood)
- `ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py` - 68 cases: tracer JSON -> runs[] -> summary -> threshold; classification; run_record shapes; rule boundaries (0.40 / 10.0 / 20.0 / 0.108 / floor 0.2); CELLS order; threshold examples incl. 0.4375; schema keys; render sanitising; CLI codes; constants vs walk_check.py; ast purity
- `ros2_ws/src/dog_gazebo/setup.py` - `extras_require={'test': ['pytest']},` (one line)
- `ros2_ws/src/dog_gazebo/package.xml` - `<test_depend>python3-pytest</test_depend>`

## Decisions Made
- The acceptance constants are pinned, not tunable: `test_constants_match_walk_check` reads `walk_check.py` and fails if `MIN_BODY_HEIGHT` (0.108) or the `worst_tilt` threshold (60.0) drift; `test_module_is_pure` fails if rclpy/numpy/walk_check appear in the imports.
- `verdict.pass` reads only from scored cells; a report cell (`flat_B_bwd05`) and a `pass=None` scored cell can never fake a pass; `build_result('jazzy','ideal',5,'abc',{})` gives `failures == ['no scored cells']`.
- The `tilt_deg > 60.0` criterion covers the fall in the last manoeuvre; `run_record` is the single place where the `walk_check --trace` JSON becomes `runs[]` (both noted in the plan's assumption flags).
- The CLI prints only verified values and writes nothing; `schema` and `distro` are checked before use (T-01-03-03), the threshold never leaves 0.2..0.4 and never comes from the real model or fewer than 5 repeats (T-01-03-01).

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Inconsistent literal] The falls-warning test asserts the plan's exact message template**
- **Found during:** Task 3 (first run of `test_main_threshold_warns_on_falls`)
- **Issue:** the task's `<behavior>` says "… `'falls'` в stderr", but the task's exact warning template is `warning: <путь>: flat_A_bwd10 has %d fall(s), the threshold is derived from all valid repeats` — `'falls'` is not a substring of `'fall(s)'`, so the two could not both be satisfied literally.
- **Fix:** the module follows the exact template (the more specific instruction), and the test asserts `'warning' in stderr and 'has 1 fall(s)' in stderr` — the fall count and the template wording.
- **Files modified:** ros2_ws/src/dog_gazebo/test/test_acceptance_stats.py
- **Verification:** full suite 68 passed after the change.
- **Committed in:** 39fb28e

**Total deviations:** 1 auto-fixed (test literal aligned to the plan's message template; no implementation change).
**Impact on plan:** none on behaviour — the CLI warning matches the plan template; only the test assertion wording differs from the `<behavior>` shorthand.

## Issues Encountered
- The session `python3` is the Hermes toolchain interpreter (3.14, no pytest), so every suite and CLI check ran with `/usr/bin/python3` (Python 3.13.5, pytest 9.0.3), as in plan 01-02. No repository change; CI keeps `python -m pytest`.
- Simulations, `colcon` and `ros2` were not run (D-24): the plan's verification is local pytest plus the CLI. The acceptance job (plan 01-13) starts with this pytest on both distros.

## User Setup Required
None - no external service configuration required.

## Next Phase Readiness
- Plans 01-11 (acceptance CLI), 01-13 (CI job) and 01-17 (push threshold) can consume `run_record`, `build_result`, `render_summary` and the `threshold` command without re-deriving any rule; the module imports without ROS.
- GAIT-01 and GAIT-02 stay open in REQUIREMENTS.md until the sibling plans that also declare them finish (the `requirements.ready-ids` gate): GAIT-01 is also declared by 01-10, 01-11, 01-13, 01-16, 01-17 and GAIT-02 by 01-11, 01-13, 01-17.
- Backstop (D5): the module reports the minimum, median and `wall_s`; the real Gazebo spread and the Jazzy vs Lyrical difference are only observable in the plan 01-13 CI acceptance run.

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-05*

## Self-Check: PASSED
