---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 16
subsystem: gait
tags: [acceptance, backward-walk, gait-shape, ci, d-01, d-07, gait-01, gait-06, not-needed]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 13
    provides: "ci.yml workflow_dispatch inputs and the acceptance jobs, the baseline of 2026-10-06 (01-BASELINE.md) to compare against"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 11
    provides: "the acceptance CLI and its cells (flat_A_bwd10, flat_B_bwd10, flat_B_bwd05, waves10_A_bwd10, rocks10_A_bwd10)"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 03
    provides: "acceptance_stats: per-cell summary (n, ratio_min/median, falls, max|dyaw5|, pass, reasons) and verdict"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 15
    provides: "the reduced close: no measured numbers and no auto_period, so the period stays 0.55 from robot.yaml"
provides:
  - "A fresh acceptance run on the current servo config (RUN_ID 40324372510, 83ce7c9, cells=all repeats=5): four JSONs and their summary in acceptance/post-numbers/"
  - "Decision gait_changes: not_needed, failed_criteria: none — the backward transfer shape stays as it is"
  - "A test-only fix for the red build + test: dog_bringup no longer hard-codes the v1 thigh offset"
affects: [01-17 (final acceptance and the D-05 threshold edit), phase 02]

# Actuals (#2632)
actuals:
  tokens: 1500
  tasks: 1
  commits: 3

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "The acceptance decision is computed from the JSON by a script that reproduces the plan's criteria, never read off the tables by eye"
    - "gh run download needs --pattern 'acceptance-*' or the per-artifact name: the unfiltered download aborts on the dockerbuild artifact"

key-files:
  created:
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-POST-NUMBERS.md
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/acceptance/post-numbers/summary.md
  modified:
    - ros2_ws/src/dog_bringup/test/test_mock_bringup.py

key-decisions:
  - "gait_changes: not_needed — Jazzy real passes both gates with margin (53.5 % and 51.4 % against 40 %, yaw 6.2 deg against 10, no falls), so no transfer-shape candidate K1-K3 was tried and the gait code is unchanged"
  - "The plan was adapted, not executed as written: period_s_before comes from robot.yaml (0.55) because 01-15 closed without auto_period and the gtest ShippedDefaultsStartWithAutoPeriod does not exist; the criteria of D-01/D-07 are unchanged"
  - "The failing dog_bringup test was fixed as its own commit (test-only): it hard-coded the v1 offset 45 deg while servos.yaml now carries the measured 56 deg, so the same pose gave 1266.1 us instead of 1370"

patterns-established:
  - "A plan whose premise disappeared (measured numbers) can be re-based on the closest honest evidence (a fresh run of the same acceptance on the current config) instead of being skipped"

requirements-completed: [GAIT-01, GAIT-06]

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "Acceptance data on the current config: four schema-1 JSONs with repeats=5 and their summary, plus the computed decision in 01-POST-NUMBERS.md"
    requirement: "GAIT-01"
    verification:
      - kind: e2e
        ref: "GitHub Actions run 40324372510: acceptance (jazzy,ideal), (lyrical,ideal), (lyrical,real), (jazzy,real) all success; repeats=5, cells=all"
        status: pass
      - kind: other
        ref: "recompute script: DECISION=not_needed failed=none, matches the gait_changes/failed_criteria lines of 01-POST-NUMBERS.md"
        status: pass
    human_judgment: false
  - id: D2
    description: "Forward walking, the other maneuvers, the period and the servo peak unchanged (no gait code touched)"
    requirement: "GAIT-06"
    verification:
      - kind: e2e
        ref: "run 40324372510: gazebo walk check (jazzy) and (lyrical) success; BUILD + TEST on e112250 success on both distros"
        status: pass
      - kind: unit
        ref: "local: tools/local_gtest/run.sh dog_control {gait,kinematics,crawl,greet,locomotion,servo_limits} -> 62 tests passed"
        status: pass
    human_judgment: false
  - id: D3
    description: "A new backward transfer shape (K1-K3) with the candidate ladder"
    verification: []
    human_judgment: true
    rationale: "Not needed: the decision is not_needed, so the plan's tasks 2-3 do nothing by design. A human still owns the reading of the numbers — the gate was passed with 13 p.p. of margin on one 5-repeat series, and the run-to-run Gazebo spread is not characterised."

# Metrics
duration: 1h 10m
completed: 2026-10-26
status: complete
---

# Phase 1 Plan 16: backward transfer shape — not needed on the current config

**A fresh acceptance run on the current servo config (branch 83ce7c9, cells=all repeats=5) passes every gate with margin — Jazzy real 53.5 % on flat_A_bwd10 (median 62.2 %) against 40 %, yaw 6.2° against 10°, no falls anywhere — so `gait_changes: not_needed`, the gait code is untouched, and the only code change of this plan is a test-only fix for the red `build + test`.**

## Performance

- **Duration:** 1h 10m (includes the 25-minute acceptance run and the CI investigation)
- **Started:** 2026-10-26T12:40Z
- **Completed:** 2026-10-26T13:55Z
- **Tasks:** 1 of 3 executed (tracer); tasks 2-3 are the conditional gait work and do nothing at `not_needed`
- **Files modified:** 8 (2 planning documents, 4 JSONs + summary, 1 test)

## Accomplishments

- **Run:** `ci_dispatch.sh --ref … -f acceptance=true -f repeats=5 -f cells=all -f strict=false --wait-job 'acceptance ('` — RUN_ID 40324372510, all four matrix jobs `success`; the four JSONs are schema 1, `repeats` 5, `git_sha` 83ce7c9.
- **Numbers (Jazzy real, the scoring element):** `flat_A_bwd10` min 53.5 %, median 62.2 %, max |dyaw5| 6.2°; `flat_B_bwd10` 51.4 % / 54.9 %; waves 41.4 %; rocks 52.1 %; falls 0 in every cell; `flat_B_bwd05` 28.5 % (report only). The other three elements pass too: Jazzy ideal 65.2 %, Lyrical ideal 52.4 %, Lyrical real 45.2 %.
- **Decision:** `gait_changes: not_needed`, `failed_criteria: none`, verified by a script that recomputes the plan's criteria from the JSON — the tables in the document are not the source of truth.
- **Comparison with the baseline of 2026-10-06:** at n=5 Jazzy real moved from 50.4 % to 53.5 %, Lyrical real from 49.1 % to 45.2 % (a dip, still above the 0.20 floor); the servo trims and knee windows of 2026-10-26 are the only config difference.
- **Red CI found and fixed:** `build + test` had been failing since `61878f1` because `test_mock_bringup.py` hard-coded the v1 thigh offset (45°) and the measured mount trim moved it to 56°, so the same pose gave 1266.1 µs instead of 1370. The test now reads `offset_deg` from `cal_info`; on `e112250` both `build + test` jobs are green.

## Task Commits

1. **Task 1 (tracer): acceptance data and the decision** - `64c5650` (docs)
2. **Pre-existing red CI: the bringup test follows the config** - `e112250` (fix, after the run; test-only)
3. **Plan close (this SUMMARY, STATE, ROADMAP, .continue-here)** - this commit (docs)

## Files Created/Modified

- `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-POST-NUMBERS.md` - the run conditions, four per-element tables with the baseline column, the decision lines, the diagnosis and the deviations
- `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/acceptance/post-numbers/` - `acceptance_{jazzy,lyrical}_{ideal,real}.json` and the concatenated `summary.md`
- `ros2_ws/src/dog_bringup/test/test_mock_bringup.py` - the mount pose is taken from `cal_info`, not from a constant

## Decisions Made

- **No gait change.** Both gates pass with margin on the scoring element; a new transfer shape (K1-K3) would be guessing at an unmeasured problem, and D-17 says shape changes are allowed, not required.
- **The plan is re-based, not skipped.** Its premise (measured numbers) disappeared with the reduced 01-15, so the tracer runs the same acceptance on the current config and `period_s_before` is read from `robot.yaml`; the criteria are untouched.
- **The bringup test is fixed where it lives.** The defect came from a completed plan's change (mount trims), so it is a minimal fix plus a comment, documented here and in the commit.

## Deviations from Plan

- **Task 1's precondition** (`## Применение (01-15)`) does not hold; replaced by the owner's approved adaptation (2026-10-26): `period_s_before` from `robot.yaml`, no auto-period gtest.
- **`period_s_before` = 0.55** instead of a value printed by `Locomotion.ShippedDefaultsStartWithAutoPeriod` (the test does not exist because 01-15's task 3 was not executed).
- **`code_sha_before` = 83ce7c9, and the branch moved to e112250 afterwards** with the test-only fix; the plan's "no code after code_sha_before" check is therefore documented as a deviation rather than satisfied.
- **Artifact download** needed `--pattern 'acceptance-*'` plus two per-artifact downloads; the unfiltered `gh run download` aborts on the dockerbuild artifact (known from 01-13).

## Issues Encountered

- **`build + test` had been red on the branch since 61878f1**, i.e. before this plan started: the knee/offset recalibration changed `servos.yaml` but the bringup test kept the v1 constant. It surfaced only because the acceptance run's sibling job went red; `gh run view --log` was empty, so the cause was found in the `test-logs-jazzy` artifact (`dog_bringup` 0/1 tests, assertion 1266.1 != 1370).
- **CI job logs via `gh run view --job … --log` returned nothing** for that run; the artifacts (`latest_test/events.log`) carried the answer.

## User Setup Required

None.

## Next Phase Readiness

- **01-17 is next and is the last code edit of the phase:** final acceptance plus the `--backward-ratio` threshold edit (D-05). It now has a fresh "after" dataset on the current config: `flat_A_bwd10` minima 53.5 % (Jazzy real), 65.2 % (Jazzy ideal), 52.4 % (Lyrical ideal), 45.2 % (Lyrical real) at n=5, plus the 2026-10-10 n=10 series in 01-BASELINE.md (42.1 % / 37.1 %).
- **No gait code changed**, so 01-17 starts from the same walk the baseline measured; the only code delta on the branch is the bringup test.
- **Open for the owner:** the D-05 thresholds themselves (jazzy 0.35 / lyrical 0.40 candidates from 01-BASELINE.md) are 01-17's decision.

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-26*
