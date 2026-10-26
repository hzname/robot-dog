---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 17
subsystem: gait
tags: [acceptance, backward-walk, thresholds, checkpoint, d-01, d-05, gait-01, gait-02, gait-06, gait-10]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 16
    provides: "no gait changes were needed, so the walk under test is the one the baseline measured; the acceptance CLI and the five cells"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 13
    provides: "ci.yml workflow_dispatch inputs, the acceptance jobs, the push-CI walk check at --backward-ratio 0.2, 01-BASELINE.md"
provides:
  - "acceptance/final/: eight JSONs (four repeats=5 over all cells, four repeats=10 over flat_A_bwd10) at 0917551 plus summary.md"
  - "01-ACCEPTANCE-REPORT.md: per-cell min and median for four elements, the verdicts, 100 run times, the derived thresholds (jazzy 0.30, lyrical 0.40) and the assumptions"
  - "The phase's checkpoint: the scoring element fails one criterion by 0.8 deg, so ci.yml and docs/TERRAIN.md are deliberately untouched"
affects: [phase 01 close, the owner's decision on the yaw criterion, phase 02]

# Actuals (#2632)
actuals:
  tokens: 3200
  tasks: 2
  commits: 3

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "The report is generated from the eight JSONs by a script (ros2_ws/build/_ci/01-17-report.py), so no number is typed by hand"
    - "Two runs must not overlap: run B was only dispatched once run A's RUN_ID was in the log, and the parallel runs are a candidate explanation for the single yaw outlier"

key-files:
  created:
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-ACCEPTANCE-REPORT.md
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/acceptance/final/summary.md
  modified: []

key-decisions:
  - "Stop at the checkpoint instead of re-running: the owner closed the question with 'не. закрывай' (2026-10-26) after seeing that the only failure is one repeat at 10.8 deg against a 3.3-6.1 deg spread and a 6.8 deg maximum over ten more repeats"
  - "ci.yml keeps --backward-ratio 0.2 and docs/TERRAIN.md stays unwritten: the plan forbids changing the threshold and the docs when the scoring element fails (D-17, D-01)"
  - "The derived thresholds are still recorded in the report (jazzy 0.30, lyrical 0.40) so the owner can decide without re-running anything"

patterns-established:
  - "A max-based gate over five repeats is fragile by construction: 20 samples of flat_A_bwd10 today gave one 10.8 deg outlier against a 3.3-6.8 deg body"

requirements-completed: [GAIT-06]

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "Final acceptance data: eight schema-1 JSONs at the pushed SHA, five or ten repeats per cell, plus summary.md"
    requirement: "GAIT-01"
    verification:
      - kind: e2e
        ref: "runs 40328414499 (repeats=5, cells=all, strict=true) and 40328467878 (repeats=10, cells=flat_A_bwd10); data check OK final data"
        status: pass
    human_judgment: false
  - id: D2
    description: "The report reproduces every number from the JSONs and derives the thresholds by the D-05 formula"
    verification:
      - kind: other
        ref: "the plan's task-2 check: OK report FAIL 0.30 0.40"
        status: pass
    human_judgment: false
  - id: D3
    description: "The Jazzy real scoring gate: flat_A_bwd10 min >= 40 %, heading <= 10 deg in mode A, flat_B_bwd10 min >= 40 %, no falls"
    requirement: "GAIT-01"
    verification:
      - kind: e2e
        ref: "acceptance_jazzy_real.json: verdict.pass false, failures ['flat_A_bwd10: max|dyaw5|=10.8 deg > 10.0']"
        status: fail
      - kind: e2e
        ref: "spread10_jazzy_real.json: max|dyaw5| 6.8 deg over ten repeats of the same cell"
        status: pass
    human_judgment: true
  - id: D4
    description: "The push-CI threshold applied per distro and verified green on both distros"
    requirement: "GAIT-02"
    verification: []
    human_judgment: true
    rationale: "Not executed: tasks 3-4 belong to the pass branch and the scoring element failed."

# Metrics
duration: 1h 45m
completed: 2026-10-26
status: blocked
---

# Phase 1 Plan 17: the acceptance gate fails one criterion by 0.8 degrees

**Everything except the heading gate passes with margin — Jazzy `real` gives 61.9 % of the commanded distance (median 65.6 %) against the 40 % gate, no falls in any cell, and all the CI jobs except the acceptance one are green — but `flat_A_bwd10` shows max |dyaw5| 10.8° in one repeat out of five (the other four are 3.3-6.1°), so `jazzy_real_scoring` is FAIL, the plan stops at `CHECKPOINT REACHED`, and `ci.yml` and `docs/TERRAIN.md` are deliberately untouched.**

## Performance

- **Duration:** 1h 45m (two CI runs of ~25 minutes each plus the report)
- **Started:** 2026-10-26T14:10Z
- **Completed:** 2026-10-26T15:55Z
- **Tasks:** 2 of 4 executed (data, report); tasks 3-4 are the pass branch
- **Files created:** 10 (8 JSONs, summary.md, report)

## Accomplishments

- **Data:** run A (40328414499, `repeats=5 cells=all strict=true`) and run B (40328467878, `repeats=10 cells=flat_A_bwd10`) at `0917551`; every JSON carries that `git_sha`, each cell has five or ten valid repeats, and the eight files plus `summary.md` are committed under `acceptance/final/`.
- **Numbers, Jazzy real (the scoring element):** `flat_A_bwd10` min 61.9 %, median 65.6 %, heading 10.8° (gate 10°); `flat_B_bwd10` 48.2 % / 53.1 %; waves 37.0 %; rocks 46.7 %; `flat_B_bwd05` 14.1 % (report only); falls 0 everywhere.
- **The failure is one repeat:** heading per repeat 10.8 / 5.5 / 4.5 / 3.3 / 6.1°; run B's ten repeats of the same cell peak at 6.8°. Across 20 samples of `flat_A_bwd10` today there is exactly one value above 10°.
- **Lyrical reference and Jazzy ideal:** Lyrical passes both models (`flat_A_bwd10` 43.0 % real, 54.4 % ideal), Jazzy ideal 43.5 % / 69.4 % — the `real` profile costs about 3.8 p.p. on the median, and on the minimum the spread dominates.
- **Derived thresholds (recorded, not applied):** Jazzy 0.30, Lyrical 0.40 by the D-05 formula from the ideal model over 15 repeats per distro.
- **Everything else green in run A:** `build + test` on both distros, `gazebo walk check` on both, `terrain limits`, `robot image`, `robot parameter form`, `servo speed analysis`, `camera auto-calibration tool`.

## Task Commits

1. **Task 1 (data):** `37d3ea2` - docs(01-17): final acceptance data
2. **Task 2 (report):** `ac11642` - docs(01-17): acceptance report (Jazzy scoring failed)
3. **Plan close at the checkpoint (this SUMMARY, STATE, ROADMAP, .continue-here)** - this commit (docs)

## Decisions Made

- **No second run.** The owner declined the repeat after seeing the per-repeat breakdown ("не. закрывай"). The evidence is in the report either way: five repeats of the gate cell and ten more of the same cell.
- **The threshold is not applied.** Per D-17 the threshold and the docs move only on a passed gate, so `ci.yml` stays at 0.2 and `docs/TERRAIN.md` has no Phase 1 section yet.
- **The derived numbers are recorded anyway.** 0.30 / 0.40 are in the report so a later decision does not need another 50 minutes of CI.

## Deviations from Plan

- **Tasks 3 and 4 are not executed**, by the plan's own FAIL branch (task 2, step 5). No `ci.yml` change, no `docs/TERRAIN.md` section, no push-CI verification of the new threshold.
- **The one permitted re-run was not used:** the owner closed the plan instead (2026-10-26).

## Issues Encountered

- **The 5-repeat maximum is a fragile gate.** Typical heading is 3-7°; a single 10.8° repeat is enough to fail. The report says so plainly and run B (ten repeats, max 6.8°) is the counter-evidence.
- **Runs A and B overlapped** (B was dispatched once A's RUN_ID appeared, as the plan requires for script safety), so runner contention is a plausible contributor to the outlier; this is recorded as a suspicion, not a proven cause.

## Next Phase Readiness

- **Phase 01 cannot close:** GAIT-01 (the scoring gate) and GAIT-02 (the push threshold) stay open; GAIT-06 is verified by the walk checks and by 100 acceptance repeats without a fall.
- **Three ways forward, for the owner:** (1) fix the walk against the gap (`/gsd-plan-phase 1 --gaps`) — the heading drift in mode A is the only defect; (2) re-run the acceptance once on a quiet runner if 10.8° is judged an outlier; (3) revisit the criterion itself — D-01's "maximum over five repeats" is what makes a single outlier decisive, and the plan does not change criteria on its own.
- **Data is ready for any of the three:** eight JSONs at a known SHA, a report that regenerates from them, and a generator script (`ros2_ws/build/_ci/01-17-report.py`).

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Checkpoint reached: 2026-10-26*
