---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 07
subsystem: docs-process
tags: [measurement-sheet, parts-order, review, simulation, deployment, terrain, readme, auto-period, servo-model]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 01
    provides: "phase branch and the tools/ conventions (offline python3 checks are the local tooling)"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 02
    provides: "frozen Phase 1 key names (servo.*, servo_sim.*, gait.auto_period/min_period, description.body_com_x) and the GROUPS field ids/labels the sheet must match"
provides:
  - "01-MEASUREMENT-SHEET.md part 1: owner tables (leg, body+COM, masses, limits, knee rod) with ids/labels verbatim from robot_setup GROUPS, the D-22 leg-scheme STOP check and the chat answer format (CAL-16, D-18..D-22, D-26)"
  - "01-MEASUREMENT-SHEET.md part 2: power multimeter checklist (STOP at 5 V on VCC), shunt R100/R010, INA219 address 0x41/0x44/0x45, docker compose stop + pca9685_probe off, protractor us_per_deg = 472 / angle, run per tools/servo_speed README (CAL-17, D-09..D-11)"
  - "docs/PARTS_ORDER.md: V+ servo switch (SAF-09), LiPo low-voltage alarm (SAF-17), current-limited lab supply (Phase 7), spare MG996R; nothing to buy for Phase 1 (D-09)"
  - "docs/REVIEW.md item 26: the driver clamps joint-space speed while the knee servo runs 1.3-1.6x faster - handed to Phase 3 (PR-03)"
  - "Docs corrected to 10/10 (stand, 8 manevrov, lie) and the 'Realistic servo profile and auto period' section with the 01-04 period table (GAIT-06, GAIT-10)"
affects: [01-14 (owner runs the sheet), 01-15 (enters the numbers), 01-08 and 01-10 (documented key names), phase 01 verification, Phase 3 (REVIEW item 26)]

# Actuals (#2632) - pairs with the plan's estimate
actuals:
  tokens: 11026
  tasks: 4
  commits: 4
plan_head_before: 5a1766f2f449f103b8f6cfc997f7fc3e20696cee

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "The measurement sheet mirrors robot_setup GROUPS: field ids and CLI labels are asserted against the live module (r.FIELDS[i][1]), not a hand copy, so form label drift reddens the check"
    - "A plan assumption (A1) narrows an outline-level instruction where the outline number is factually wrong for a subset of rows: the TERRAIN slope row keeps 8/8 because walk_check runs 8 checks on a slope"

key-files:
  created:
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-MEASUREMENT-SHEET.md
    - docs/PARTS_ORDER.md
  modified:
    - docs/REVIEW.md
    - docs/SIMULATION.md
    - docs/DEPLOYMENT.md
    - docs/TERRAIN.md
    - README.md

key-decisions:
  - "A1: docs/TERRAIN.md keeps 8/8 in the slope row with the (уклон: stand, 6 манёвров, lie) mark; 10/10 (stand, 8 манёвров, lie) is recorded for the flat floor - walk_check.py runs 8 checks on slope, 10 on flat, a blind replacement would have been wrong"
  - "Step 1.0 of the sheet carries the D-22 STOP (hip axis, three-joint scheme, knee direction, axis height difference); a mismatch returns to the owner, the kinematics is never changed silently (PR-06)"
  - "dog_hardware was not touched: the driver speed clamp goes to Phase 3 via docs/REVIEW.md item 26 (PR-03); Phase 1 counts the period in servo space (servo.knee_ratio, gait.auto_period)"

patterns-established:
  - "Owner-facing sheets assert their completeness against the live tool module (GROUPS/FIELDS), not against a copied list"

requirements-completed: [CAL-16, CAL-17, GAIT-06, GAIT-10]

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "Measurement sheet part 1: usage rules, the D-22 STOP check with five questions, five tables (leg, body+COM, masses, limits, knee rod) with ids and labels verbatim from robot_setup GROUPS"
    requirement: "CAL-16"
    verification:
      - kind: other
        ref: "python3 GROUPS/FIELDS assertion over the sheet -> 'missing [] []', exit 0; grep verify: '^| Поле' = 5, СТОП >= 2, ±3 мм, all four schemas present, exit 0"
        status: pass
    human_judgment: false
  - id: D2
    description: "Measurement sheet part 2: bench conditions, multimeter checklist with the 5 V VCC STOP, shunt R100/R010, INA219 address, docker compose stop + pca9685_probe off, protractor 1134/1606 us and 472 / angle, run per README, not_saturated reading"
    requirement: "CAL-17"
    verification:
      - kind: other
        ref: "18-string grep checklist + table counts ('^| Шаг' = 5, '^| Поле' = 5) over the sheet, exit 0"
        status: pass
    human_judgment: false
  - id: D3
    description: "docs/PARTS_ORDER.md (four-row order table: SAF-09 switch, SAF-17 alarm, Phase 7 lab supply, spare MG996R) and docs/REVIEW.md item 26 (servo_driver.cpp:226 clamp, 1.3-1.6x, Phase 3 owner)"
    requirement: "GAIT-06"
    verification:
      - kind: other
        ref: "grep row/string checks for PARTS_ORDER.md and the awk 'Открыто' section check for REVIEW.md (rows 12-25 intact), exit 0"
        status: pass
    human_judgment: false
  - id: D4
    description: "Count 10/10 (stand, 8 манёвров, lie) in README/SIMULATION/DEPLOYMENT/TERRAIN (slope row keeps 8/8 with the mark); the 'Realistic servo profile and auto period' section with the 01-04 period table; DEPLOYMENT body_com_x subsection, knee rod lengths and CAL-17 note; README PARTS_ORDER row"
    requirement: "GAIT-10"
    verification:
      - kind: other
        ref: "three grep verifies (counts; SIMULATION/DEPLOYMENT keys; README) + the git-log scope check over all (01-07) commits, exit 0; walk_check.py:213-230 source read confirms 8 slope / 10 flat checks"
        status: pass
    human_judgment: false
  - id: D5
    description: "Backstop (plan must_haves, verification: backstop): the owner's measurement accuracy (caliper, scales, protractor) is not checked by automation"
    verification: []
    human_judgment: true
    rationale: "The sheet sets tolerances; errors are caught by robot_setup --check (masses within 8 %, stand reachability) and walk_check in plan 01-15"
  - id: D6
    description: "Backstop: the sheet is written to be filled by the owner on the real robot (parts 1-2); no automation can prove the robot-side procedure works before 01-14"
    verification: []
    human_judgment: true
    rationale: "01-14 runs the procedure on the bench (selftest, dry-run, run) and 01-15 enters the numbers; the sheet is their input document"

# Metrics
duration: 6 min
completed: 2026-10-05
status: complete
---

# Phase 1 Plan 07: Owner and process documents (measurement sheet, parts order, docs) Summary

**Owner measurement sheet with the D-22 STOP check and the servo-speed power/shunt/protractor checklist (ids and labels verbatim from `robot_setup` GROUPS), a four-row parts order, the driver-clamp handover to Phase 3, and docs moved to «10/10 (stand, 8 манёвров, lie)» plus the realistic servo profile and auto-period section — docs-only, all source assertions green, no simulation run (D-24).**

## Performance

- **Duration:** 6 min
- **Started:** 2026-10-05T17:47:55Z
- **Completed:** 2026-10-05T17:54:10Z
- **Tasks:** 4
- **Files modified:** 7 (2 created, 5 modified; plus the four .planning close-out files)

## Accomplishments

- `01-MEASUREMENT-SHEET.md` part 1: usage rules (tools D-19, «between axis centres», tolerances, `идентификатор = значение` chat format), Step 1.0 — the D-22 leg-scheme check with the STOP rule — and five tables (leg, body with `body_com_x`, masses, limits, knee rod) whose ids and labels are the live `robot_setup` GROUPS strings.
- Part 2: bench conditions (D-09/D-10), the four-row multimeter checklist (6.0 ± 0.1 V on V+, common ground, VCC 3.3 ± 0.1 V with STOP at 5 V, capacitor), shunt R100/R010 with `--shunt-ohm`, address 0x41/0x44/0x45 with `--ina-address`, `docker compose stop` + `pca9685_probe off`, protractor 1134/1606 μs and `us_per_deg = 472 / angle`, run via `tools/servo_speed/README.md`, `not_saturated` from 7 rad/s read as a fast servo (not a failure).
- `docs/PARTS_ORDER.md`: V+ servo switch (SAF-09), LiPo low-voltage alarm (SAF-17, 3.5 V per cell), current-limited lab supply (Phase 7), spare MG996R; explicit «Не нужно для Phase 1» (D-09).
- `docs/REVIEW.md` item 26: the driver clamps joint-space speed (`servo_driver.cpp:226`) while the knee servo runs 1.3–1.6× faster; Phase 3 owns the servo-space fix via `Linkage` — `dog_hardware` untouched (PR-03).
- Count corrected to «10/10 (stand, 8 манёвров, lie)» in README, SIMULATION and DEPLOYMENT (slope table keeps «8/8» with the «(уклон: stand, 6 манёвров, lie)» mark); SIMULATION now describes the ideal speed limiter (no «позиционный регулятор», `sim_p_gain` unused) and carries the «Реалистичный профиль серв и автопериод» section with the plan 01-04 period table (1.030 … 0.5575, 0.550 floor); DEPLOYMENT gained the `body_com_x` subsection, the four knee rod lengths (D-20) and the CAL-17 note; README documents table links `docs/PARTS_ORDER.md`.

## Task Commits

Each task was committed atomically:

1. **Task 1: Measurement sheet part 1 — table measurements (CAL-16)** - `c258ba5` (docs)
2. **Task 2: Measurement sheet part 2 — servo speed preparation (CAL-17)** - `b31c4ed` (docs)
3. **Task 3: Parts order + REVIEW item 26 (SAF-09, SAF-17)** - `719be13` (docs)
4. **Task 4: 10/10 count, servo profile and auto period (GAIT-06, GAIT-10)** - `3f04d9e` (docs)

**Plan metadata:** `docs(01-07): complete owner and process documents plan` (this commit: SUMMARY + STATE + ROADMAP + REQUIREMENTS)

## Files Created/Modified

- `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-MEASUREMENT-SHEET.md` - the owner sheet: part 1 (table, D-22 STOP) and part 2 (servo-speed preparation) — created in tasks 1-2
- `docs/PARTS_ORDER.md` - the parts order list with the four rows and the selection notes
- `docs/REVIEW.md` - item 26 in «Открыто»: the driver speed clamp handed to Phase 3
- `docs/SIMULATION.md` - ideal speed limiter wording, 10/10 example, «Реалистичный профиль серв и автопериод» section
- `docs/DEPLOYMENT.md` - PARTS_ORDER link, form checks, `body_com_x` subsection, knee rod note, 10/10 criterion, CAL-17 note
- `docs/TERRAIN.md` - slope row keeps «8/8» with the mark; flat-floor «10/10 (stand, 8 манёвров, lie)» line below the table
- `README.md` - «10/10 (stand, 8 манёвров, lie)» status row and the `docs/PARTS_ORDER.md` documentation row

## Decisions Made

- A1 (plan assumption): the slope row in TERRAIN.md keeps «8/8» with an explicit mark; «10/10 (stand, 8 манёвров, lie)» is recorded for the flat floor. A blind replacement would have introduced a wrong count.
- Step 1.0 of the sheet carries the D-22 STOP: hip axis along the body, three joints per leg, knee backwards, axis height difference ≤ 5 mm; a mismatch returns to the owner — the kinematics is never changed silently (PR-06).
- `dog_hardware` untouched: the clamp goes to Phase 3 via REVIEW item 26; Phase 1 counts the period in servo space (`servo.knee_ratio`, `gait.auto_period`).

## Deviations from Plan

None - plan executed exactly as written.

**Recorded per the plan objective (A1, an outline-level deviation):** the outline asked to replace «8/8» everywhere; assumption A1 keeps «8/8» in `docs/TERRAIN.md` (slope table) with the «(уклон: stand, 6 манёвров, lie)» mark — `walk_check` runs 8 checks on a slope, so the number is correct there — and records «10/10 (stand, 8 манёвров, lie)» for the flat floor. Executed exactly as A1 specifies; `report/index.html` untouched.

## Issues Encountered

- The session `python3` is the Hermes toolchain interpreter (3.14, no PyYAML); every verify command ran with `/usr/bin/python3` (3.13.5), as in plans 01-02..01-06. No repository change.
- `requirements.ready-ids` reports 0/4 ready (CAL-16, CAL-17, GAIT-06, GAIT-10 are declared by sibling plans without summaries yet), so the shared-ID gate correctly deferred marking them complete.
- No `colcon`/`ros2` and no simulations were run (D-24): the checks are grep/python3 source assertions, as the plan prescribes.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- 01-14 (owner) can run the servo-speed procedure from the sheet's part 2; 01-15 enters the numbers per part 1 — the sheet is the input document for both.
- Phase 3 has the driver-clamp entry (REVIEW item 26); plans 01-08/01-10 reference the documented key names (fixed by 01-02).
- The «10/10» count in the docs is confirmed end-to-end by the next CI `walk_check` run on the phase branch (source-level check done here).

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-05*

## Self-Check: PASSED

- All key files exist on disk (7 checked with `[ -f ]`).
- All four commits exist: `c258ba5`, `b31c4ed`, `719be13`, `3f04d9e` (ledger base `5a1766f`; measured `commits: 4`).
- No stubs/TODOs in the changed files; no origin address or token in the changed docs (`grep -rEq 'https?://[^ ]*@'` clean).
