---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 14
subsystem: bench-measurement
tags: [servo-speed, mg996r, ina219, pca9685, orange-pi-zero-3w, cal-17, unusable, protractor, us-per-deg]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 12
    provides: "dog_bench/servo_speed_test (selftest / dry-run / run), the CSV and meta format, SignalGuard and the servo release on every exit"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 06
    provides: "tools/servo_speed/analyze.py and its 77 pytest: v_sat by the neighbour-speed trace distance, v_dur, the verdict flags"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 07
    provides: "01-MEASUREMENT-SHEET.md (parts A-D) that frames the owner's steps"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 13
    provides: "the pre-gait-change baseline and the CI acceptance jobs; the measurement itself needs neither"
provides:
  - "Measured protractor calibration of the thigh servo: us_per_deg = 12.065 (two independent pulse pairs), 27.8 % above the 9.444 nominal"
  - "Four bench recordings of the thigh ramp (lf/rf/lr/rr, channels 1/4/7/10, 136.25 s each, ~126k rows, peaks 2.54-2.87 A, stop_reason completed) with meta and analysis JSONs"
  - "01-SERVO-SPEED-RESULT.md: verdict unusable, the noise analysis that explains it, and the contract for 01-15 (speed and margin PENDING, bus_v_mean measured)"
  - "Evidence that the image pipeline works: phase-branch image built with buildx for arm64, loaded on the Orange Pi in 2m15s, servo_speed_test --help exit=0, selftest PASS at 920 Hz"
  - "A configuration finding: at 12.065 us/deg the configured 520..2220 us window covers about 141 deg, not the 180 deg written in range_deg"
affects: [01-15 (nothing measured to input), 01-16, phase 02, phase 03 (driver speed clamp), phase 07 (servo calibration)]

# Actuals (#2632)
actuals:
  tokens: 2200
  tasks: 4
  commits: 1

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Bench measurement is taken over SSH with the tool inside the compose container and the CSV written to a bind-mounted bench/ directory"
    - "The servo page that owns the PCA9685 is stopped with systemctl (root) before the tool runs; the page is started again afterwards so the board returns to its previous state"
    - "Every recording is analysed separately and the four analyses are compared before any verdict is written"

key-files:
  created:
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-SERVO-SPEED-RESULT.md
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/run_01.csv
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/run_02.csv
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/run_03.csv
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/run_04.csv
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/analysis_01.json
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/analysis_02.json
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/analysis_03.json
    - .planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/analysis_04.json
  modified: []

key-decisions:
  - "us_per_deg = 12.065 from both protractor pairs (472 us -> 38.1 deg and 600 us -> 50.75 deg) instead of the plan's single-pair 472/38.1 = 12.388; the owner approved the combined value because the wider span has the smaller reading error"
  - "The measurement ran at 7.48 V, not at the 6.0 +- 0.1 V that D-10 demands: the owner decided to take it as preliminary (no 6 V BEC on the bench) and not to feed the number into 01-15"
  - "Four thighs were measured instead of one (owner's request after the first leg returned unusable), so the four analyses could be compared"
  - "Verdict unusable and the owner rejected the measurement: 6.0 rad/s stays in robot.yaml and servos.yaml, servo.margin stays 0.8, 01-15 has no measured speed to input"
  - "The 520..2220 us window at 12.065 us/deg covers about 141 deg, not 180: recorded in the report as a calibration question for a later phase, not fixed here"

patterns-established:
  - "A bench measurement that the analysis method cannot resolve is reported as unusable rather than tuned until it yields the expected number"

requirements-completed: []

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "The phase-branch image with dog_bench is built for arm64, transferred to the board and answers: servo_speed_test --help exit=0, pca9685_probe check/off, selftest PASS at 920 Hz with 0 I2C errors"
    requirement: "CAL-17"
    verification:
      - kind: manual_procedural
        ref: "ssh to 192.168.31.166: docker compose run --rm robot .../servo_speed_test --help -> exit=0; selftest -> 'PASS selftest', median 1.08667 ms, p99 1.09326 ms, errors 0"
        status: pass
    human_judgment: false
  - id: D2
    description: "Four ramp recordings of the thigh servo (lf/rf/lr/rr) with meta: 136.25 s each, ~126k rows, stop_reason completed, peak 2.54-2.87 A, output released after every run"
    requirement: "CAL-17"
    verification:
      - kind: integration
        ref: "verify-01: meta vs the owner's values (shunt 0.1, channel, us_per_deg) and the CSV header -> 'verify1 ok, stop_reason = completed'"
        status: pass
    human_judgment: false
  - id: D3
    description: "01-SERVO-SPEED-RESULT.md with the measured us_per_deg, the four-run table, the crack check, verdict = unusable, the contract lines and the owner's rejection"
    requirement: "CAL-17"
    verification:
      - kind: other
        ref: "verify-03: exactly one contract line each for servo_max_speed_rad_s / bus_v_mean / servo_margin_approved / verdict, bus_v_mean equal to analysis_01.json -> 'verify3 ok unusable'"
        status: pass
    human_judgment: false
  - id: D4
    description: "A usable servo speed for the gait period"
    verification: []
    human_judgment: true
    rationale: "Not delivered: the analysis method cannot separate the neighbour-speed traces from the repetition noise on a free-hanging leg, so the owner rejected the measurement and kept 6.0 rad/s. Deciding whether to re-measure (other rig, more repetitions, a different statistic) or to keep the design value belongs to the owner."

# Metrics
duration: 2h 40m
completed: 2026-10-26
status: complete
---

# Phase 1 Plan 14: MG996R speed on the bench — measured geometry, four recordings, verdict unusable

**The thigh servo's pulse-to-angle scale was measured with a protractor (12.065 us/deg, 27.8 % above the 9.444 nominal, which also means the configured 520..2220 us window covers about 141 deg, not 180), four ramp recordings were taken on lf/rf/lr/rr with the phase-branch image, and all four analyses returned `saturated_at_first_speed` because the repetition noise is three times the neighbour-speed difference — the owner rejected the measurement and 6.0 rad/s stays in the config.**

## Performance

- **Duration:** 2h 40m (image build 19 min, transfer 2 min, four ramp runs 2.5 min each, analysis and report; the earlier part A/C/D checklist of this plan was done on 2026-10-06)
- **Started:** 2026-10-26T10:37Z (image build from the phase branch)
- **Completed:** 2026-10-26T12:20Z
- **Tasks:** 4 (task 1 checkpoint, task 2 checkpoints, task 3 analysis, task 4 decision)
- **Files modified:** 13 (1 report, 4 CSVs, 4 metas, 4 analysis JSONs)

## Accomplishments

- **Image and board:** the phase branch was built for arm64 with `docker buildx` (9 packages, `dog_bench` included, 19 min), loaded on the Orange Pi Zero 3W in 2m15s (`docker save | gzip | ssh | docker load`), the previous image kept as `robot-dog:2.0-before-phase1`, and the new one tagged `robot-dog:2.0`. `servo_speed_test --help` answers with exit=0 (`image_ok = yes`).
- **Preflight:** `pca9685_probe check` (50.0288 Hz) and `off` green; `i2cdetect` on `/dev/i2c-0` shows 0x29, 0x40, 0x41, 0x68, 0x70; `selftest` PASS — 2000 ticks, median 1.08667 ms (920 Hz), p99 1.09326 ms, max 1.15288 ms, errors 0, bus 7.26 V, quiescent 47.1-51.6 mA.
- **Protractor calibration (task 1):** two pulse pairs on channel 1 with the owner reading the angle: 1134→1606 us gave 38.1 deg (12.389 us/deg) and 1100→1700 us gave 50.75 deg (11.823 us/deg); the combined 1072 us / 88.85 deg gives **us_per_deg = 12.065**, i.e. +27.8 % against the MG996R nominal 9.444. Implied full-travel window is about 2172 us, while the config's 520..2220 us spans only about 141 deg although `range_deg` says 180.
- **Four recordings (task 2):** `run_01` lf ch 1, `run_02` rf ch 4, `run_03` lr ch 7, `run_04` rr ch 10 — each 136.25 s, 125 732-127 054 rows at 922.8-932.5 Hz, amplitude ±25 deg around 1370 us, peaks 2.54-2.87 A, `stop_reason = completed`, output released with `pca9685_probe off` after every run.
- **Analysis (task 3):** all four JSONs give `v_sat = 1.5 rad/s`, `agreement = no`, flag `saturated_at_first_speed`; `v_dur_plateau` 10.45-14.08 rad/s; plateau current 0.79-1.04 A; `eps_rel` 0.271-0.319 against `distance_rel` 0.042-0.188. The raw peaks grow 0.23→1.55-1.86 A up to 9-10 rad/s and `v_dur` on the 8-10 steps matches the commanded speed, so the servo is fast, but no number passes the method's own significance test.
- **Report and decision (task 4):** `01-SERVO-SPEED-RESULT.md` records the inputs, the four-run table, the crack check, `verdict = unusable`, the contract (`servo_max_speed_rad_s = PENDING`, `bus_v_mean = 7.224`, `servo_margin_approved = PENDING`) and the owner's decision of 2026-10-26: reject the measurement, keep 6.0 rad/s.

## Task Commits

1. **Tasks 1-4 (image, four runs, analysis, report)** - `b16a2c6` (docs; 13 files: report, four CSVs, four metas, four analyses)

**Plan metadata:** `docs(01-14): complete the servo speed measurement plan` (this commit: SUMMARY + STATE + ROADMAP)

## Files Created/Modified

- `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/01-SERVO-SPEED-RESULT.md` - inputs, the four-run table, the crack check, verdict, contract, owner's rejection
- `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/run_0{1,2,3,4}.csv` + `.meta.json` - the recordings and their metadata (shunt 0.1, us_per_deg 12.065, amp 25, center 1370, channel per leg, stop_reason completed)
- `.planning/phases/01-zamery-na-stole-i-hod-nazad-v-simulyatsii/servo_speed/analysis_0{1,2,3,4}.json` - the analyzer output for each leg

## Decisions Made

- **us_per_deg = 12.065 (combined), not 12.388 (single pair).** The plan's formula uses one pair; the owner approved the combined value because the 600 us span carries the smaller protractor error, and the two pairs agree within 4.8 %.
- **Measurement at 7.48 V is preliminary.** D-10 asks for 6.0 ± 0.1 V; there is no 6 V BEC on the bench. The owner decided to measure now and not to treat the result as final.
- **All four thighs.** After the first leg returned unusable the owner asked for the same run on rf/lr/rr, which turned a single-leg oddity into a property of the whole method.
- **Rejected, not retried.** The owner's decision: the measurement is unusable, 6.0 rad/s and `servo.margin` 0.8 stay; a retry needs different conditions, not another pass of the same procedure.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking] The servo page could not be stopped without root**
- **Found during:** Task 2 (preparing the board for the ramp)
- **Issue:** `servo_pose.service` owns the PCA9685 (`User=sg`, `Restart=on-failure`); `systemctl stop` as `sg` returns "Access denied", and killing the process would have made systemd restart it
- **Fix:** the page's channels were released through its own API, then the owner supplied the sudo password and the service was stopped cleanly; started again after the runs, page HTTP 200, all channels full-off
- **Files modified:** none (board state only)
- **Verification:** `systemctl is-active servo_pose` = inactive during the runs, port 8080 free; after the runs active and HTTP 200
- **Committed in:** board state, not in git

**2. [Rule 1 - Bug] Extra recordings under the plan's retry name**
- **Found during:** Task 2/3
- **Issue:** the plan reserves `run_02` for a retry of the failed leg, but the owner's request for the other three legs took `run_02..run_04`
- **Fix:** the report states explicitly which run belongs to which leg and channel, so the names stay unambiguous; no retry was taken
- **Files modified:** `01-SERVO-SPEED-RESULT.md`
- **Verification:** the plan's meta verify passes on `run_01` (channel 1, shunt 0.1, us_per_deg 12.065)
- **Committed in:** b16a2c6

---

**Total deviations:** 2 (1 blocking, 1 naming)
**Impact on plan:** no scope creep; the voltage (7.5 V instead of 6.0 V) and the `us_per_deg` choice are owner decisions recorded in the report, not silent changes.

## Issues Encountered

- **The analyzer cannot resolve the knee on a free-hanging leg.** Neighbour-speed trace distances are 4-19 % of the plateau while the repetition noise is 9-11 %, so `eps = 3σ` is 27-32 % and the rule "traces coincide = saturated" fires at the very first speed in every run. A check with the grid cut offline to 4..10 rad/s reproduces the same result, so the grid range is not the cause: at 1.5-4 rad/s the servo is barely loaded (0.10-0.20 A against 0.05 A quiescent) and the traces really do look alike.
- **`v_dur` is unusable on the low steps** (8.5-18.2 rad/s for commanded 1.5-7) and agrees with the command only on 8-10 rad/s, where the current pulse is large enough to time.
- **Process:** the Claude Code session that started this plan hit its session limit at 13:42 MSK; the work was picked up in a Hermes session with the transcript as the source of state. Nothing was lost: the image build was still running and finished at 13:56.

## User Setup Required

None - no external service configuration required. The bench itself needs the 6 V BEC for a final (non-preliminary) measurement; the servos are left under 7.2-7.5 V power with all channels off and the switch belongs to the owner.

## Next Phase Readiness

- **01-15 (input the measured numbers) has nothing to input:** the speed and the margin stay PENDING, `robot.yaml` and `servos.yaml` keep 6.0 rad/s. The plan needs an explicit disposition before the wave is executed.
- **01-16 (the backward transfer shape) is not blocked by this measurement**, because it works on the simulator data from the baseline in 01-13.
- **Open calibration question for a later phase:** at the measured 12.065 us/deg the configured pulse window 520..2220 us covers about 141 deg while `range_deg` claims 180, so commanded angles deliver less rotation than the driver assumes. Either the window widens to about 380..2560 us or `range_deg` follows the real travel.
- **If the speed is ever needed:** change the conditions, not the threshold — measure with the robot on its side (gravity removed from the leg) or with several times more strokes per speed, otherwise the repetition noise covers the neighbour-speed difference again.

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-26*
