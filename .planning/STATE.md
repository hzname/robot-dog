---
gsd_state_version: "1.0"
current_phase: 01
current_phase_name: Замеры на столе и ход назад в симуляции
status: executing
stopped_at: Completed 01-07-PLAN.md
last_updated: "2026-10-05T17:54:44.414Z"
last_activity: 2026-10-05
last_activity_desc: Phase 01 execution started
state_head: 3f04d9e653376e3bc01f35a1dfe0db830c953ae8
progress:
  total_phases: 8
  completed_phases: 0
  total_plans: 17
  completed_plans: 6
  percent: 0
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-09-29)

**Core value:** Робот надёжно и безопасно ходит по полу с геймпада, не падая и не убивая свои сервы.
**Current focus:** Phase 01 — Замеры на столе и ход назад в симуляции

## Current Position

Phase: 01 (Замеры на столе и ход назад в симуляции) — EXECUTING
Plan: 8 of 17
Status: Ready to execute
Last activity: 2026-10-05 — Phase 01 execution started

Progress: [░░░░░░░░░░] 0%

## Performance Metrics

**Velocity:**

- Total plans completed: 0
- Average duration: -
- Total execution time: 0.0 hours

**By Phase:**

| Phase | Plans | Total | Avg/Plan |
|-------|-------|-------|----------|
| - | - | - | - |

**Recent Trend:**

- Last 5 plans: -
- Trend: -

*Updated after each plan completion*
**Per-Plan Metrics:**

| Plan | Duration | Tasks | Files |
|------|----------|-------|-------|
| Phase 01 P01 | 13 min | 3 tasks | 2 files |
| Phase 01 P02 | 7 min | 4 tasks | 4 files |
| Phase 01 P03 | 6 min | 3 tasks | 4 files |
| Phase 01 P04 | 8 min | 3 tasks | 4 files |
| Phase 01 P05 | 10 min | 3 tasks | 11 files |
| Phase 01 P06 | 23 min | 3 tasks | 6 files |
| Phase 01 P07 | 6 min | 4 tasks | 7 files |

## Accumulated Context

### Decisions

Decisions are logged in PROJECT.md Key Decisions table.
Recent decisions affecting current work:

- [Roadmap]: порядок владельца: симуляция, защита серв и корпуса, калибровка на железе, пол
- [Roadmap]: Phase 3 (защита драйвера) идёт параллельно с Phase 1–2, решение владельца 2026-09-29; Phase 4 ждёт обоих потоков
- [Roadmap]: защёлка аварии драйвера (SAF-03) строится в Phase 3 раньше всего, что через неё эскалирует
- [Roadmap]: keep-alive калибровки поставляется в одном изменении с watchdog драйвера (SAF-04)
- [Roadmap]: защита по наклону включается только после проверки IMU (CAL-08, Phase 7); физический выключатель (SAF-09, Phase 6) обязателен до пола
- [Phase 01]: Replaced the tokenized origin URL with the canonical HTTPS URL of the repository; the gh credential helper (account hzname, scope repo) now authenticates git and gh operations non-interactively
- [Phase 01]: ci_dispatch.sh validates --ref/-f/--run-id/--wait-job before any network call; values pass only as argv and are never built into shell strings
- [Phase 01]: run.sh rejects dog_hardware on purpose (phase 3 territory) and always builds with -Werror; Task 1 (branch creation) intentionally has no commit
- [Phase 01]: The Phase 1 config contract is fixed by plan 01-02: robot.yaml carries gait.auto_period/gait.min_period, the servo.* and servo_sim.* blocks (reference values with a source and unit each) and description.body_com_x; plans 01-04, 01-08, 01-10 and 01-15 consume these names without redefining them — Names and formats must be fixed by one plan; otherwise parallel plans redefine them. All values keep the current behaviour (auto_period off, nodes ignore undeclared YAML keys) so existing simulation results stay comparable (D-12, D-15).
- [Phase 01]: The three servo speeds (description.servo_velocity, servo.max_speed, servos.yaml max_joint_speed) are pinned as one number by tools/robot_setup/test/test_yaml_contract.py, which the existing robot-setup CI job already runs; a divergence fails CI instead of warming up under-specced servos (D-13) — A single number duplicated in YAML diverges silently; catching it in the tool job keeps the period computed from the speed the servo really has (Core Value: not killing servos).
- [Phase 01]: robot_setup keeps measurement entry unblocked: sensor geometry errors for simulated-only sensors become warnings via SENSORS_ON_ROBOT = frozenset() (D-18), and knee_ratio_max ports the driver Linkage with C++ golden values, warning when robot.yaml servo.knee_ratio is below the computed ratio (D-20) — The robot carries no perception sensors (docs/HEAD.md), so their template geometry must not block entering real body measurements; the knee rod drive makes the servo faster than the joint (up to 1.39x), and a too-low knee_ratio would understate the period and the servo peak.
- [Phase 01]: acceptance_stats stays pure stdlib on top of terrain_sweep.never_stood; test_constants_match_walk_check pins MIN_BODY_HEIGHT 0.108 and FALL_TILT_DEG 60.0 to walk_check.py, and the acceptance constants (MIN_RATIO 0.40, MAX_DYAW5_DEG 10.0, MAX_TILT_DEG 20.0, MIN_REPEATS 5, floor 0.2) are never retuned (D-01, D-03, D-07)
- [Phase 01]: run_record is the single translator of walk_check --trace JSON into runs[]; a fall in the last manoeuvre is caught by tilt_deg > 60.0 without the skipped-record flag, and missing data (dyaw5, backward ratio, tilt_deg, z) is always a failure reason, never a skipped check (GAIT-01)
- [Phase 01]: D-05 thresholds are per distro, ideal model only, from cell flat_A_bwd10 with n >= 5; the threshold CLI exits 0 (derived) / 1 (not derivable) / 2 (unreadable file, schema not 1, unknown distro) and writes nothing to disk (GAIT-02)
- [Phase 01]: peakServoSpeed and minimalPeriod are tested against pinnedParams() (the v1 robot as shipped), never against code defaults, so the plan 01-15 defaults sync cannot redden the algorithm tests (OI-1)
- [Phase 01]: The pinned period table uses the guard-window fits (4.0 -> 0.9050, 6.35 -> 0.5575); exactly 0.55 s is returned from 6.4 rad/s and the window rejects the fragile first fits 0.8975 and 0.550 (D-11)
- [Phase 01]: minimalPeriod returns 0.0 for no-fit and every degenerate input (NaN, inf, margin*max_speed or knee_ratio not positive, min_period below the TrotGait clamp of 0.1 s, max_period < min_period); plan 01-08 turns 0.0 into a startup refusal
- [Phase 01]: The dog_bench I2C layer keeps the address inside every ioctl(I2C_RDWR) message (no separate address ioctl); FakeI2cBus::failRange accumulates failing transaction ranges — the planned Selftest tests need two disjoint single failures — The bench tool must never write to the PCA9685 at 0x40; accumulation is what the planned CountsConsecutiveErrors test relies on (fix 7a457e2)
- [Phase 01]: runSelftest measures intervals between tick starts (pause counted from the tick start) and judges median <= 1.5 ms and p99 (rank ceil(0.99 n) - 1) <= 5 ms; three consecutive failed transactions abort and the statistics still cover the intervals collected so far; below kSelftestMinSamples = 100 it refuses without touching the bus — Intervals between tick starts carry the transaction cost and scheduler stalls; refusing too few samples keeps the p99 meaningful
- [Phase 01]: servo_speed_test checks every argument before opening the bus; --help wins over all other checks and exits 0; dry-run and run pass validation and then refuse with exit 2 because no PWM code is linked (exit code 3 reserved for plan 01-12) — No code in dog_bench can enable a servo; the 20 wrong argument sets all exit 2 and 0x40/0x50/0x70 never reach the bus
- [Phase 01]: tools/servo_speed contract is fixed by plan 01-06: CSV t_s,stroke_id,direction,cmd_us,shunt_raw,bus_raw and <csv>.meta.json with required shunt_ohm/us_per_deg/speeds_rad_s/strokes_per_speed; analyze CLI exit codes 0/1/2; result keys and flags per README; direction -1/0/+1 outside strokes. Plans 01-12 (session), 01-13 (CI job) and 01-14 (owner graph) consume these names without redefining them — The bench session writes the CSV and the job runs the same pytest; one plan must fix the format or the three plans drift (same rule as the phase-1 config contract from 01-02)
- [Phase 01]: synth.py b_coef default is 0.0001, not the prototype 0.0002: at the sharper value the +/-1 ms front-phase jitter of the current edges inflated the ramp_timing saturated-pair distances past eps (6.0 rad/s, 8 mA, up direction 1.33x eps, kink lost); at 0.0001 the worst pair is 0.62x eps over a seed sweep and v_dur stays 2-4% low — The plan explicitly allows adjusting kp/a_max/tau_i/b_coef within reason when the analyzer tests fail; the reason is recorded in the synth.py docstring
- [Phase 01]: analyze.py per-direction kernels compute I_plateau over that direction's own strokes; the mixed value made up/down plateau currents identical and hid the friction asymmetry (now up - down = 0.079 A, flag direction_mismatch is meaningful) — Per-direction report must carry its own sigma_rep, eps and I_plateau (D-08); the mixed average was a bug surfaced by the per_direction test
- [Phase 01]: A1: docs/TERRAIN.md keeps 8/8 in the 6-degree slope row with an explicit (уклон: stand, 6 манёвров, lie) mark; 10/10 (stand, 8 манёвров, lie) is recorded for the flat floor (walk_check.py runs 8 checks on slope, 10 on flat) — a blind replacement would have written a wrong count
- [Phase 01]: The measurement sheet fixes the D-22 check as Step 1.0 with an explicit STOP: a mismatch of the hip axis, the three-joint leg scheme, the knee direction or the axis height difference (>5 mm) stops the process and returns to the owner; the kinematics is never changed silently (PR-06)
- [Phase 01]: Docs now describe servo_model:=real and the auto period (period table from plan 01-04); dog_hardware was not touched — the driver speed clamp goes to Phase 3 via docs/REVIEW.md item 26 (PR-03)

### Pending Todos

- Заказать детали физического выключателя V+ серв (SAF-09) и сигнализатора низкого напряжения LiPo (SAF-17) в начале Phase 1, чтобы они пришли к Phase 6
- Перевыпустить токен GitHub, вшитый в URL remote `origin`, и перейти на SSH или credential helper (не связано с фазами)

### Blockers/Concerns

- Phase 1: неизвестно, как внедрять задержку, трение, люфт и скорость серв в симуляцию; фазе нужен `--research-phase`, начинать со спайка
- Phase 6-7: датчик тока INA226/INA219 установлен (подтверждено владельцем 2026-09-29), но значение шунта не проверено (10 мОм насыщается около 8 А); напряжение элемента батареи датчик не видит
- Phase 6: неизвестны I2C-нумерация на разъёме M4 Zero, поведение TWI H618 при `I2C_TIMEOUT`, доступность пина OE
- Пороги наклона и тока предварительные; уточнять по данным Phase 2 и записи ходьбы в Phase 7-8

## Deferred Items

Items acknowledged and deferred at milestone close, most recent first:

| Category | Item | Status | Deferred At | Milestone |
|----------|------|--------|-------------|-----------|
| *(none)* | | | | |

## Session Continuity

Last session: 2026-10-05T17:54:33.062Z
Stopped at: Completed 01-07-PLAN.md
Resume file: None
