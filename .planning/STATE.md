---
gsd_state_version: "1.0"
current_phase: 01
current_phase_name: Замеры на столе и ход назад в симуляции
status: executing
stopped_at: Completed 01-02-PLAN.md
last_updated: "2026-10-05T08:49:12.524Z"
last_activity: 2026-10-05
last_activity_desc: Phase 01 execution started
state_head: 50cb5bf7271a742ff039fc67a00b73d9a9bb649c
progress:
  total_phases: 8
  completed_phases: 0
  total_plans: 17
  completed_plans: 1
  percent: 0
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-09-29)

**Core value:** Робот надёжно и безопасно ходит по полу с геймпада, не падая и не убивая свои сервы.
**Current focus:** Phase 01 — Замеры на столе и ход назад в симуляции

## Current Position

Phase: 01 (Замеры на столе и ход назад в симуляции) — EXECUTING
Plan: 3 of 17
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

Last session: 2026-10-05T08:49:12.490Z
Stopped at: Completed 01-02-PLAN.md
Resume file: None
