---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 12
subsystem: bench-tooling
tags: [servo-speed, session, ina219, pca9685, signals, csv, gtest, ci, cpp17]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 09
    provides: "Ramp, SafetyGuard/eventName, Pca9685Out (preflight/pulse/release) and the pca constants (kAllLedOffBytes) this plan assembles without interface changes"
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 06
    provides: "the CSV/meta contract (t_s,stroke_id,direction,cmd_us,shunt_raw,bus_raw and <csv>.meta.json) with analyze.py load_csv/load_meta"
provides:
  - "dog_bench::Session: SessionConfig/Env/Result and run() with the strict order validate+confirm -> preflight -> arm -> read-only pre-roll -> 1 kHz loop -> release -> files; 13 stop reasons and exit codes 0/1/2/3; metaJson"
  - "SignalGuard: SIGINT/SIGTERM/SIGHUP/SIGQUIT set a flag read every tick; SIGSEGV/SIGABRT/SIGBUS/SIGFPE write ALL_LED_OFF into the armed emergency descriptor and _exit(3); openEmergencyFd selects the PCA9685 address for that one write()"
  - "servo_speed_test dry-run and run: the measurement flags, the exact-YES confirmation, the emergency path, the summary/WARN/EMERGENCY output; README section 'Запуск на роботе'"
  - "14 Session/SignalGuard/Confirmation gtests; dog_bench builds and tests green in CI on Jazzy and Lyrical (14 tests, 0 errors, 0 failures, 0 skipped)"
affects: [01-14 (the owner runs servo_speed_test on the robot), 01-13 (servo-speed CI job), phase 01 verification]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "One try around arm + pre-roll + loop; the release lives outside it, so no exception can skip taking the outputs down"
    - "Stop decision per tick: stop_requested -> signal, ramp done -> completed, otherwise one guard event by priority via eventName()"
    - "Rows accumulate in memory (reserved from plannedDurationS) and the CSV/meta are written after the release, before the exit code is computed"
    - "Handler state is process-wide (one SignalGuard); the fatal handler body only reads statics, writes and _exit(3)"

key-files:
  created:
    - ros2_ws/src/dog_bench/include/dog_bench/session.hpp
    - ros2_ws/src/dog_bench/src/session.cpp
    - ros2_ws/src/dog_bench/test/test_session.cpp
  modified:
    - ros2_ws/src/dog_bench/src/servo_speed_test_main.cpp
    - ros2_ws/src/dog_bench/CMakeLists.txt
    - tools/servo_speed/README.md

key-decisions:
  - "The tracer recording runs the first four grid speeds (3.0..4.5): analyze.load_meta requires a grid of at least four speeds (plan 01-06), so a single-speed tracer cannot be accepted by the analysis tool the plan requires it to satisfy"
  - "The release is attempted once and retried once on failure; a persistent failure is exit 3 with the emergency message on stderr"
  - "The meta carries exactly the 11 plan keys with %g numbers; an empty out_path records nothing (the CLI makes --out mandatory for run anyway)"
  - "The bus register is read on every 8th loop tick and the row of that tick carries bus_raw (the selftest cadence); rate_hz = rows / (last tick start - pre-roll start)"
  - "The pca_error test uses a pulse-only write fault (PulseFaultPwm): FakePwm::failWritesFrom also fails release(), which would turn the planned 'released, code 1' assertion into exit 3"

patterns-established:
  - "Session determinism: Sim time advances only through the bus transaction hook and the injected pause; no clocks, no sleeps; fork+pipe for the fatal-signal tests"
  - "Every refusal before the loop leaves the bus untouched and writes no files; files appear only after the release"

requirements-completed: [CAL-17]

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "Session::run drives the ramp on the fake bus/PWM at a 1 kHz poll and writes the CSV + <csv>.meta.json that tools/servo_speed/analyze.py accepts (load_csv and load_meta on the genuine tracer recording)"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "ros2_ws/src/dog_bench/test/test_session.cpp#Session.TracerRampToCsvAndMeta; tools/local_gtest/run.sh dog_bench session -> [  PASSED  ] 14 tests."
        status: pass
      - kind: integration
        ref: "verify 2: analyze.load_csv + analyze.load_meta on ros2_ws/build/_local/dog_bench/session_out/tracer.csv (ids 0..39, stop_reason completed) -> exit 0"
        status: pass
    human_judgment: false
  - id: D2
    description: "Release on every stop reason: overcurrent within 60 ms of the jump, saturation, implausible shunt, INA errors, PCA write error, tick overrun, timeout, signal, exception, refused preflight/confirmation/config - each with releases()>=1, files only after the loop started, exit codes 1/2/3"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "test_session.cpp#Session.{OvercurrentStopsAndReleases,EveryStopReasonReleases,ExceptionReleases,RefusesForeignChannel,RefusesWithoutConfirmation,NoPulseBeforeInaAnswers,InvalidConfigFailsClosed,ReleaseFailureGivesExit3}"
        status: pass
    human_judgment: false
  - id: D3
    description: "SignalGuard: four soft signals set the flag and restore every handler on destruction; four fatal signals in forked children write exactly {0xFA,0,0,0,0x10} into the armed descriptor and _exit(3); disarmed writes nothing; openEmergencyFd refuses a missing device"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "test_session.cpp#SignalGuard.{SoftSignalsSetFlagAndRestoreHandlers,FatalSignalsWriteFiveBytesAndExit3,DisarmedFatalSignalWritesNothing,EmergencyFdRefusesMissingDevice}"
        status: pass
    human_judgment: false
  - id: D4
    description: "CLI: dry-run prints the plan and touches nothing (even with a missing device); run refuses without the exact YES and without a bus; 13 wrong argument sets exit 2 before any bus access; no file appears on a refused run"
    requirement: "CAL-17"
    verification:
      - kind: integration
        ref: "verify 1: g++ -O2 -Werror build + 13 bad sets + dry-run/run behaviors -> exit 0"
        status: pass
    human_judgment: false
  - id: D5
    description: "README section 'Запуск на роботе' as the last heading: the on-robot procedure (stop, probe check/off, selftest, dry-run, run, up), the example command, the exit-code table, the protected/not-protected notes and the analysis hint"
    requirement: "CAL-17"
    verification:
      - kind: other
        ref: "verify 2: last '## ' heading is '## Запуск на роботе', >=7 sections, all required substrings present"
        status: pass
    human_judgment: false
  - id: D6
    description: "dog_bench builds and tests in CI on Jazzy and Lyrical without compiler warnings from src/dog_bench; test_session runs 14 tests with 0 errors, 0 failures, 0 skipped on both"
    requirement: "CAL-17"
    verification:
      - kind: integration
        ref: "CI run 37375137177 job logs (ros2_ws/build/_ci/job_111981778347.log, job_111981778490.log): no 'src/dog_bench/...: (warning|error):', '[==========] 14 tests from 3 test suites ran.', '[  PASSED  ] 14 tests.', 'Summary: 261 tests, 0 errors, 0 failures, 0 skipped'"
        status: pass
    human_judgment: false
  - id: D7
    description: "Backstop (plan must_haves, verification: backstop): SIGKILL, Banana Pi power loss and a kernel hang are not interceptable; the PCA9685 holds the last pulse. The second line is the owner's hand on the power (D-10, plan 01-14) and pca9685_probe off"
    verification: []
    human_judgment: true
    rationale: "Depends on the robot hardware and the owner's procedure; the plan itself carries this as a backstop and plan 01-14 is where the bench runs on the robot"
  - id: D8
    description: "Backstop (plan must_haves, verification: backstop): the H618 100 kHz bus giving >=500 Hz is [ASSUMED]; only selftest and the real run's rate_hz prove it (400 kHz overlay otherwise)"
    verification: []
    human_judgment: true
    rationale: "Depends on the robot hardware; the owner runs selftest on the Banana Pi in plan 01-14"
  - id: D9
    description: "Backstop (plan must_haves, verification: backstop): the us_per_deg error (protractor, about 4 %) is covered by the D-11 margin; CAL-01 calibration is Phase 7"
    verification: []
    human_judgment: true
    rationale: "Cannot be proven without the protractor measurement on the robot (plan 01-14 checks the constant)"

# Actuals (#2632)
actuals:
  tokens: 18135
  tasks: 3
  commits: 2
plan_head_before: 171fe9e1e7fd33a59bfd99a822913b4aaac9370e

# Metrics
duration: 60min
completed: 2026-10-05
status: complete
---

# Phase 1 Plan 12: Measurement session, SignalGuard and the servo_speed_test run mode Summary

**Session::run drives the 01-09 ramp through one PCA9685 channel while polling the INA219 at 1 kHz, writes the CSV + .meta.json that analyze.py's load_csv and load_meta accept, and releases the servo on every one of the 13 stop reasons; SignalGuard adds the soft-flag / fatal-write emergency path — 14 gtests green locally, dog_bench green in CI on Jazzy and Lyrical.**

## Performance

- **Duration:** 60 min
- **Started:** 2026-10-05T20:35:13Z
- **Completed:** 2026-10-05T21:35:26Z
- **Tasks:** 3 (two code tasks committed; task 3 is CI evidence only, its logs are git-ignored)
- **Files modified:** 6 (3 created, 3 modified)

## Accomplishments

- `Session`/`SessionConfig`/`SessionEnv`/`SessionResult` in `session.hpp` + `session.cpp`: the strict order of `run()` — (1) validate + confirmation without touching the bus, (2) preflight (any refusal is `refused_foreign_channel`), (3) one `try` with `on_armed`, the 50-tick read-only pre-roll (every tick feeds the guard, so an event stops the run before the first pulse) and the 1 ms loop, (4) stop decision per tick (`signal` / guard event via `eventName` / `completed`), (5) release outside the try (retried once; failure is exit 3), (6) CSV + meta when the loop had started. Exit codes: 0 `completed`, 2 `invalid_config`, 3 release failure, 1 everything else.
- The loop: shunt read with before/after labels (`t_s` is the midpoint from the pre-roll start, `max_read_ms` the duration), the bus every 8th tick (a failure makes `ina_ok` false), `ramp.update(t0 - previous t0)`, the guard fed with `{ina_ok, raw, pca_ok of the last write, plausibility_window}`, the pulse rewritten every 20 ms (a failed write stops the run on the next tick), rows accumulated in memory with `cmd_us` as the last written pulse (the synth.py staircase), `bus_raw` empty without a bus read.
- `SignalGuard`: SIGINT/SIGTERM/SIGHUP/SIGQUIT write their number into a `volatile sig_atomic_t` read every tick; SIGSEGV/SIGABRT/SIGBUS/SIGFPE run `onFatalSignal` (SA_ONSTACK | SA_RESETHAND, a static 64 KiB alternate stack) — one `write()` of `pca::kAllLedOffBytes` into the armed descriptor, then `_exit(3)`; the destructor restores all eight previous handlers and the alternate stack. `openEmergencyFd` opens the device and selects the PCA9685 slave address — the only place in the package with that ioctl (the measurement bus keeps the address inside every I2C_RDWR message).
- `metaJson` writes exactly the 11 plan keys (`shunt_ohm`, `us_per_deg`, `amp_deg`, `center_us`, `channel`, `speeds_rad_s`, `strokes_per_speed`, `hold_s`, `rest_s`, `ina_config`, `stop_reason`); `isConfirmed` accepts only `YES` after trailing `\r`/`\n` are stripped.
- `servo_speed_test`: `selftest` unchanged; new measurement flags (`--pca-address` (0x40 only), `--channel` (mandatory, exactly one), `--amp-deg`, `--center-us`, `--us-per-deg`, `--speeds`, `--max-seconds` (planned + 5 s must fit), `--out` (mandatory for run, an existing path is refused), `--yes`) accepted by `dry-run`/`run` only; every check runs before the bus and before stdin; `dry-run` prints the plan (INA address/shunt/config/saturation in A; channel, pulse and angle ranges; speeds; planned time; the `overcurrent` safety line) and `dry-run: nothing was written to the bus`; `run` adds the exact-YES confirmation, then opens the bus in the plan's order (bus → INA configure → Pca9685Out → openEmergencyFd → SignalGuard), runs the session with `SessionEnv::realtime`, prints one summary line, a WARN below `kMinPollRateHz` and the EMERGENCY text on exit 3.
- 14 new gtests (tracer, overcurrent, seven stop reasons, exception, two refusals, no-pulse-before-INA, five invalid-config cases, release failure, four SignalGuard, confirmation) — all deterministic (Sim time via the transaction hook; fork+pipe for the fatal signals).
- CI: `dog_bench` builds and tests green on Jazzy and Lyrical in run 37375137177 (all 8 jobs success): no compiler warnings from `src/dog_bench`, `test_session` ran 14 tests, 0 errors, 0 failures, 0 skipped on both.

## Task Commits

Each task was committed atomically:

1. **Task 1 (tracer): measurement session, SignalGuard, CSV/meta and release on every stop reason** - `c12d9e4` (feat)
2. **Task 2: servo_speed_test dry-run and run, README robot-run section** - `efa2cfd` (feat)
3. **Task 3: CI build and test on Jazzy and Lyrical** - no repository change (its files are the git-ignored CI logs in `ros2_ws/build/_ci/`); evidence is run 37375137177

**Plan metadata:** `docs(01-12): complete measurement session and run mode plan` (this commit: SUMMARY + STATE + ROADMAP + WINDOWS)

## Files Created/Modified

- `ros2_ws/src/dog_bench/include/dog_bench/session.hpp` - kSession* constants, kCsvHeader, kConfirmPrompt, SessionConfig/Env/Result, Session, isConfirmed, SignalGuard, openEmergencyFd, onFatalSignal
- `ros2_ws/src/dog_bench/src/session.cpp` - the signal state and handlers, openEmergencyFd (the single address ioctl), isConfirmed, validate, metaJson, csvText and the full run() order
- `ros2_ws/src/dog_bench/test/test_session.cpp` - 14 gtests with the Sim/Rig, PulseFaultPwm, NoReleasePwm and the fork+pipe signal tests
- `ros2_ws/src/dog_bench/src/servo_speed_test_main.cpp` - the extended CLI (flags, validation, plan, confirmation, the run flow with the emergency fd + guard + bus + INA + PWM declarations in destruction order, the result output)
- `ros2_ws/src/dog_bench/CMakeLists.txt` - `src/session.cpp` in the core, `session` in the gtest foreach
- `tools/servo_speed/README.md` - new last section `## Запуск на роботе` (previous sections untouched)

## Decisions Made

- The tracer recording runs the first four grid speeds (3.0, 3.5, 4.0, 4.5; 40 strokes, ids 0..39): `analyze.load_meta` (plan 01-06, fixed contract) requires a grid of at least four speeds, so a single-speed tracer cannot satisfy the plan's own verify and must-have. The refined verify keeps the exact id check, now over 40 strokes (see deviation 1).
- The release is attempted once and retried once on failure ("при `false` ещё раз"); a persistent failure is exit 3, keeps `error` non-empty and still writes the files.
- The loop's first tick uses `dt = 0` (the ramp starts at the loop, not at the pre-roll); the ramp's total time is tick-independent (01-09), so this shifts nothing.
- The bus is read on every 8th loop tick (the selftest cadence) and the row of that tick carries `bus_raw`; `rate_hz = rows / (last tick start - pre-roll start)`.
- `metaJson` numbers use `%g`; an empty `out_path` records nothing (the CLI requires `--out` for `run`).
- `SessionEnv::realtime(SignalGuard &, int)` takes the one guard and the emergency descriptor, so `stop_requested`/`on_armed`/`on_released` bind to them without process-wide globals.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Plan bug] The tracer recording must satisfy `analyze.load_meta` (a grid of at least four speeds)**
- **Found during:** Task 1 (first run of verify 2)
- **Issue:** the plan's tracer uses one 3.0 rad/s speed (10 strokes, ids 0..9), but `analyze.load_meta` rejects a grid shorter than four speeds (`"speeds_rad_s" must list at least 4 speeds`). The plan's verify 2 and its must-have "их принимают `analyze.load_csv` и `load_meta`" cannot both hold with a single-speed recording.
- **Fix:** the tracer runs the first four grid speeds (3.0..4.5) -> 40 strokes, ids 0..39, and the verify's id range was refined from `range(10)` to `range(40)`; both loaders are exercised on the genuine recording and the check is not weakened (it now also covers the REST rows and the group transitions).
- **Files modified:** `ros2_ws/src/dog_bench/test/test_session.cpp`
- **Verification:** verify 2 exits 0 (`load_csv` + `load_meta` accept tracer.csv and its meta; ids 0..39; `stop_reason` completed); 14/14 gtests green.
- **Committed in:** c12d9e4 (test change); the verify refinement is recorded here (plan files are not modified)

**2. [Rule 1 - Plan bug] The `pca_error` injection (`failWritesFrom(30)`) also fails the release**
- **Found during:** Task 1 (test design, before the task commit)
- **Issue:** `FakePwm::failWritesFrom(30)` shares one attempt counter between `setPulseUs` and `release`, so once the 30th attempt fails the release retry fails too -> `released=false`, exit 3, contradicting the plan's blanket assertion for this case (released, `releases()>=1`, code 1).
- **Fix:** the `pca_error` case uses a test-local `PulseFaultPwm` whose pulse writes fail from the 30th attempt while the release stays healthy; the persistent-fault path (release failure -> exit 3) remains covered by `Session.ReleaseFailureGivesExit3`.
- **Files modified:** `ros2_ws/src/dog_bench/test/test_session.cpp`
- **Verification:** `EveryStopReasonReleases` pca_error case: `stop_reason` pca_error, released, `releases()>=1`, exit 1, files written.
- **Committed in:** c12d9e4

**3. [Plan-sanctioned refinement] Task 3's log regex refined to the real CI log format**
- **Found during:** Task 3 (first CI run 37372430526)
- **Issue:** the plan expected `colcon test-result --verbose` to print `test_session.gtest.xml: 14 tests, 0 errors, 0 failures, 0 skipped`; the real logs contain no per-file result lines. The test_session evidence there is the gtest suite output (`[==========] 14 tests from 3 test suites ran.` and `[  PASSED  ] 14 tests.`) plus the total `Summary: 261 tests, 0 errors, 0 failures, 0 skipped`.
- **Fix:** the regex was refined per the real log, keeping the number 14 and the zeros of errors, failures and skips — exactly as the plan allows ("регулярное выражение во втором verify разрешено уточнить по реальному журналу, сохранив число 14 и нули ошибок, сбоев и пропусков"). The check was not weakened: it adds the global zero-errors/failures/skips summary.
- **Files modified:** none (verify command refinement; the CI logs are git-ignored)
- **Verification:** the refined verify 2 exits 0 on run 37375137177 (both job logs pass all four greps).
- **Committed in:** n/a

---

**Total deviations:** 2 auto-fixed plan bugs + 1 plan-sanctioned verify refinement.
**Impact on plan:** the code side is unchanged by the fixes (only tests and the verify regex); the session's behavior matches the plan's spec (release on every exit, exit codes 0/1/2/3, the analyze.py contract). No scope creep — only the plan's own files were touched.

## Issues Encountered

- The session `python3` is the Hermes toolchain interpreter (3.14, no numpy); verify 2 ran with `/usr/bin/python3` (Python 3.13.5, numpy 2.4.3), as in plans 01-02/01-05/01-06. No repository change; CI keeps `python3`.
- `gh` calls in background shells must drop the placeholder `GITHUB_TOKEN` (it panics gh 2.67.0), so the task 3 commands ran under `env -u GITHUB_TOKEN`; the plan's command text is otherwise unchanged.
- First CI run 37372430526: three queued jobs (`robot parameter form`, `gazebo walk check (lyrical)`, `camera auto-calibration tool`) were cancelled after about 15 minutes without a runner — the same infrastructure issue recorded in `deferred-items.md` for 01-10. The awaited `build + test` jobs passed there too; the second dispatched run 37375137177 concluded with all 8 jobs success, so nothing was fixed (outside `dog_bench`).
- `ci_dispatch.sh` prints `warning: incomplete status ... retrying` on every poll while a run is in progress (the known 01-10 deferred parse quirk); cosmetic, the tool still exits 0 on the awaited jobs.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Plan 01-14 runs `servo_speed_test` on the robot with the owner's hand on the switch: `selftest` first, then `dry-run`, then `run`; the README section 'Запуск на роботе' is the procedure, and `analyze.py --plot` reads the recording.
- `CAL-17` stays open in REQUIREMENTS.md: `requirements.ready-ids` reports 0/1 ready (a sibling plan declaring CAL-17 has no summary yet), so the shared-ID gate correctly deferred completion.
- Backstops carried: SIGKILL/power loss (hand on the switch, plan 01-14), the H618 500 Hz assumption (selftest/rate on the robot) and the us_per_deg error (the D-11 margin, CAL-01 in Phase 7).

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-05*

## Self-Check: PASSED

- All 6 plan files exist on disk (session.hpp, session.cpp, test_session.cpp, servo_speed_test_main.cpp, CMakeLists.txt, README.md; checked with `[ -f ]`).
- Both task commits exist: `c12d9e4`, `efa2cfd` (plan ledger base `171fe9e`; measured `commits: 2` at SUMMARY write).
- All verifications pass: local gtests 6+5+6+9+6+14; verify 2 (`analyze.load_csv` + `load_meta` on the tracer recording) exit 0; task 2's 13-set CLI verify exit 0; CI run 37375137177: all 8 jobs success, no `src/dog_bench` warnings, `test_session` 14 tests with 0 errors, 0 failures, 0 skipped on Jazzy and Lyrical.
- `ros2_ws/src/dog_hardware` and `.github` are untouched (`git status --porcelain` empty); `origin` address appears in no log (`grep '://'` on push.txt/dispatch.txt is empty).
