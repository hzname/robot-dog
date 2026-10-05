---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 06
subsystem: bench-tooling
tags: [servo-speed, ina219, saturation, numpy, python38, cli, synthetic]

# Dependency graph
requires:
  - phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
    plan: 01
    provides: "phase branch and the tools/ conventions (offline Python tools run with /usr/bin/python3; pytest is the local check)"
provides:
  - "tools/servo_speed: numpy-only analyzer of INA219 current traces; v_sat from the kink of the distance between neighbour-speed traces, cross-checked by stroke duration (D-08)"
  - "CSV contract t_s,stroke_id,direction,cmd_us,shunt_raw,bus_raw and <csv>.meta.json (shunt_ohm, us_per_deg, speeds_rad_s, strokes_per_speed required; direction -1/0/+1 outside strokes; base window 0.15 s)"
  - "analyze.py result keys and flags (v_sat_rad_s, v_sat_status, v_dur_plateau_rad_s, v_dur_rad_s, agreement, servo_max_speed_rad_s, flag, plateau_current_a, noise_floor, bus_v_mean, per_direction, speeds_rad_s, distance_rel, eps_rel, strokes) and CLI exit codes 0/1/2"
  - "synth.py: deterministic generator (plain and ramp_timing per 01-09), write_run and a CLI; ramp_timing reproduces APPROACH, holds and REST"
affects: [01-12 (session writes this CSV and its meta), 01-13 (servo-speed CI job runs this pytest), 01-14 (owner checks the constants and the graph), phase 01 verification]

# Tech tracking
tech-stack:
  added: [numpy (already a repo dependency of the tools), matplotlib (optional, only for analyze.py --plot)]
  patterns:
    - "Offline tool: pure stdlib+numpy module with a thin argparse CLI; matplotlib imported inside the plot function only (AST test pins this)"
    - "Front-aligned trace comparison: traces are aligned at the current-rise front, so the 0..20 ms command desync does not move v_sat"
    - "eps scales itself: max(3*sigma_rep, 2 % of I_plateau) in relative units, so the shunt value cancels"
    - "A grid without saturation reports not_saturated + null, never the top grid speed; disagreement of the two methods takes the smaller value and raises v_sat_v_dur_disagree"
    - "Deterministic synthetic generator as the test fixture; lru_cache on synth_run keeps the suite at ~16 s"

key-files:
  created:
    - tools/servo_speed/analyze.py
    - tools/servo_speed/synth.py
    - tools/servo_speed/tests/conftest.py
    - tools/servo_speed/tests/test_analyze.py
    - tools/servo_speed/requirements.txt
    - tools/servo_speed/README.md
  modified: []

key-decisions:
  - "synth.py b_coef default 0.0001 (not the prototype 0.0002): at the sharper value the 1 ms front-phase jitter of the current edges inflated the ramp_timing saturated-pair distances past eps (6.0 rad/s, 8 mA, up direction 1.33x eps, kink lost); at 0.0001 the worst saturated pair is 0.62x eps over a seed sweep and v_dur stays 2-4 % low"
  - "analyze.py per-direction kernels compute I_plateau over that direction's own strokes; the mixed value made up/down plateau currents identical and hid the friction asymmetry (now up - down = 0.079 A)"
  - "The roundtrip test compares decisive fields (v_sat, status, agreement, servo_max, flag, strokes, speeds) exactly and continuous values with rel=1e-6, because the CSV keeps t_s at 1 us and cmd_us at 0.1 us"
  - "PRE_S = 0.15 s kept as specified: with hold_s 0.4 s the base window lies 0.25+ s after the previous command end, past the braking tail; stroke 0 after APPROACH is the one dropped stroke (rejected = 1 of 150 in the ramp_timing runs)"

patterns-established:
  - "The v_sat kink is the first grid speed after which all neighbour pairs stay below eps for at least three pairs; the grid top is never reported as saturation"
  - "cross_check(v_sat, v_dur_plateau): 15 % tolerance, kink wins on agreement, the smaller wins on disagreement with the v_sat_v_dur_disagree flag"

requirements-completed: [CAL-17]

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "Kink core + deterministic synthetic generator (tracer): analyze(*synth_run(6.0)) -> ok and v_sat within one grid step; 11 tests (tracer, find_saturation cases, seed determinism, plain and ramp_timing row contracts)"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "tools/servo_speed/tests/test_analyze.py#tracer/find_saturation/synth (11 tests); /usr/bin/python3 -m pytest -q tools/servo_speed/tests -k 'tracer or find_saturation or synth' -> 11 passed"
        status: pass
    human_judgment: false
  - id: D2
    description: "Cross-check, per-direction report and flags: 12 kink cases (v_max 3.5/4.5/6.0 on the default grid, 7.5 on the extended grid; noise 8/15/30 mA), 6 ramp_timing cases, 7 cross_check cases, soft kink, not_saturated, shunt-scale/jitter/drop invariance (46 tests total after task 2)"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "tools/servo_speed/tests/test_analyze.py#kink/ramp_timing/cross_check/shunt/jitter/drop; /usr/bin/python3 -m pytest -q tools/servo_speed/tests -> 46 passed (after task 2)"
        status: pass
    human_judgment: false
  - id: D3
    description: "CSV/meta I/O, CLI and plot: write_run -> load_csv roundtrip (ramp_timing run, direction 0 rows), line-numbered ValueError cases, exit codes 0/1/2, collision guard, PNG plot, AST contract (stdlib+numpy only, py3.8 syntax); demo chain synth.py -> analyze.py gives ok + agreement + v_sat 6.0"
    requirement: "CAL-17"
    verification:
      - kind: unit
        ref: "tools/servo_speed/tests/test_analyze.py (77 tests total); demo: synth.py --v-max 6.0 -> analyze.py -> v_sat=6.0 rad/s (ok), agreement: yes, servo.max_speed candidate = 6 rad/s"
        status: pass
    human_judgment: false
  - id: D4
    description: "Backstop (plan must_haves, verification: backstop): the analysis constants (0.6 s window, 5 ms smoothing, 0.3 level threshold, 7 % v_dur plateau tolerance) were tuned on synthetic traces only"
    verification: []
    human_judgment: true
    rationale: "The owner checks them on the real MG996R traces by the graph of plan 01-14 (the plan says so explicitly); the synthetic suite cannot prove the real trace shape"
  - id: D5
    description: "Backstop (plan must_haves, verification: backstop): the us_per_deg conversion error (protractor +-4 %) enters v_dur and the grid speeds; the analysis does not remove it"
    verification: []
    human_judgment: true
    rationale: "It is covered by the servo.margin gate (D-11): the measured number is applied with a wide margin after the owner approves it (plan 01-14)"

# Actuals (#2632)
actuals:
  tokens: 14723
  tasks: 3
  commits: 3
plan_head_before: 5fd8cb25c64827f5f476b9f6d59dda790a71beb1

# Metrics
duration: 23 min
completed: 2026-10-05
status: complete
---

# Phase 1 Plan 06: Servo speed from INA219 current traces (`tools/servo_speed`) Summary

**Offline numpy tool: front-aligned current traces give v_sat within one grid step on synthetic ramps (12 kink + 6 ramp_timing cases), cross-checked by stroke duration (15 % tolerance, the smaller wins), with a strict CSV/meta contract and CLI exit codes 0/1/2 — 77 tests green.**

## Performance

- **Duration:** 23 min
- **Started:** 2026-10-05T17:15:44Z (first recorded clock reading)
- **Completed:** 2026-10-05T17:38:31Z
- **Tasks:** 3
- **Files modified:** 6 created (plus the four .planning close-out files)

## Accomplishments

- `synth.py`: deterministic generator (`np.random.default_rng(seed)`) of INA219 current traces for the bench ramp; plain mode (strokes back to back with a 0.3 s pre-roll) and `ramp_timing=True` which repeats the 01-09 sequence exactly (APPROACH hold + ramp at speeds[0], strokes with a hold_s hold each, REST rest_s between groups); per-stroke command delay U(0, 20 ms), P controller with v_max/a_max caps, current `i_hold + c*[|vel|>0.05] + a_coef*|vel| + b_coef*|acc|` low-passed at tau_i and noised at noise_a; shunt/bus registers quantized as the sensor reads them. `write_run` and a CLI (`--v-max`, `--out`, `--seed`, `--noise-a`, `--shunt-scale`).
- `analyze.py`: `find_saturation` (three consecutive pairs rule; all-below -> speeds[0]; exact-eps counts as below; length mismatch -> ValueError), `extract_strokes` (front-aligned traces: base from the previous hold tail, 95th-percentile level, threshold max(0.3*L, 6*sigma_b), quiet-end detection), the kink core (`eps = max(3*sigma_rep, 2 % of I_plateau)`, relative units), `plateau_duration`, `cross_check` (15 % tolerance, smaller wins, `v_sat_v_dur_disagree`), per-direction kernels, and the full result dict with flags in the specified order.
- Cross-checks on the synthetic matrix behave as designed: kink found at the first grid speed not below v_max in all 18 parametrized cases (noise 8/15/30 mA), v_dur_plateau 2-4 % below v_max, agreement True with `flag: None`; grid without saturation (12.0 rad/s; 7.5 rad/s on the default grid) gives `not_saturated`, null values and the flag; the soft-kink case (kp=100, a_max=3000) gives agreement False with the smaller candidate.
- Invariance proven by tests: shunt scale 0.1/10 keeps v_sat and v_dur_plateau (rel <= 1e-3), plateau_current_a scales within 2 %, noise_floor/distance_rel within 10 %; start desync 0 vs 20 ms keeps v_sat and moves v_dur_plateau <= 2 %; 10 % dropped rows shift the v_sat index by 0; bus_v_mean = 5.94 V in the 5.90..5.98 window; plateau current 0.460 A (expected 0.46 +-0.03) with up - down = 0.079 A.
- `load_csv`/`load_meta`/`validate_meta` with line-numbered ValueErrors, the direction rule (+1/-1 inside a stroke; -1, 0, +1 outside), MAX_ROWS 5e6, bool-is-not-a-number; `analyze` validates the meta as its first line. CLI: `--csv/--meta/--out/--plot`, realpath collision guard, exit 0 (ok) / 1 (not_saturated) / 2 (input error); JSON indent 2 sort_keys; the plot (log-Y distance graph + duration "hockey stick") writes a PNG with matplotlib imported inside the function.
- README (Russian): install, offline check, input tables for CSV and `.meta.json`, method + result keys, how to read the result (D-11 approval, disagreement, not_saturated as a lower bound, the >= 6 rad/s PWM-frame scatter), D-09 measurement conditions, tests.

## Task Commits

Each task was committed atomically:

1. **Task 1 (tracer): synth_run -> analyze -> v_sat core** - `0314d12` (feat)
2. **Task 2: duration cross-check, per-direction report, flags** - `998675b` (feat)
3. **Task 3: CSV/meta input, CLI, plot, README** - `2baaecc` (feat)

**Plan metadata:** `docs(01-06): complete servo speed analysis tool plan` (this commit: SUMMARY + STATE + ROADMAP + REQUIREMENTS)

## Files Created/Modified

- `tools/servo_speed/analyze.py` - the analyzer, CSV/meta I/O, CLI and plot (stdlib + numpy; matplotlib only inside `plot_result`)
- `tools/servo_speed/synth.py` - deterministic generator, `write_run`, CLI
- `tools/servo_speed/tests/conftest.py` - sys.path bootstrap (same pattern as tools/autocal)
- `tools/servo_speed/tests/test_analyze.py` - 77 tests: tracer, find_saturation, contracts, kink/ramp matrices, cross_check, invariance, I/O, CLI, AST contract
- `tools/servo_speed/requirements.txt` - `numpy>=1.24` + note that matplotlib is only for `--plot`
- `tools/servo_speed/README.md` - format, method, reading the result, D-09 conditions, tests

## Decisions Made

- `b_coef` default 0.0001 instead of the prototype 0.0002 (reason in the synth.py docstring): the plan allows adjusting kp/a_max/tau_i/b_coef within reason when the analyzer tests fail; the change keeps the kink sharp and the saturated pairs well below eps.
- Per-direction kernels compute their own I_plateau (over that direction's strokes only) - the mixed value was a bug (see Deviations).
- The CSV/meta names, result keys, flags and CLI exit codes are the fixed contract for plans 01-12, 01-13 and 01-14.
- The roundtrip comparison: decisive fields exactly, continuous values at rel=1e-6 (CSV quantization 1 us / 0.1 us).

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] Per-direction I_plateau averaged over both directions**
- **Found during:** Task 2 (the `per_direction` test showed up and down plateau currents identical)
- **Issue:** `_kernel` computed `I_plateau` over all strokes of a speed regardless of `dirs`, so the per-direction report could not show the friction asymmetry (up - down = 0.079 A was hidden) and per-direction eps used the wrong plateau.
- **Fix:** the means now use only strokes with `direction in dirs`; the per-direction values became 0.500 A (up) / 0.421 A (down).
- **Files modified:** `tools/servo_speed/analyze.py`
- **Verification:** the full suite passes (46 tests at that point); the per_direction test asserts up >= down + 0.04.
- **Committed in:** 998675b

**2. [Plan-sanctioned model adjustment] synth.py b_coef 0.0002 -> 0.0001**
- **Found during:** Task 1/2 (pre-validating the ramp_timing case matrix with the prototype constants)
- **Issue:** with b_coef 0.0002 the 1 ms front-phase jitter of the current edges inflated the distances between saturated speeds in the ramp_timing run (6.0 rad/s, noise 8 mA, up direction: worst pair 1.33x eps), so `v_sat` came out None for one direction and `direction_mismatch` fired; the ramp test matrix cannot pass with it.
- **Fix:** b_coef default 0.0001 (kp/a_max/tau_i unchanged); worst saturated pair 0.62x eps over a seed sweep, kink still sharp, v_dur 2-4 % low. The plan explicitly allows changing these constants within reason with the reason recorded in the docstring; thresholds and test tolerances were not loosened.
- **Files modified:** `tools/servo_speed/synth.py` (docstring documents the reason)
- **Verification:** full case matrix green (18 kink/ramp cases, flag None); seed sweep in the analysis above.
- **Committed in:** 0314d12

---

**Total deviations:** 1 bug fix + 1 plan-sanctioned model adjustment.
**Impact on plan:** both were required for the plan's own tests; no scope creep - only the plan's files were touched, no thresholds or tolerances loosened.

## Issues Encountered

- The session `python3` is the Hermes toolchain interpreter (3.14, no numpy/pytest); every suite and CLI acceptance command ran with `/usr/bin/python3` (Python 3.13.5, numpy 2.4.3, pytest 9.0.3, matplotlib 3.11.1), as in plans 01-02 and 01-03. No repository change; CI keeps `python -m pytest`.
- The CSV roundtrip changes continuous values in the last digits (t_s is kept at 1 us, cmd_us at 0.1 us); the roundtrip test compares decisive fields exactly and the rest at rel=1e-6.
- No `colcon`/`ros2` locally (D-24): the CI job for this suite is added by plan 01-13; the tool itself needs only Python 3.8+ and numpy.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Plan 01-12 can write the session CSV and `<csv>.meta.json` to this exact contract and analyze the robot recording with `analyze.py`; plan 01-13 adds the `servo-speed` pytest job; plan 01-14 gets the graph (`--plot`) and checks the constants on the real traces.
- `CAL-17` stays open in REQUIREMENTS.md: `requirements.ready-ids` reports 0/1 ready (sibling plans declaring CAL-17 have no summary yet), so the shared-ID gate correctly deferred completion.
- Backstops carried: the real MG996R trace shape (constants checked by the owner, 01-14) and the `us_per_deg` error covered by the servo.margin gate (D-11).

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-05*

## Self-Check: PASSED

- All 6 key-files.created exist on disk (checked with `[ -f ]`).
- All three commits exist: `0314d12`, `998675b`, `2baaecc` (ledger base `5fd8cb2`; measured `commits: 3`).
- The full suite passes: `/usr/bin/python3 -m pytest -q tools/servo_speed/tests` -> `77 passed` (no failures, no skips); the demo chain synth.py -> analyze.py -> `v_sat = 6 rad/s (ok)`, `agreement: yes`, `servo.max_speed candidate = 6 rad/s`.
- `git status --short -- tools ros2_ws .github` shows nothing outside `tools/servo_speed/` (empty after the task commits).
