# Deferred Items — Phase 01

Out-of-scope discoveries logged during plan 01-10 execution (not fixed here, per the executor scope boundary).

## `tools/ci_dispatch/ci_dispatch.sh` — run-state parse shifts fields while the run is in progress

- **Found during:** 01-10 Task 4 (CI dispatch).
- **Issue:** the waiter reads `gh run view --json status,conclusion` as one TSV line. While a run is in progress `conclusion` is an empty string and `(.conclusion // "-")` does not substitute for it; the resulting double tab collapses under bash `read`, so the loop prints `warning: incomplete status` on every poll and only resolves once the run concludes. After a `gh run rerun --failed`, the tool can also decide from the pre-rerun attempt if the run had already concluded before the rerun (seen here: it reported `build + test (lyrical) cancelled` although the rerun attempt had made it `success`).
- **Suggested fix:** map empty conclusions explicitly, e.g. `(.conclusion // "-" | if . == "" then "-" else . end)`, or parse the JSON fields individually instead of a TSV line.
- **Owner:** the tool shipped with plan 01-01; not modified in 01-10 (outside the plan's files).

## GitHub Actions cancelled queued jobs after ~15 min without a runner

- **Found during:** 01-10 Task 4; two events, 19:35:31Z and 20:00:13Z, each ~15 min 0 s after the jobs were queued.
- **Effect:** four jobs concluded `cancelled` without ever getting a runner (`runner_id = 0`, no steps): `build + test (lyrical)` in the first event; `robot image (arm64)` and `robot parameter form (tools/robot_setup)` in the second. The two walk checks and `build + test (jazzy)` stayed green.
- **Workaround used:** `gh run rerun --failed` (run attempt 2) — `build + test (lyrical)` passed; `ci_dispatch.sh --run-id …` then exited 0 with all four awaited jobs success.
- **Watch:** if long-queued jobs keep being cancelled, plan 01-13's acceptance matrix (a long run on a busy runner) may need the same rerun handling.

## `gh run download` aborts on the `robot image` dockerbuild artifact and fetches nothing

- **Found during:** 01-13 Task 3/4 (trial and baseline artifact downloads).
- **Issue:** `gh run download <run> --dir DIR` (no filter) stops at the artifact `hzname~robot-dog~<id>.dockerbuild` (`error extracting zip archive: zip: not a valid zip file`) and leaves the target directory empty — so `ci_dispatch.sh --download` delivers no acceptance artifacts for any full `ci.yml` run. Seen on runs 37414394613, 37416318962, 37416429124 (the tool prints `warning: artifact download failed` and still exits 0).
- **Workaround used:** `gh run download <run> --pattern 'acceptance-*' --dir DIR` downloads only the needed artifacts; all three runs' acceptance artifacts were fetched this way.
- **Suggested fix:** in `ci_dispatch.sh`, when `--download` is set, list the run artifacts (`gh api .../artifacts`) and fetch them one by one with `--name`, warning and skipping the failed ones instead of aborting.
- **Owner:** tool shipped with plan 01-01; will also hit plans 01-16/01-17 downloads (`post-numbers`, `post-fix`, `final5`, `final10`).
