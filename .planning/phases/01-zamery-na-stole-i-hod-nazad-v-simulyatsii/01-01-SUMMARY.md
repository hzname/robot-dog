---
phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii
plan: 01
subsystem: infrastructure
tags: [bash, github-actions, gh-cli, gtest, g++, ci]

# Dependency graph
requires: []
provides:
  - "tools/ci_dispatch/ci_dispatch.sh: dispatch ci.yml on the phase branch, wait for selected jobs, download artifacts, never print an address"
  - "tools/local_gtest/run.sh: build and run one ROS-free core gtest with g++ -Werror, no colcon"
  - "remote branch gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii (upstream of the local branch)"
affects: [all remaining phase 01 plans that run CI or local core tests]

# Actuals (#2632) - pairs with the plan's estimate
actuals:
  tokens: 3369
  tasks: 3
  commits: 2

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "CI dispatched through one pinned helper; every gh call passes --repo hzname/robot-dog and all output is filtered through sed"
    - "ROS-free core tests built with a direct g++ command instead of colcon (D-24)"

key-files:
  created:
    - tools/ci_dispatch/ci_dispatch.sh
    - tools/local_gtest/run.sh
  modified:
    - .planning/STATE.md
    - .planning/ROADMAP.md
    - .planning/REQUIREMENTS.md

key-decisions:
  - "Replaced the tokenized origin URL with the canonical HTTPS URL of the repository so the already-configured gh credential helper (account hzname, scope repo) authenticates git and gh operations non-interactively; the old URL could only ever prompt for a password"
  - "ci_dispatch.sh validates --ref/-f/--run-id/--wait-job against whitelists before any network call; values are only ever passed as argv, never built into shell strings"
  - "run.sh rejects dog_hardware on purpose (phase 3 territory) and always builds with -Wall -Wextra -Wpedantic -Werror"
  - "Task 1 (branch creation) intentionally has no commit: it changes no files"

requirements-completed: [GAIT-02, GAIT-06]

# Coverage metadata (#1602)
coverage:
  - id: D1
    description: "Phase branch gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii exists locally and on GitHub, based on main, upstream configured"
    verification:
      - kind: other
        ref: "git branch --show-current && git merge-base --is-ancestor main HEAD && git ls-remote --heads origin gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii | grep refs/heads/"
        status: pass
    human_judgment: false
  - id: D2
    description: "tools/ci_dispatch/ci_dispatch.sh dispatches ci.yml, waits for selected jobs, downloads artifacts, and rejects unsafe arguments with exit 2"
    verification:
      - kind: other
        ref: "bash -n ci_dispatch.sh; --help | grep --wait-job; --dry-run --ref <branch> -f acceptance=true -f repeats=1 | grep 'gh workflow run ci.yml --repo hzname/robot-dog --ref ...'"
        status: pass
      - kind: other
        ref: "ci_dispatch.sh --dry-run --run-id 123456789 matches 'gh run view 123456789 --repo hzname/robot-dog' and does NOT print 'gh workflow run'"
        status: pass
      - kind: integration
        ref: "ci_dispatch.sh --run-id 36309182660 --wait-job 'build + test' --poll-sec 5 --timeout-min 5 -> exit 0 (2/2 jobs success, table printed)"
        status: pass
      - kind: other
        ref: "ci_dispatch.sh --dry-run --ref 'a;b' -> exit 2; -f 'x=$(id)' -> exit 2; negative grep for eval/get-url/git lines passes"
        status: pass
    human_judgment: false
  - id: D3
    description: "tools/local_gtest/run.sh builds and runs the five dog_control core gtests with -Werror and no colcon; binary in git-ignored ros2_ws/build/_local/"
    verification:
      - kind: unit
        ref: "tools/local_gtest/run.sh dog_control {kinematics,gait,crawl,greet,locomotion} -> all rc=0, PASSED, no FAILED"
        status: pass
      - kind: unit
        ref: "tools/local_gtest/run.sh dog_control locomotion --gtest_filter='Locomotion.JointSpeedsFitTheServos' -> PASSED 1 test"
        status: pass
      - kind: other
        ref: "run.sh dog_hardware x -> exit 2; missing test -> exit 2; git check-ignore ros2_ws/build/_local/dog_control/test_kinematics passes"
        status: pass
    human_judgment: false
  - id: D4
    description: "D-25 secrecy: the origin address (which embeds the token) is never printed by the new tooling or in the task log"
    verification: []
    human_judgment: true
    rationale: "Log hygiene across push and gh poll output is a review property (every call was piped through sed); automated greps cover only the script, so a human/verifier should confirm no address line exists in the task log"

# Metrics
duration: 13 min
completed: 2026-10-05
status: complete
---

# Phase 1 Plan 01: CI dispatch and local gtest helpers Summary

**Phase branch on GitHub plus two pinned helpers: ci_dispatch.sh (dispatch/watch/artifact-download for ci.yml) and run.sh (colcon-free g++ gtest runner with -Werror).**

## Performance

- **Duration:** 13 min
- **Started:** 2026-10-05T08:23:06Z
- **Completed:** 2026-10-05T08:36:00Z
- **Tasks:** 3
- **Files modified:** 2 created (plus STATE/ROADMAP/REQUIREMENTS updated at close-out)

## Accomplishments
- Phase branch `gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii` created from main and pushed to origin with upstream set; main untouched.
- `tools/ci_dispatch/ci_dispatch.sh`: one-command CI dispatch (workflow_dispatch), new-run discovery, job-filtered waiting, artifact download; exit codes 0/1/2/3; all gh output filtered so no address can reach a log.
- `tools/local_gtest/run.sh`: builds and runs one dog_control/dog_bench core gtest with `-Werror` in seconds, no colcon; all five existing dog_control tests pass.

## Task Commits

Each task was committed atomically:

1. **Task 1: create phase branch and push it** - no commit (branch-only task, no file changes; branch pushed at 13bb63f)
2. **Task 2: ci_dispatch helper** - `ce63322` (feat)
3. **Task 3: local gtest runner** - `0cf0836` (feat)

**Plan metadata:** `docs(01-01): complete ci_dispatch and local gtest helpers plan` (this commit: SUMMARY + STATE + ROADMAP + REQUIREMENTS)

## Files Created/Modified
- `tools/ci_dispatch/ci_dispatch.sh` - dispatch ci.yml via workflow_dispatch, wait for selected jobs or the whole run, download artifacts, filter all output, validate inputs before any network call
- `tools/local_gtest/run.sh` - compile core sources + one test with `g++ -std=c++17 -O1 -Werror`, run it, propagate its exit code
- `.planning/STATE.md`, `.planning/ROADMAP.md`, `.planning/REQUIREMENTS.md` - updated at plan close-out (metadata commit)

## Decisions Made
- Replaced the tokenized origin URL with the canonical HTTPS URL so the already-configured gh credential helper (account hzname, repo scope) can authenticate non-interactively (see Deviation 1).
- `--wait-job` values constrained to a conservative charset because the substring is embedded into a jq filter; `--ref`/`-f` use the plan's whitelists; values never enter a shell-evaluated string.
- Task 1 deliberately produces no commit: it changes no files (branch creation and push are its artifacts).

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking] origin could not authenticate non-interactively; URL replaced**
- **Found during:** Task 1 (push)
- **Issue:** `git push -u origin ...` died with "could not read Password ... No such device or address": the origin URL carried the token only in the username position, so git had no password and prompted (no tty); gh's credential helper correctly refuses a requested username that does not match the account. `git ls-remote --heads origin` failed the same way, blocking the task's own verification.
- **Fix:** `git remote set-url origin` to the canonical HTTPS URL of the repository (no token). Auth now flows through the already-configured gh helper (account hzname, scope repo). The stale token is no longer stored in `.git/config`, aligning with the PROJECT.md pending todo.
- **Files modified:** `.git/config` (local, not tracked)
- **Verification:** push succeeded; `git ls-remote --heads origin` lists the branch; all later gh calls succeed.
- **Committed in:** n/a (local config change)

**2. [Rule 1 - Bug] bash syntax error in --wait-job validation**
- **Found during:** Task 2, first verification run
- **Issue:** `[[ "$sub" =~ ^[A-Za-z0-9_ .:+()/-]+$ ]]` failed to parse (unquoted space and parens inside the pattern tokenized as separate words); the script aborted before validation.
- **Fix:** pattern changed to `^[A-Za-z0-9_\ .:+/-]+$` (escaped space, no parens); 'build + test' and 'gazebo walk check' accepted, quotes/`$()` rejected.
- **Files modified:** tools/ci_dispatch/ci_dispatch.sh
- **Verification:** `bash -n` passes; all --wait-job checks pass.
- **Committed in:** ce63322

**3. [Rule 1 - Bug] jq precedence in the poll filter**
- **Found during:** Task 2, first live run (poll loop never completed; every gh run view "failed")
- **Issue:** the two-output jq program `[meta...] | @tsv, (rows...)` was mis-parsed; the rows branch received the meta array and gojq raised "expected an object but got: array".
- **Fix:** wrapped each output branch in parentheses: `([meta...] | @tsv), (rows... | @tsv)`.
- **Files modified:** tools/ci_dispatch/ci_dispatch.sh
- **Verification:** live run then completed: 2/2 jobs table printed, exit 0.
- **Committed in:** ce63322

**4. [Rule 1 - Bug] `gh run view --json artifacts` is not a supported field (gh 2.67)**
- **Found during:** Task 2 download smoke test
- **Issue:** artifact counting via `gh run view --json artifacts` errored ("Unknown JSON field: artifacts") and the error text was parsed as a count.
- **Fix:** attempt `gh run download` directly; on failure match gh's "no valid artifacts" message to emit the required no-artifacts warning, otherwise warn with the detail. Download failure stays non-fatal; exit codes remain job-based.
- **Files modified:** tools/ci_dispatch/ci_dispatch.sh
- **Verification:** download smoke test: warnings printed, rc 0 (a gh-side quirk with a buildx-cache artifact surfaces as a warning, not an error).
- **Committed in:** ce63322

---

**Total deviations:** 4 auto-fixed (3 bugs, 1 blocking issue)
**Impact on plan:** All four were required for the deliverables to work non-interactively; no scope creep (no files beyond the plan's two).

## Issues Encountered
- Same as deviation 1 (origin auth) - resolved.
- gh CLI cannot extract the repository's `*.dockerbuild` buildx-cache artifact from older runs ("zip: not a valid zip file"); the script reports it as a warning and continues. gh-side limitation, not script logic.
- Task 1 has no commit by design (no file changes).

## User Setup Required
None - no external service configuration required.

## Next Phase Readiness
- Remaining phase plans can now dispatch `ci.yml` and run local core gtests with one command each.
- Note for CI jobs: `workflow_dispatch` on this branch runs the whole existing workflow (research measured ~15 min); the acceptance job arrives in a later plan.
- Owner action still pending (PROJECT.md): reissue the previously embedded GitHub token - it is no longer stored in `.git/config` after the origin URL fix.

---
*Phase: 01-zamery-na-stole-i-hod-nazad-v-simulyatsii*
*Completed: 2026-10-05*

## Self-Check: PASSED
