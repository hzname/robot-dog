#!/usr/bin/env bash
#
# ci_dispatch.sh - dispatch the GitHub Actions workflow ci.yml on a branch,
# wait for the run, and download its artifacts.
#
# Why this exists: long simulations run only in GitHub Actions (D-24), so every
# CI task in this phase goes through one command instead of a hand-made chain
# of gh calls. The repository is pinned to hzname/robot-dog and every gh call
# passes --repo explicitly. The remote origin address is never read and never
# printed; all gh output is filtered so that no address can reach the log.
#
# Workflow inputs (-f) are passed to gh as separate argv entries and are never
# interpolated into a string the shell executes, so a value cannot become a
# command (ASVS V5). Values are validated before any network call.
#
# Example (phase acceptance run):
#   tools/ci_dispatch/ci_dispatch.sh \
#       --ref gsd/phase-1-zamery-na-stole-i-hod-nazad-v-simulyatsii \
#       -f acceptance=true -f repeats=5 \
#       --wait-job acceptance --download out/
#
# Push the branch commits to the remote before dispatching: the workflow runs
# the code GitHub has, not the local working tree.
#
# Exit codes: 0 - all awaited jobs succeeded; 1 - at least one did not;
#             2 - invalid arguments; 3 - timeout.
set -euo pipefail

REPO="hzname/robot-dog"
WORKFLOW="ci.yml"

usage() {
  cat <<'EOF'
ci_dispatch.sh - dispatch and watch the CI workflow in hzname/robot-dog.

Usage:
  ci_dispatch.sh --ref REF [options]
  ci_dispatch.sh --run-id ID [options]

Options:
  --ref REF          branch/ref to run ci.yml on (required unless --run-id)
  -f key=value       workflow input, repeatable; only with --ref
  --wait-job SUBSTR  wait for jobs whose name contains SUBSTR, repeatable;
                     without it the whole run is awaited
  --download DIR     download the run artifacts into DIR
  --run-id ID        attach to an existing run: no dispatch, only wait/download
  --timeout-min N    overall wait timeout in minutes (default 120)
  --poll-sec N       poll interval in seconds (default 30)
  --dry-run          print the gh commands that would run, execute nothing
  --help             show this help

Exit codes: 0 all awaited jobs succeeded; 1 at least one did not;
2 invalid arguments; 3 timeout.
EOF
}

die() {
  echo "error: $*" >&2
  exit 2
}

# All gh output goes through this filter: an address must never reach the log.
ghq() {
  gh "$@" 2>&1 | sed -E 's#https?://[^ ]+#<url-hidden>#g'
}

REF=""
RUN_ID=""
DOWNLOAD_DIR=""
TIMEOUT_MIN=120
POLL_SEC=30
DRY_RUN="false"
declare -a INPUTS=()
declare -a WAIT_JOBS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --ref)
      [ $# -ge 2 ] || die "--ref requires a value"
      REF="$2"; shift 2 ;;
    -f)
      [ $# -ge 2 ] || die "-f requires a key=value argument"
      INPUTS+=("$2"); shift 2 ;;
    --wait-job)
      [ $# -ge 2 ] || die "--wait-job requires a value"
      WAIT_JOBS+=("$2"); shift 2 ;;
    --download)
      [ $# -ge 2 ] || die "--download requires a directory"
      DOWNLOAD_DIR="$2"; shift 2 ;;
    --run-id)
      [ $# -ge 2 ] || die "--run-id requires a value"
      RUN_ID="$2"; shift 2 ;;
    --timeout-min)
      [ $# -ge 2 ] || die "--timeout-min requires a value"
      TIMEOUT_MIN="$2"; shift 2 ;;
    --poll-sec)
      [ $# -ge 2 ] || die "--poll-sec requires a value"
      POLL_SEC="$2"; shift 2 ;;
    --dry-run)
      DRY_RUN="true"; shift ;;
    --help|-h)
      usage; exit 0 ;;
    *)
      die "unknown option: $1" ;;
  esac
done

# ---- argument validation, before any network call ---------------------------

if [ -n "$REF" ] && ! [[ "$REF" =~ ^[A-Za-z0-9._/-]+$ ]]; then
  die "invalid --ref '$REF' (allowed: letters, digits, dot, underscore, slash, dash)"
fi

for kv in "${INPUTS[@]}"; do
  if ! [[ "$kv" =~ ^[a-z_][a-z0-9_]*=[A-Za-z0-9._,:-]*$ ]]; then
    die "invalid -f value '$kv' (expected key=value, key ^[a-z_][a-z0-9_]*, value from [A-Za-z0-9._,:-])"
  fi
done

if [ -n "$RUN_ID" ]; then
  if ! [[ "$RUN_ID" =~ ^[0-9]+$ ]]; then
    die "invalid --run-id '$RUN_ID' (digits only)"
  fi
  if [ "${#INPUTS[@]}" -gt 0 ]; then
    die "-f inputs cannot be used with --run-id (nothing is dispatched)"
  fi
fi

if [ -z "$RUN_ID" ] && [ -z "$REF" ]; then
  die "either --ref (to dispatch) or --run-id (to attach) is required"
fi

for sub in "${WAIT_JOBS[@]}"; do
  if ! [[ "$sub" =~ ^[A-Za-z0-9_\ .:+/-]+$ ]]; then
    die "invalid --wait-job value '$sub' (unexpected characters)"
  fi
done

if ! [[ "$TIMEOUT_MIN" =~ ^[0-9]+$ ]] || [ "$TIMEOUT_MIN" -lt 1 ]; then
  die "invalid --timeout-min '$TIMEOUT_MIN' (a positive integer of minutes)"
fi
if ! [[ "$POLL_SEC" =~ ^[0-9]+$ ]] || [ "$POLL_SEC" -lt 1 ]; then
  die "invalid --poll-sec '$POLL_SEC' (a positive integer of seconds)"
fi

# ---- dry run: print the planned commands, execute nothing -------------------

if [ "$DRY_RUN" = "true" ]; then
  if [ -n "$RUN_ID" ]; then
    echo "gh run view $RUN_ID --repo $REPO --json jobs,status,conclusion"
    if [ -n "$DOWNLOAD_DIR" ]; then
      echo "gh run download $RUN_ID --repo $REPO --dir $DOWNLOAD_DIR"
    fi
  else
    echo "gh run list --repo $REPO --workflow $WORKFLOW --branch $REF --event workflow_dispatch --json databaseId"
    launch="gh workflow run $WORKFLOW --repo $REPO --ref $REF"
    for kv in "${INPUTS[@]}"; do
      launch="$launch -f $kv"
    done
    echo "$launch"
    echo "gh run view <RUN_ID> --repo $REPO --json jobs,status,conclusion"
    if [ -n "$DOWNLOAD_DIR" ]; then
      echo "gh run download <RUN_ID> --repo $REPO --dir $DOWNLOAD_DIR"
    fi
  fi
  exit 0
fi

# ---- dispatch (unless --run-id) ---------------------------------------------

list_run_ids() {
  ghq run list --repo "$REPO" --workflow "$WORKFLOW" --branch "$REF" \
    --event workflow_dispatch --limit 100 --json databaseId --jq '.[].databaseId'
}

if [ -z "$RUN_ID" ]; then
  if ! before="$(list_run_ids)"; then
    echo "error: could not list the existing runs of '$REF'" >&2
    exit 1
  fi

  echo "dispatching $WORKFLOW on branch '$REF'"
  if [ "${#INPUTS[@]}" -gt 0 ]; then
    for kv in "${INPUTS[@]}"; do
      echo "  input $kv"
    done
  fi
  if ! ghq workflow run "$WORKFLOW" --repo "$REPO" --ref "$REF" "${INPUTS[@]}"; then
    echo "error: could not dispatch the workflow" >&2
    exit 1
  fi

  echo "waiting for the new run to appear (up to 120 s)"
  new_id=""
  attempts=0
  while [ "$attempts" -lt 40 ]; do
    sleep 3
    attempts=$((attempts + 1))
    if ! current="$(list_run_ids)"; then
      echo "warning: listing runs failed, retrying" >&2
      continue
    fi
    for id in $current; do
      if ! grep -qx "$id" <<< "$before"; then
        new_id="$id"
        break
      fi
    done
    if [ -n "$new_id" ]; then
      break
    fi
  done
  if [ -z "$new_id" ]; then
    echo "error: no new workflow_dispatch run appeared for '$REF' within 120 s" >&2
    exit 3
  fi
  RUN_ID="$new_id"
fi

echo "RUN_ID=$RUN_ID"

# ---- wait for the selected jobs -------------------------------------------------

# jq filter over the jobs selected by --wait-job (all jobs when none given).
sub_filter="true"
if [ "${#WAIT_JOBS[@]}" -gt 0 ]; then
  sub_filter=""
  for sub in "${WAIT_JOBS[@]}"; do
    if [ -z "$sub_filter" ]; then
      sub_filter="(.name | contains(\"$sub\"))"
    else
      sub_filter="$sub_filter or (.name | contains(\"$sub\"))"
    fi
  done
fi

JQ_PROG="([.status, (.conclusion // \"-\"), ([.jobs[] | select($sub_filter)] | length), ([.jobs[] | select($sub_filter) | select(.status == \"completed\")] | length), ([.jobs[] | select($sub_filter) | select(.status == \"completed\" and .conclusion != \"success\")] | length)] | @tsv), (.jobs[] | select($sub_filter) | [.name, .status, (.conclusion // \"-\")] | @tsv)"

start_epoch=$(date +%s)
deadline_epoch=$((start_epoch + TIMEOUT_MIN * 60))

while :; do
  now_epoch=$(date +%s)
  if [ "$now_epoch" -gt "$deadline_epoch" ]; then
    echo "error: timed out after ${TIMEOUT_MIN} min waiting for run $RUN_ID" >&2
    exit 3
  fi

  if ! state="$(ghq run view "$RUN_ID" --repo "$REPO" --json jobs,status,conclusion --jq "$JQ_PROG")"; then
    echo "warning: gh run view failed for run $RUN_ID, retrying in ${POLL_SEC}s" >&2
    sleep "$POLL_SEC"
    continue
  fi

  meta_line="${state%%$'\n'*}"
  rows=""
  if [[ "$state" == *$'\n'* ]]; then
    rows="${state#*$'\n'}"
  fi
  if [ -z "$meta_line" ]; then
    echo "warning: empty status for run $RUN_ID, retrying in ${POLL_SEC}s" >&2
    sleep "$POLL_SEC"
    continue
  fi

  IFS=$'\t' read -r run_status run_concl sel_total sel_done sel_bad <<< "$meta_line"
  if [ -z "$run_status" ] || [ -z "$sel_total" ] || [ -z "$sel_done" ] || [ -z "$sel_bad" ]; then
    echo "warning: incomplete status for run $RUN_ID, retrying in ${POLL_SEC}s" >&2
    sleep "$POLL_SEC"
    continue
  fi

  if [ "$sel_total" -gt 0 ] && [ "$sel_done" -eq "$sel_total" ]; then
    break
  fi
  if [ "$sel_total" -eq 0 ] && [ $((now_epoch - start_epoch)) -ge 600 ]; then
    echo "error: no jobs matching the --wait-job filters after 10 min" >&2
    exit 3
  fi

  echo "waiting: ${sel_done}/${sel_total} awaited jobs completed (run $run_status)"
  sleep "$POLL_SEC"
done

# ---- report -----------------------------------------------------------------

echo "run $RUN_ID: ${sel_done}/${sel_total} awaited jobs completed"
printf '%-50s %-12s %s\n' "JOB" "STATUS" "CONCLUSION"
if [ -n "$rows" ]; then
  while IFS= read -r row; do
    IFS=$'\t' read -r job_name job_status job_concl <<< "$row"
    printf '%-50s %-12s %s\n' "$job_name" "$job_status" "$job_concl"
  done <<< "$rows"
fi

# ---- artifacts ---------------------------------------------------------------

if [ -n "$DOWNLOAD_DIR" ]; then
  mkdir -p "$DOWNLOAD_DIR"
  if ! dl_out="$(ghq run download "$RUN_ID" --repo "$REPO" --dir "$DOWNLOAD_DIR")"; then
    if grep -qi 'no valid artifacts' <<< "$dl_out"; then
      echo "warning: run $RUN_ID has no artifacts to download" >&2
    else
      echo "warning: artifact download failed for run $RUN_ID" >&2
      printf '%s\n' "$dl_out" >&2
    fi
  fi
fi

# ---- exit code ---------------------------------------------------------------

if [ "$sel_bad" -gt 0 ]; then
  exit 1
fi
exit 0
