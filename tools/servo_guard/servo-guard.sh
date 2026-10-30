#!/usr/bin/env bash
#
# servo-guard.sh - keep the PCA9685 outputs off when the driver is not running.
#
# The driver drops its outputs on SIGTERM (relax_on_exit), on its own faults and
# after three bad bus frames. What it cannot survive is kill -9, an OOM kill or a
# crashed container: those skip every handler, the chip keeps the last pulse and
# the servos hold their load until the battery dies. This guard looks at the
# container from the host and calls `pca9685_probe off` when it disappears
# (SAF-16, D-07).
#
# Two ways to run it, both calling the same script:
#   watch  - systemd service, polls the container once a second (catches OOM/kill)
#   off    - ExecStopPost of a container unit, runs right after a normal stop
#
# Usage: servo_guard.sh [watch|off]
set -uo pipefail

COMPOSE_DIR="${SERVO_GUARD_COMPOSE_DIR:-/home/robot-dog}"
SERVICE="${SERVO_GUARD_SERVICE:-robot}"
COOLDOWN="${SERVO_GUARD_COOLDOWN:-5}"          # [s] no two drops closer than this
STATE="${SERVO_GUARD_STATE:-/run/servo-guard.last}"

log() { echo "$(date -Iseconds) servo-guard: $*" >&2; }

compose() { docker compose -f "$COMPOSE_DIR/docker-compose.yml" "$@"; }

running() {
  compose ps --status running --services 2>/dev/null | grep -qx "$SERVICE"
}

drop_outputs() {
  local now last
  now=$(date +%s)
  last=$(cat "$STATE" 2>/dev/null || echo 0)
  if [ $((now - last)) -lt "$COOLDOWN" ]; then
    log "cooldown: outputs were dropped ${last} s ago"
    return 0
  fi
  echo "$now" > "$STATE" 2>/dev/null || true
  if compose run --rm --no-deps "$SERVICE" \
       ros2 run dog_hardware pca9685_probe off; then
    log "outputs dropped"
  else
    log "failed to drop the outputs - check the I2C bus and the chip address"
    return 1
  fi
}

case "${1:-watch}" in
  off)
    drop_outputs
    ;;
  watch)
    log "watching '$SERVICE' in $COMPOSE_DIR"
    seen=0
    while true; do
      if running; then
        seen=1
      elif [ "$seen" = 1 ]; then
        log "container '$SERVICE' is gone"
        drop_outputs || true
        seen=0
      fi
      sleep 1
    done
    ;;
  *)
    echo "usage: $0 [watch|off]" >&2
    exit 2
    ;;
esac
