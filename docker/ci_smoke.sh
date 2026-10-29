#!/bin/bash
# Proof that the arm64 image is whole (CAL-09): start the real launch with the mock
# backend and wait until the state machine answers. If libraries, message types,
# launch files or the python side are broken, this fails instead of the robot.
set -uo pipefail

# The entrypoint saw "bash ci_smoke.sh", not the launch arguments, so it skipped the
# hardware check (CI passes DOG_PREFLIGHT=off). Ask the same question here, with the
# arguments the launch will actually use: the mock backend needs no hardware.
python3 /ws/tools/preflight/preflight.py --args "backend:=mock"

ros2 launch dog_bringup robot.launch.py backend:=mock gamepad:=false web:=false \
    imu:=mock power:=mock profile:=bench > /tmp/launch.log 2>&1 &
pid=$!
trap 'kill -INT $pid 2>/dev/null || true' EXIT

for _ in $(seq 1 90); do
  if grep -q "locomotion ready at 50 Hz" /tmp/launch.log; then
    break
  fi
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "smoke: the launch died before the state machine came up"
    tail -40 /tmp/launch.log
    exit 1
  fi
  sleep 2
done

if ! grep -q "locomotion ready at 50 Hz" /tmp/launch.log; then
  echo "smoke: the state machine never started"
  tail -60 /tmp/launch.log
  exit 1
fi
echo "smoke: the launch reached the state machine"

# /dog/state is latched: a volatile subscriber (what `ros2 topic echo` uses) waits for
# the next change of state instead of reading the stored one. check_state.py asks for
# transient_local and prints the state, or says why it could not.
if ! timeout 120 python3 /ws/docker/check_state.py; then
  echo "smoke: the launch is up but /dog/state says nothing"
  ros2 topic info /dog/state 2>&1 | head -10
  tail -40 /tmp/launch.log
  exit 1
fi
exit 0
