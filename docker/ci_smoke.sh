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

for _ in $(seq 1 45); do
  if timeout 5 ros2 topic echo --once /dog/state > /tmp/state.txt 2>/dev/null; then
    echo "smoke: /dog/state answered"
    sed -n 1,4p /tmp/state.txt
    exit 0
  fi
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "smoke: the launch died before the state appeared"
    tail -40 /tmp/launch.log
    exit 1
  fi
  sleep 2
done

echo "smoke: /dog/state never appeared"
ros2 node list 2>&1 | head -20
ros2 topic list 2>&1 | head -20
tail -60 /tmp/launch.log
exit 1
