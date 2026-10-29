#!/bin/bash
# The documented chain, end to end: a bag recorded the way the bench records it must feed
# the tool that turns it into thresholds (SAF-07). Mock stack, no hardware.
#
#   bash /ws/docker/ci_bag_check.sh
#
# What is checked: the record launch writes a bag, /dog/power is in it, and
# power_thresholds reads that bag without the columns a human used to copy by hand.
set -uo pipefail

# A missing glob must not decide the check: with both patterns named, `ls` fails as soon
# as one of them matches nothing, even when the bag is right there.
bag_files() {
  local saved
  saved=$(shopt -p nullglob || true)
  shopt -s nullglob
  local out=("$BAG"/*.db3* "$BAG"/*.mcap*)   # file compression adds .zstd
  eval "$saved"
  [ ${#out[@]} -gt 0 ]
}

python3 /ws/tools/preflight/preflight.py --args "backend:=mock"

BAG=/tmp/ci-bag
rm -rf "$BAG"
rm -f /tmp/robot.log /tmp/record.log /tmp/th.log

ros2 launch dog_bringup robot.launch.py backend:=mock gamepad:=false web:=false \
    imu:=mock power:=mock profile:=bench > /tmp/robot.log 2>&1 &
robot=$!
ros2 launch dog_bringup record.launch.py out:="$BAG" > /tmp/record.log 2>&1 &
rec=$!
trap 'kill -TERM $rec $robot 2>/dev/null || true' EXIT

for _ in $(seq 1 60); do
  if grep -q "locomotion ready at 50 Hz" /tmp/robot.log; then
    break
  fi
  sleep 2
done
if ! grep -q "locomotion ready at 50 Hz" /tmp/robot.log; then
  echo "bag: стек не поднялся"
  tail -30 /tmp/robot.log
  exit 1
fi

# A few seconds of a real walk: the rail only shows a walk's draw while the walk runs.
ros2 topic pub --once /dog/command std_msgs/String "{data: stand}" > /dev/null 2>&1
sleep 2
timeout 20 ros2 topic pub -r 5 --times 20 /dog/cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.05}}" > /dev/null 2>&1
sleep 2

# The recorder itself, not the launch: without a terminal on stdin it ignores SIGINT (its
# keyboard handler is off), and launch forwards a shutdown to its children as SIGINT. A
# human pressing Ctrl-C in a terminal signals the whole foreground group, which is why the
# bench never sees this. SIGTERM closes the bag properly.
pkill -TERM -f "ros2 bag record" 2>/dev/null || true
# The payload file appears at once; the summary and metadata.yaml are written when the
# recorder closes, and a bag read before that is a half-written file ("read failed").
for _ in $(seq 1 30); do
  if [ -f "$BAG/metadata.yaml" ]; then
    break
  fi
  sleep 1
done
kill -TERM "$rec" 2>/dev/null || true
if [ ! -f "$BAG/metadata.yaml" ]; then
  echo "bag: запись не закрылась (нет metadata.yaml)"
  tail -15 /tmp/record.log
  exit 1
fi
if ! bag_files; then
  echo "bag: запись не появилась"
  ls -la "$BAG" 2>&1 | head -5
  tail -20 /tmp/record.log
  exit 1
fi

# What is in the bag, for the log: the count is not a criterion (the tool below decides
# whether the recording is usable), but a silent bag is worth seeing before it is judged.
ros2 bag info "$BAG" > /tmp/baginfo.log 2>&1 || true
echo "bag: $(grep -c '/dog/' /tmp/baginfo.log) строк про /dog/ в описании записи"

if ! timeout 120 python3 /ws/tools/power_thresholds/power_thresholds.py --bag "$BAG" > /tmp/th.log 2>&1; then
  echo "bag: пороги из записи не посчитались"
  cat /tmp/th.log
  exit 1
fi
cat /tmp/th.log
if ! grep -q "overcurrent_a" /tmp/th.log; then
  echo "bag: инструмент не выдал пороги"
  head -20 /tmp/baginfo.log
  exit 1
fi
samples=$(sed -n 's/^# отсчётов: ток \([0-9]*\).*/\1/p' /tmp/th.log)
if [ -z "$samples" ] || [ "$samples" -lt 20 ]; then
  echo "bag: отсчётов тока ${samples:-0} — маловато для порогов"
  exit 1
fi
echo "bag: запись прочитана, отсчётов тока ${samples}, пороги посчитаны"
exit 0
