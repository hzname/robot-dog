#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /ws/install/setup.bash
# Hardware preflight before the launch: missing I2C bus, missing chip or a missing
# gamepad stops the robot here, with a message, instead of half a graph coming up
# and the operator wondering why nothing moves. `backend:=mock` passes by design,
# and DOG_PREFLIGHT=off is the escape hatch for bench work.
if [ "${DOG_PREFLIGHT:-auto}" != "off" ]; then
  if [ -f /ws/tools/preflight/preflight.py ]; then
    python3 /ws/tools/preflight/preflight.py --args "$*"
  else
    echo "preflight: скрипта в образе нет — пропускаю" >&2
  fi
fi

exec "$@"
