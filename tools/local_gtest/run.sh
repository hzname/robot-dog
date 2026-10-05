#!/usr/bin/env bash
#
# run.sh - build and run one gtest of a ROS-free core without colcon.
#
# Why this exists: colcon and ros2 are not installed on every workstation, and
# long simulations run only in GitHub Actions (D-24). This script compiles the
# ROS-independent core of a package together with a single test file using g++,
# so core logic can be checked in seconds.
#
# Usage:
#   tools/local_gtest/run.sh <package> <test> [gtest args...]
#
#   <package>  dog_control or dog_bench (dog_hardware belongs to phase 3 and is
#              deliberately not buildable here)
#   <test>     ros2_ws/src/<package>/test/test_<test>.cpp
#
# Example:
#   tools/local_gtest/run.sh dog_control locomotion \
#       --gtest_filter='Locomotion.JointSpeedsFitTheServos'
#
# Core sources are all src/*.cpp except *_node.cpp and *_main.cpp (nodes and
# CLI executables carry main()). The binary goes to ros2_ws/build/_local/<package>/
# which git ignores (ros2_ws/build/). Builds keep -Wall -Wextra -Wpedantic
# -Werror: warnings are fixed in code, never suppressed (.claude/CLAUDE.md).
#
# Full node builds, launch tests and simulations run in GitHub Actions, not
# here.
set -euo pipefail

usage() {
  echo "usage: $0 <package> <test> [gtest args...]" >&2
  echo "  package: dog_control | dog_bench" >&2
}

if [ $# -lt 2 ]; then
  usage
  exit 2
fi

PACKAGE="$1"
TEST="$2"
shift 2

case "$PACKAGE" in
  dog_control|dog_bench) ;;
  *)
    echo "error: package '$PACKAGE' is not supported here (dog_control, dog_bench); dog_hardware belongs to phase 3" >&2
    exit 2
    ;;
esac

if ! [[ "$TEST" =~ ^[A-Za-z0-9_]+$ ]]; then
  echo "error: invalid test name '$TEST'" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PKG_DIR="$REPO_ROOT/ros2_ws/src/$PACKAGE"
TEST_FILE="$PKG_DIR/test/test_$TEST.cpp"

if [ ! -f "$TEST_FILE" ]; then
  echo "error: no such test file: $TEST_FILE" >&2
  exit 2
fi

shopt -s nullglob
sources=()
for src in "$PKG_DIR"/src/*.cpp; do
  case "$src" in
    *_node.cpp|*_main.cpp) continue ;;
  esac
  sources+=("$src")
done
shopt -u nullglob

if [ "${#sources[@]}" -eq 0 ]; then
  echo "error: no core sources in $PKG_DIR/src" >&2
  exit 2
fi

BUILD_DIR="$REPO_ROOT/ros2_ws/build/_local/$PACKAGE"
mkdir -p "$BUILD_DIR"
BIN="$BUILD_DIR/test_$TEST"

g++ -std=c++17 -O1 -Wall -Wextra -Wpedantic -Werror \
  -I "$PKG_DIR/include" \
  "${sources[@]}" "$TEST_FILE" \
  -lgtest -lgtest_main -pthread \
  -o "$BIN"

cd "$REPO_ROOT"
rc=0
"$BIN" "$@" || rc=$?
exit "$rc"
