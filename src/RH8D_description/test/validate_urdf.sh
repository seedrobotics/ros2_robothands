#!/usr/bin/env bash
# Expand the xacro for every side/coupling combination and validate the
# resulting URDF. Works both inside a colcon test run and standalone from the
# source tree (a temporary ament index entry is created if the package is not
# installed).
set -e
DIR="$(cd "$(dirname "$0")/.." && pwd)"

if ! python3 -c "from ament_index_python.packages import get_package_share_directory as g; g('rh8d_description')" 2>/dev/null; then
  FAKE="$(mktemp -d)"
  trap 'rm -rf "$FAKE"' EXIT
  mkdir -p "$FAKE/share/ament_index/resource_index/packages"
  touch "$FAKE/share/ament_index/resource_index/packages/rh8d_description"
  ln -s "$DIR" "$FAKE/share/rh8d_description"
  export AMENT_PREFIX_PATH="$FAKE${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
fi

status=0
for side in left right; do
  for mode in mimic independent; do
    out="$(mktemp --suffix=.urdf)"
    if xacro "$DIR/urdf/rh8d.urdf.xacro" side:="$side" finger_coupling:="$mode" > "$out" \
        && check_urdf "$out" > /dev/null; then
      echo "OK:   $side $mode"
    else
      echo "FAIL: $side $mode"
      status=1
    fi
    # gazebo + ros2_control variant must at least expand
    if xacro "$DIR/urdf/rh8d.urdf.xacro" side:="$side" finger_coupling:="$mode" use_gazebo:=true > /dev/null; then
      echo "OK:   $side $mode (gazebo)"
    else
      echo "FAIL: $side $mode (gazebo)"
      status=1
    fi
    rm -f "$out"
  done
done
exit $status
