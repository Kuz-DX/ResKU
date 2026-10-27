#!/usr/bin/env bash
# Run the real manual arm from ResKU without inherited dolbotZ ROS overlays.
set -euo pipefail

_resku_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec env -i \
  HOME="$HOME" USER="${USER:-$(id -un)}" TERM="${TERM:-dumb}" \
  PATH=/usr/local/bin:/usr/bin:/bin RESKU_WORKSPACE="$_resku_workspace" \
  /bin/bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source "$RESKU_WORKSPACE/install/local_setup.bash"
    set -u
    export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=0
    for package in dynamixel_hardware_interface tool_manipulator_bringup; do
      prefix="$(ros2 pkg prefix "$package")"
      expected="$RESKU_WORKSPACE/install/$package"
      if [[ "$prefix" != "$expected" ]]; then
        printf "Wrong %s prefix: %s (expected %s)\n" "$package" "$prefix" "$expected" >&2
        exit 1
      fi
    done
    exec ros2 launch tool_manipulator_bringup manual.launch.py real_hardware:=true launch_joy:=false
  '
