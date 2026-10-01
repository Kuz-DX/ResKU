#!/usr/bin/env bash
# Run the current source MoveToPose command without relying on a stale ROS install.
# Usage: bash utils/move_to_pose.sh <pose|--list> [move_to_named_pose.py options]
set -eo pipefail

workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"

exec python3 \
  "${workspace_dir}/src/arm/tool_manipulator_bringup/scripts/move_to_named_pose.py" \
  --srdf "${workspace_dir}/src/arm/tool_manipulator_moveit_config/config/tool_manipulator.srdf" \
  --hardware "${workspace_dir}/src/arm/tool_manipulator_bringup/config/hardware.yaml" \
  "$@"
