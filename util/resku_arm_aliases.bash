# Main manual control. Source this file in an existing terminal.
unalias Resku_arm_servox vr_arm_servox Resku_arm Resku_arm_sep vr_arm vr_arm_sep 2>/dev/null || true
unset -f Resku_arm vr_arm vr_arm_sep 2>/dev/null || true
_resku_arm_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
alias Resku_arm="cd ${_resku_arm_workspace@Q} && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch tool_manipulator_bringup manual.launch.py real_hardware:=true"
alias vr_arm="cd ${_resku_arm_workspace@Q} && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch tool_manipulator_bringup manual.launch.py"
unset _resku_arm_workspace
