# Source this file to make the Servo-free manual experiment available in Bash.
alias Resku_arm_servox='cd /home/sw/ResKU && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch tool_manipulator_bringup manual_servo_out.launch.py real_hardware:=true'
alias vr_arm_servox='cd /home/sw/ResKU && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch tool_manipulator_bringup manual_servo_out.launch.py'
