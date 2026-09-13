# PlanA - 만약에 여기서 헛돌거나 너무 많이 돌아서 강제 종료 하게 되는 상황에서는 내가 바로 고치거나 너한테 새로운 명령을 카톡으로 바로 보낼게. 그니까 너무 당황하지말고 planB 부터 해줘.

# 봄
ros2 launch robot_bringup mission_spring_drive.launch.py initial_straight_drive_sec:=3.0

# 가을 / 겨울
ros2 launch robot_bringup autonomous_pland.launch.py

# 여름
ros2 launch robot_bringup mission_summer_drive.launch.py


------------------------------------------------------


# PlanB - 기존 purepursuit 가공

# 봄 
ros2 launch robot_bringup mission_spring_purepursuit_drive.launch.py initial_straight_drive_sec:=3.0

# 가을 / 겨울
ros2 launch robot_bringup autonomous.planz.launch.py use_outer_wheel_boost:=true target_linear_speed_m_s:=0.4

# 여름 - 모든 구간에서 동일 그치만 헷갈리니까 다 붙여넣을게
ros2 launch robot_bringup mission_summer_drive.launch.py


------------------------------------------------------

# PlanC - 기존 autonomous 그대로 살린 코드

# 봄
ros2 launch robot_bringup mission_spring_autonomous_drive.launch.py initial_straight_drive_sec:=3.0


# 가을 / 겨울
ros2 launch robot_bringup autonomous_planc.launch.py

# 여름
ros2 launch robot_bringup mission_summer_drive.launch.py