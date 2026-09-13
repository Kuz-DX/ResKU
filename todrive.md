**봄가을 MPPI 제거 코드**
```bash
# 봄 구간
ros2 launch robot_bringup mission_spring_drive.launch.py
ros2 launch robot_bringup mission_spring_drive.launch.py initial_straight_drive_sec:=3.0
ros2 launch robot_bringup mission_spring_drive.launch.py initial_straight_drive_sec:=3.0 initial_straight_speed_mps:=0.3

# 가을/ 겨울 구간
ros2 launch robot_bringup autonomous_pland.launch.py
-> 너무 빠르게 돌면
ros2 launch robot_bringup autonomous_plane.launch.py

# 여름 구간
ros2 launch robot_bringup mission_summer_drive.launch.py

```

-------------------------------------------------------------------


# 추종구간 모든 CLI 명령어
```bash
ros2 launch dolbotz purepursuit.launch.py \
use_distance_scaled_speed:=true \
use_bearing_steering:=true \
max_wheel_speed_dps:=600.0 \
camera_pitch_rad:=0.0378 \
camera_roll_rad:=0.0000 \
max_linear_accel_mps2:=3.0 \
max_linear_decel_mps2:=1.0 \
max_angular_accel_rad_s2:=6.0 \
bearing_steering_gain:=3.0 \
bearing_deadband_lateral_m:=0.8 \ 
bearing_inplace_threshold_rad:=0.05 \
distance_speed_gain:=2.0 \
distance_speed_min_m_s:=0.0 \
distance_speed_max_m_s:=1.8 \
distance_band_min_m:=1.5 \
distance_band_max_m:=2.0 \
distance_speed_reverse_max_m_s:=0.3
```

--------------------------------------------------------------------

** 경사 주행 plan A, B, Z **

```bash
# planA
ros2 launch robot_bringup autonomous.launch.py
# planB
ros2 launch robot_bringup autonomous_blend.launch.py
# planC(planD, E가 안되면 기존 autonomous로 회귀)
ros2 launch robot_bringup autonomous_planc.launch.py
# planD
ros2 launch robot_bringup autonomous_pland.launch.py
# planE
ros2 launch robot_bringup autonomous_plane.launch.py

# planZ(planD, E가 안되면 기존 purepursuit으로 회귀)
#가을겨울
ros2 launch robot_bringup autonomous.planz.launch.py use_outer_wheel_boost:=true target_linear_speed_m_s:=0.4
#봄
ros2 launch robot_bringup mission_spring_purepursuit_drive.launch.py initial_straight_drive_sec:=5.0 initial_straight_speed_mps:=0.3

```
