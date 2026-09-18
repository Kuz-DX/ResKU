**[2026 사용자 결정, 계절 미션 정리로 삭제됨]**

계절별 미션 주행 launch(`mission_spring_drive.launch.py`,
`autonomous_pland.launch.py`, `autonomous_plane.launch.py`,
`mission_summer_drive.launch.py` 등)와 그 아래 "경사 주행 plan A/B/C/D/E,
Z" 명령어들은 소스 자체가 삭제됐다 — 지금은 manual+return 미션만
운용한다(`robot_bringup/launch/manual_return_bringup.launch.py`,
`howtorun.md` 참고). MPPI(`autonomous.launch.py`)는 나중에 recorded return
path와 비교하는 2단계 평가용으로만 소스가 남아있다.

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
