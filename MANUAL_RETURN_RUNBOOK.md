# Manual + Return 미션 실행 명령어

manual 주행 중 경로를 기록하고, RETURN 트리거 후 자동으로 출발지까지
복귀하는 미션의 실행 명령어 모음. 아키텍처/토픽 상세는 `topic.md`,
`howtorun.md`, `LOCAL_ODOM_RETURN_PATH.md` 참고.

## 1. 로봇 PC — CAN 인터페이스 켜기

```bash
sudo ip link set can_drive type can bitrate 1000000
sudo ip link set up can_drive
ip -details link show can_drive
```

## 2. 로봇 PC — manual+return 통합 미션 실행

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py
```

`rmd_x8_driver`(CAN 유일 소유, `/wheel/odom`) + `myahrs_driver`(`/imu`) +
`reduced_odom`(`/odometry/filtered`) + `drive_cmd_mux` +
`manual_path_recorder` + `return_state_machine` + `return_path_follower`가
한 번에 뜬다.


## 3. 원격 PC — 조이스틱

```bash
ros2 launch manual_joy_control manual_control.launch.py
```


## 4. rosbag 기록 (선택, 로봇 PC)

```bash
bash src/drive/autonomous/robot_bringup/scripts/record_manual_drive.sh
```

## 5. 확인용 명령어

```bash
# 오도메트리 (manual 주행 중에도 나와야 정상)
ros2 topic echo /wheel/odom
ros2 topic echo /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_link

# 조종/최종 명령
ros2 topic echo /motor_speed_cmd_manual   # 조이스틱 출력 (dps)
ros2 topic echo /cmd_vel                  # mux 최종 출력 (Twist) -> rmd_x8_driver

# 복귀 미션 상태
ros2 topic echo /mission/return/state     # IDLE/MANUAL_RECORDING/.../FINISHED
ros2 topic echo /recorded_path            # 누적 기록 경로
ros2 topic echo /return_path              # RETURN 트리거 후 1회 발행되는 복귀 경로
```

## 6. RETURN 트리거

조이스틱 버튼 8번(기본값, 미검증 — 실기에서 확인 필요). 수동으로
트리거하려면:

```bash
ros2 topic pub -1 /mission/return/trigger std_msgs/msg/Bool "{data: true}"
```

## 7. 동시 실행 금지

`autonomous.launch.py`(MPPI, 2단계 평가용 보존)와
`manual_return_bringup.launch.py`는 **절대 동시에 실행하지 말 것** — 같은
CAN 버스/`/cmd_vel`을 두고 충돌한다.
