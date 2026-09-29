# 실행 명령 (manual + return 미션)

두 PC 모두 같은 ROS 도메인을 쓴다.

```bash
export ROS_DOMAIN_ID=99
```

---

# 🤖 로봇 PC

## 1. CAN 켜기 (부팅할 때마다)

```bash
sudo ip link set can_drive type can bitrate 1000000
sudo ip link set up can_drive
```

## 2. 미션 실행 (이전 실행에서 남은 프로세스를 먼저 정리하고 실행한다)

```bash
bash ~/ResKU/src/drive/autonomous/robot_bringup/scripts/start_manual_return.sh
```

정리만 하고 싶을 때:

```bash
bash ~/ResKU/src/drive/autonomous/robot_bringup/scripts/start_manual_return.sh --cleanup-only
```


# 💻 로컬 PC

## 1. 조이스틱 (실차는 조이스틱으로만 움직인다)

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch manual_joy_control manual_control.launch.py
```

| 버튼 | 동작 |
|---|---|
| **10번 (PS)** | **녹화 시작** (`/path/record`) |
| **8번 (Share)** | **복귀 시작** (`/path/return`) |

조이스틱이 주행 상태일 때만 동작한다. (Options 9번으로 팔로 넘어가 있으면 무시됨)

---

# 📐 회전 보정치(effective_track_width_m) 재측정

바닥이 바뀌면 슬립이 달라져서 다시 재야 한다. **로봇 PC(JECS, SSH)**와
**로컬 PC** 터미널을 하나씩 띄워 놓고 진행한다.

## 🤖 로봇 PC — 연결 확인 + 미션 실행

```bash
ssh jecs
sr
bash ~/ResKU/src/drive/autonomous/robot_bringup/scripts/start_manual_return.sh
```

## 💻 로컬 PC — 모니터 (새 터미널)

```bash
cd ~/ResKU
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/drive/autonomous/robot_bringup/scripts
python3 calib_monitor.py
```

`ros2 topic list`에 `/mission/return/state`, `/odometry/filtered`가 안 보이면
로봇이 아직 연결 안 된 것이니 아래를 먼저 실행한다.
```bash
ros2 topic list
```

## 💻 로컬 PC — 조이스틱 (또 다른 새 터미널)

RUN_COMMANDS.md 위쪽 "로컬 PC > 1. 조이스틱"과 같은 명령.

## 시험 (사람이 직접)

1. 조이스틱으로 제자리 회전 3~5초, `calib_monitor.py` 화면에 찍히는 **시작·끝 시각**을 적어둔다
2. 바닥 선으로 **실제 회전각**을 잰다 (왼쪽 +, 오른쪽 -)
3. 좌 2회 · 우 2회, 최소 4번 반복

## 💻 로컬 PC — 계산 (모니터 켠 터미널에서 Ctrl+C 후)

```bash
python3 compute_calibration.py "14:32:10-14:32:15:90" "14:35:00-14:35:04:-82"
```
(현재 로봇에 적용된 값이 1.244가 아니면 앞에 `CALIB_CURRENT_TW=값 ` 붙일 것)

## 🤖 로봇 PC — 새 값으로 재실행

```bash
bash ~/ResKU/src/drive/autonomous/robot_bringup/scripts/start_manual_return.sh effective_track_width_m:=<새값>
```

## 검증 — 새 값이 맞는지 확인

`calib_monitor.py`를 다시 켜 둔 채(꺼졌으면 다시 실행), 회전 1~2회를 **똑같이** 하고
시작·끝 시각과 실제 각도를 또 적어둔다. 계산기를 돌릴 때 `CALIB_CURRENT_TW`를
**방금 넣은 새값**으로 바꿔서 넣는다:

```bash
CALIB_CURRENT_TW=<새값> python3 compute_calibration.py "15:10:00-15:10:04:88"
```

여기서 나온 "제안값"이 `<새값>`과 거의 같으면(차이가 작으면) 끝. 여전히 많이 다르면
그 제안값으로 다시 재실행 → 검증을 한 번 더 반복한다.
