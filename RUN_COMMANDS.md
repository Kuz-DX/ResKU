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
