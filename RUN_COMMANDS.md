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

**복귀 방식 선택** (기본은 정지→180도 회전→전진 복귀):

```bash
bash ~/ResKU/src/drive/autonomous/robot_bringup/scripts/start_manual_return.sh reverse_return:=true
```

회전 없이 왔던 길 그대로 후진으로 복귀한다. `/path/record`, `/path/return`은
그대로다 — UI 쪽은 이 옵션과 무관하다. **실차에서 후진 복귀는 아직 안 해봤으니
사람이 뒤쪽을 보면서 처음 몇 번은 시험할 것.**


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

## 2. RViz (새 터미널)

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run manual_return_sim sim_debug_viz
```

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash && source install/setup.bash
rviz2 -d install/manual_return_sim/share/manual_return_sim/config/sim_manual_return.rviz
```

---

# 🔁 복귀 시험 (조이스틱으로 경로 만들고 제자리로 돌아오는지)

터미널 4개: 로봇 PC(SSH) 1개, 로컬 PC에 조이스틱/RViz 2개(`sim_debug_viz`+`rviz2`)/
연결 확인용 1개. 아래 순서를 **그대로** 따라야 한다 — 특히 **PS 버튼을 언제 누르는지**가 중요.

## 순서

1. **🤖 로봇 PC**: CAN 켜기 → `start_manual_return.sh` (위 "로봇 PC" 절 그대로)
2. **💻 로컬 PC**: `ros2 topic list`로 `/mission/return/state`, `/odometry/filtered`가
   보이는지 확인 (안 보이면 연결 문제 — 아래 "연결이 안 될 때" 참고)
3. **💻 로컬 PC**: RViz 두 터미널 실행 (위 "로컬 PC > 2. RViz")
4. **💻 로컬 PC**: 조이스틱 실행 (위 "로컬 PC > 1. 조이스틱")
5. 로봇을 **경로를 시작하고 싶은 바로 그 지점**에 둔다 (여기가 나중에 "제자리"가 되는 원점)
6. **여기서 PS(10번) 버튼을 누른다.** ← 질문하신 부분, 아래 설명 참고
7. 조이스틱으로 원하는 경로를 그리며 주행한다 (RViz에 초록 선이 그려짐)
8. `space`(정지 상태)로 세운 뒤 **Share(8번) 버튼**을 눌러 복귀 시작
9. RViz에서 하늘색(실제 복귀 궤적)이 초록(기록한 경로)을 따라 원점 근처로
   돌아오는지 관찰한다 (주황선은 계획된 복귀 경로)

## PS(10번) 버튼, 꼭 처음에 눌러야 하나?

**네, 경로 기록을 시작하고 싶은 지점에서 딱 한 번 눌러야 합니다.** 안 누르면 로봇은
`IDLE` 상태에 계속 머물고, 경로가 하나도 안 쌓여서 나중에 Share(복귀)를 눌러도
아무 일도 안 일어납니다(경로가 없으니 `WAIT_RETURN_COMMAND` 상태 자체에 못 들어감).

- 조이스틱으로 미리 이동만 하는 건 상관없다(그 동안은 기록 안 됨). **PS를 누른 순간의
  위치가 그대로 원점(0,0)이 된다.** 그러니 "여기가 출발점이다" 싶은 자리에서 눌러야 한다.
- PS를 늦게 누르면(이미 경로 중간까지 간 뒤에 누르면) 그 중간 지점이 원점이 되고,
  그 전에 지나온 구간은 기록에서 빠진다.
- 한 번 눌렀으면 다시 누를 필요 없다(눌러도 `IDLE` 상태가 아니면 무시된다).

## 연결이 안 될 때

`ros2 topic list`에 로봇 토픽이 안 보이면:
```bash
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
그래도 안 되면 JECS 쪽 네트워크 인터페이스 설정을 다시 봐야 한다(이전에 겪었던
`~/ResKU/config/cyclonedds_jecs.xml`의 인터페이스 이름 문제).

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

## 시험 (사람이 직접) — 시계는 안 봐도 된다

1. 조이스틱으로 제자리 회전 3~5초
2. 바닥 선으로 **실제 회전각**을 잰다 (왼쪽 +, 오른쪽 -) — **몇 번째로 돌린 건지 순서만
   기억해두면 된다** (시각은 CSV에 자동으로 남아서 안 적어도 됨)
3. 좌 2회 · 우 2회, 최소 4번 반복

## 💻 로컬 PC — 구간 확인 (모니터 켠 터미널에서 Ctrl+C 후, 같은 폴더에서)

```bash
python3 compute_calibration.py --list
```
방금 한 회전들이 번호(1, 2, 3...)와 함께 자동으로 나열된다. "이동+회전 혼합" 표시가
붙은 구간은 순수 회전이 아니니 빼고 계산한다.

## 💻 로컬 PC — 계산: 번호=실제각도

```bash
python3 compute_calibration.py "1=90" "2=-82" "3=88" "4=-79"
```
(현재 로봇에 적용된 값이 1.244가 아니면 앞에 `CALIB_CURRENT_TW=값 ` 붙일 것)

## 🤖 로봇 PC — 새 값으로 재실행

```bash
bash ~/ResKU/src/drive/autonomous/robot_bringup/scripts/start_manual_return.sh effective_track_width_m:=<새값>
```

## 검증 — 새 값이 맞는지 확인

`calib_monitor.py`를 다시 켜 둔 채(꺼졌으면 다시 실행), 회전 1~2회를 **똑같이** 한다.
계산기를 돌릴 때 `CALIB_CURRENT_TW`를 **방금 넣은 새값**으로 바꿔서 넣는다:

```bash
python3 compute_calibration.py --list
CALIB_CURRENT_TW=<새값> python3 compute_calibration.py "1=88"
```

여기서 나온 "제안값"이 `<새값>`과 거의 같으면(차이가 작으면) 끝. 여전히 많이 다르면
그 제안값으로 다시 재실행 → 검증을 한 번 더 반복한다.
