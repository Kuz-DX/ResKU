# GPS 없이 Local Odom으로 왕복 경로를 만드는 방법

## 결론

GPS와 같은 절대 좌표계가 없어도 구현할 수 있다.

로봇이 출발하는 순간의 위치와 방향을 `(0, 0, 0)`으로 정의한 고정 좌표계를 만들고, 수동 조종 중 로봇의 local odometry를 그 좌표계에 누적하면 된다. 조종이 끝난 뒤 기록된 경로의 순서를 뒤집고 로봇을 180도 회전시키면 원래 출발점으로 돌아가는 경로를 만들 수 있다.

여기서 사용하는 `odom` 또는 별도의 `mission` 프레임은 지구 기준의 전역 좌표계는 아니지만, 한 번의 임무가 진행되는 동안 고정되어 있으므로 로봇 입장에서는 가상의 전역 좌표계 역할을 한다.

단, 바퀴와 궤도의 미끄러짐 및 IMU 오차가 누적되기 때문에 실제 바닥에서 완전히 동일한 궤적을 되짚는 것은 보장할 수 없다.

## 전체 동작 순서

```text
WAIT_ODOM
    ↓
수동 조종 시작
    ├─ 시작 시간 저장: mission_start_time
    ├─ 현재 위치와 방향을 mission 원점 (0, 0, 0)으로 설정
    └─ local odom을 Path로 누적
    ↓
수동 조종 종료
    ├─ 모터 정지
    └─ 기록 경로 정리 및 저장
    ↓
반환 경로 생성
    ├─ 경로점 순서 반전
    └─ 반환 진행 방향 재계산
    ↓
IMU/odom 기준 180도 회전
    ↓
반환 경로 추종
    ↓
원래 출발점 도착 및 정지
```

권장 상태 머신은 다음과 같다.

```text
IDLE
  → RECORD_MANUAL
  → STOP_BEFORE_TURN
  → TURN_180
  → FOLLOW_RETURN_PATH
  → FINISHED
```

어떤 상태에서도 비상 정지와 명령 타임아웃은 유지해야 한다.

## 시작점을 `(0, 0, 0)`으로 만드는 방법

기록 시작 순간의 `odom → base_link` 자세를 `T0`라고 한다. 이후 각 시각의 로봇 자세를 다음과 같이 변환한다.

```text
T_mission_robot(t) = inverse(T0) × T_odom_robot(t)
```

그러면 기록 시작 시점은 항상 다음과 같다.

```text
x = 0
y = 0
yaw = 0
time = 0
```

시간은 다음처럼 상대 시간으로 저장할 수 있다.

```text
relative_time = current_ros_time - mission_start_time
```

단순한 `nav_msgs/Path` 추종에는 시간 정보가 필수는 아니다. 기록 당시의 속도까지 재현하려면 위치만 저장하는 Path가 아니라 시간, 선속도, 각속도를 포함한 별도의 trajectory 형식이 필요하다.

## 현재 odometry 코드와의 관계

현재 `reduced_odom`의 상태는 `[x, y, yaw, vx, wz]`이고, 노드 생성 시 `x`와 `y`를 0으로 초기화한다.

- `src/drive/autonomous/reduced_odom/include/reduced_odom/reduced_ekf.hpp`
- `src/drive/autonomous/reduced_odom/src/reduced_odom_node.cpp`

이 노드는 다음 데이터를 사용하고 발행한다.

```text
/wheel/odom ─┐
             ├─ reduced_odom_node
/imu ────────┘
                 ├─ /odometry/filtered
                 └─ TF: odom → base_link
```

위치 상태는 0에서 시작하지만 yaw는 첫 IMU 방위각으로 초기화된다. 이 초기 방위각은 임의의 유효한 값일 수 있으므로, 경로 기록기가 시작 자세의 역변환을 적용해야 `(0, 0, 0)` 기준을 확실히 만들 수 있다.

또한 odom 노드를 실행한 시점과 실제 경로 기록 시작 시점이 다를 수 있으므로, odom 노드를 재시작해서 원점을 맞추기보다 기록 시작 순간의 자세를 캡처해 별도의 `mission` 좌표계를 만드는 편이 안전하다.

## 반환 경로 생성

수동 조종 중 기록한 경로가 다음과 같다고 가정한다.

```text
P0, P1, P2, ... PN
```

경로의 순서는 다음처럼 반전한다.

```text
PN, ..., P2, P1, P0
```

로봇이 반환 경로를 전진으로 따라갈 경우 각 경로점의 방향도 반대로 만들어야 한다.

```text
return_yaw = normalize(recorded_yaw + π)
```

실제로는 기록된 로봇 yaw를 그대로 사용하는 것보다, 반전된 경로의 인접 점으로부터 진행 방향을 다시 계산하는 방법이 더 안정적이다.

```text
return_yaw[i] = atan2(
    return_y[i + 1] - return_y[i],
    return_x[i + 1] - return_x[i]
)
```

현재 `purepursuit.py`는 경로점의 orientation보다 점의 위치와 순서를 사용한다. 따라서 로봇을 먼저 180도 회전시키고 반전된 점들을 순서대로 제공하면 원리적으로 추종할 수 있다. Nav2처럼 Path의 orientation도 사용하는 제어기를 사용한다면 반환 방향을 반드시 올바르게 채워야 한다.

수동 주행 중 후진이나 제자리 회전이 포함되었다면 단순 경로 반전만으로는 원래 동작과 같은 경로가 되지 않을 수 있다. 이런 경우에는 각 구간의 전진·후진 방향과 속도 부호도 함께 기록해야 한다.

## 반환 시작점도 다시 `(0, 0)`으로 만드는 경우

반환 시작 시점, 즉 원래 경로의 끝점에 새로운 `return_origin` 프레임을 고정할 수 있다.

이때 로봇이 180도 회전을 끝낸 자세를 `return_origin`의 `(0, 0, 0)`으로 정의하고, 반전된 전체 경로를 이 좌표계로 변환한다.

```text
T_return_path[i] = inverse(T_return_start) × T_mission_path[i]
```

그러면 반환 경로의 첫 점은 `(0, 0)`이 되고, 원래 출발점은 `return_origin`에서 본 최종 목적지가 된다.

하지만 실제 `odom`을 중간에 리셋하는 것은 권장하지 않는다. odom을 리셋하면 TF가 순간적으로 끊기거나 좌표가 점프하여 제어기가 잘못된 명령을 만들 수 있다. 기존 odom은 계속 유지하고 경로 좌표만 변환하거나 별도의 고정 프레임을 추가해야 한다.

가장 단순하고 안전한 방식은 처음 만든 `mission` 프레임을 끝까지 유지하는 것이다. 이 경우 반환 경로는 원래 끝점에서 시작해 최초 원점 `(0, 0)`으로 도착한다.

## 현재 수동 주행 구성의 제약

현재 수동 주행용 `can_driver_node`는 속도 명령만 CAN으로 보내고 모터 응답을 읽지 않는다. 따라서 수동 주행 중에는 `/wheel/odom`을 생성할 수 없다.

이 제약은 다음 파일에도 명시되어 있다.

- `src/drive/manual/can_driver/can_driver/can_driver_node.py`
- `src/drive/autonomous/robot_bringup/launch/manual_drive_sensors.launch.py`

반면 `rmd_x8_driver_node`는 다음 기능을 모두 가지고 있다.

- `/cmd_vel` 구독
- 모터 CAN 피드백 수신
- `/wheel/odom` 발행
- 모터 통신 타임아웃 및 진단

관련 파일:

- `src/drive/autonomous/rmd_x8_driver/rmd_x8_driver/rmd_x8_driver_node.py`

따라서 이 기능을 구현할 때는 `rmd_x8_driver_node`를 유일한 CAN 소유자로 두는 구성이 권장된다.

```text
manual joystick
    → /cmd_vel_manual ─┐
                       ├─ drive command mux → /cmd_vel
return path controller ─┘                       ↓
                                      rmd_x8_driver_node
                                         ├─ CAN 모터 제어
                                         └─ /wheel/odom
                                                  ↓
                                         reduced_odom_node
                                                  ↓
                                       /odometry/filtered
```

수동용 `can_driver_node`와 `rmd_x8_driver_node`를 동시에 실행하면 같은 CAN 채널과 모터 ID에 서로 다른 명령을 보낼 수 있으므로 같이 실행하면 안 된다.

명령 선택기는 현재 상태에 따라 수동 명령과 반환 명령 중 하나만 통과시켜야 한다. 두 제어기가 동일한 모터 명령 토픽에 동시에 발행하도록 구성하면 안 된다.

## 경로 기록 시 권장 처리

odom 메시지를 받을 때마다 모두 저장하면 경로점이 지나치게 많고 정지 노이즈까지 기록될 수 있다. 다음 조건 중 하나를 만족할 때만 새 점을 추가하는 방식을 권장한다.

- 이전 점에서 일정 거리 이상 이동했을 때
- yaw가 일정 각도 이상 변했을 때
- 최대 기록 주기를 초과했을 때

기록이 끝난 뒤에는 다음 처리가 필요할 수 있다.

- 중복점 제거
- 너무 가까운 점 제거 또는 일정 간격으로 재표본화
- 위치 및 방향 노이즈 완화
- 급격한 꺾임 제한
- 출발점과 도착점에 정지 허용 오차 설정

기록 원본은 별도로 보존하고, 실제 추종에는 정리된 Path를 사용하는 것이 좋다.

## 기존 경로 추종 코드 사용 시 주의점

`src/dolbotz/dolbotz/purepursuit.py`는 Path를 수신할 때 해당 경로를 `base_link` 좌표로 변환하고 그 결과를 저장한다. 고정된 `mission` 또는 `odom` 경로를 한 번만 보내면 로봇이 움직인 뒤 저장된 `base_link` 기준 좌표가 오래된 값이 된다.

따라서 기존 pure pursuit를 사용하려면 다음 중 하나가 필요하다.

- 반환 Path를 현재 시간으로 계속 재발행하여 매번 최신 TF로 변환
- pure pursuit가 고정 프레임 Path를 보관하고 제어 주기마다 `base_link`로 변환하도록 수정

또는 `odom` 프레임의 고정 경로를 직접 추종하는 Nav2 `FollowPath` 구조를 사용할 수 있다. 현재 `path_relay_node`의 기본 목표 프레임도 `odom`이다.

- `src/drive/autonomous/path_relay/src/path_relay_node.cpp`

## 180도 회전 제어

180도 회전을 일정 시간 동안 모터를 구동하는 방식으로 처리하면 안 된다. 궤도 미끄러짐과 지면 상태에 따라 실제 회전량이 달라지기 때문이다.

회전 시작 yaw를 `yaw_start`라고 하면 목표는 다음과 같다.

```text
yaw_target = normalize(yaw_start + π)
yaw_error  = normalize(yaw_target - current_yaw)
```

`yaw_error`에 따라 각속도를 줄이는 폐루프 제어를 사용하고, 오차가 허용 범위 안에서 일정 시간 유지될 때 회전 완료로 판정해야 한다. 회전하는 동안 선속도는 0으로 유지한다.

## 정확도와 한계

가상 좌표계는 좌표를 일관되게 표현해 줄 뿐, 위치 오차를 제거하지는 않는다.

주요 오차 원인은 다음과 같다.

- 궤도 또는 바퀴의 미끄러짐
- 좌우 바퀴 반경 및 유효 트랙 폭 오차
- IMU yaw 바이어스와 시간에 따른 드리프트
- 울퉁불퉁한 지면과 경사
- 제자리 180도 회전 중 발생하는 큰 슬립
- CAN 피드백 지연 또는 누락

따라서 짧은 거리에서는 대략적인 왕복 주행이 가능하지만, 긴 거리나 미끄러운 지면에서는 실제 위치가 기록된 좌표와 점점 달라질 수 있다. 출발점에 정확하게 복귀해야 한다면 GPS 대신 다음과 같은 상대 위치 보정을 사용할 수 있다.

- LiDAR SLAM 및 loop closure
- Visual SLAM 또는 visual odometry
- 출발점이나 경로 중간의 AprilTag
- 고정 카메라 또는 UWB 앵커
- 알려진 랜드마크 기반 위치 보정

## 구현에 필요한 핵심 구성요소

1. 수동 조종 명령을 `/cmd_vel_manual`로 만드는 노드 또는 변환기
2. 수동/반환 명령을 독점적으로 선택하는 drive command mux
3. `rmd_x8_driver_node`와 `reduced_odom_node`를 수동 주행 중에도 유지하는 launch 구성
4. 시작 자세를 원점으로 변환해 local odom을 기록하는 path recorder
5. 경로 순서 반전, 재표본화 및 반환 yaw를 생성하는 path processor
6. IMU/odom yaw 기반 180도 폐루프 회전 제어
7. 반환 Path 추종기
8. 전체 과정을 관리하는 상태 머신과 비상 정지 처리

## 최종 판단

GPS 없이도 구현 가능하다. 핵심은 움직이는 `base_link`가 아니라 출발 순간에 고정한 `mission` 프레임에서 경로를 기록하는 것이다.

현재 저장소에서는 odometry와 경로 추종에 필요한 기반 코드가 이미 일부 존재하지만, 수동 주행 체인이 odom을 만들지 못하므로 수동 및 자율 주행을 하나의 피드백 가능한 CAN 드라이버 체계로 통합해야 한다. 또한 기록, 반전, 180도 회전, 명령 전환을 관리하는 별도의 상태 머신이 추가로 필요하다.
