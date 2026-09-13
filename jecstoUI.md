# JECS → DOLBOT CENTER

## 터미널 1 — CAN

```bash
ssh jecs@192.168.0.100
sudo ip link set down can_drive
sudo ip link set can_drive type can bitrate 1000000
sudo ip link set up can_drive
sudo ip link set down can_arm
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set up can_arm
ip -br link | grep -E 'can_drive|can_arm'
```

## 터미널 2 — 주행용 D455

```bash
ssh jecs@192.168.0.100
source /opt/ros/humble/setup.bash
source ~/dolbotZ/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch dolbotz drive_cam.launch.py
```

## 터미널 3 — 로봇팔용 D455

```bash
ssh jecs@192.168.0.100
source /opt/ros/humble/setup.bash
source ~/dolbotZ/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch dolbotz arm_cam.launch.py
```

## 터미널 4 — 좌·우 사이드 카메라

```bash
ssh jecs@192.168.0.100
source /opt/ros/humble/setup.bash
source ~/dolbotZ/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch dolbotz side_cameras.launch.py
```

`left_device`/`right_device` 기본값이 `/dev/v4l/by-id/...` 고정 경로(시리얼
26FFF4DF=좌측, B8C0586F=우측)로 박혀있어 매번 `v4l2-ctl --list-devices`로
번호 확인 후 인자로 넘길 필요 없음(재부팅/재연결해도 번호 안 바뀜). 카메라를
다른 걸로 교체했을 때만 `ls -la /dev/v4l/by-id/`로 새 시리얼 확인 후
`left_device:=...`/`right_device:=...`로 오버라이드할 것.

## 터미널 5 — 경로·주행 상태

```bash
ssh jecs@192.168.0.100
source /opt/ros/humble/setup.bash
source ~/dolbotZ/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch dolbotz mission_winter.launch.py enable_visualizer:=false
```

## 터미널 6 — 오도메트리·IMU·진단

```bash
ssh jecs@192.168.0.100
source /opt/ros/humble/setup.bash
source ~/dolbotZ/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch robot_bringup reduced_odom_bringup.launch.py
```

## 터미널 7 — 로봇팔 관절

```bash
ssh jecs@192.168.0.100
source /opt/ros/humble/setup.bash
source ~/dolbotZ/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py
```

## 터미널 8 — rosbridge

```bash
ssh -L 9090:localhost:9090 jecs@192.168.0.100
source /opt/ros/humble/setup.bash
source ~/dolbotZ/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## 모니터링 PC — UI

```bash
cd /home/kuzdx/DolbotZ-Center
python3 -m http.server 8080
xdg-open http://localhost:8080/index-compressed.html
```
