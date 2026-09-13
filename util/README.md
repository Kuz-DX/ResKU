# ROS CompressedImage → MediaMTX RTSP 브리지

`usb_cam`이 발행하는 JPEG `sensor_msgs/msg/CompressedImage`를 구독하고,
GStreamer로 H.264 baseline 영상을 인코딩해 MediaMTX에 RTSP/TCP로 publish한다.
Python에서는 JPEG를 디코딩하지 않으며 ROS 패키지 빌드가 필요 없다.

## 의존성

ROS 2 Humble과 `usb_cam`이 설치되어 있다는 전제에서 최초 한 번 설치한다.

```bash
sudo apt update
sudo apt install gstreamer1.0-rtsp gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly python3-gi
```

필수 GStreamer 요소를 확인한다.

```bash
gst-inspect-1.0 appsrc jpegdec x264enc h264parse rtspclientsink
```

## 실행 순서

### 1. MediaMTX

```bash
cd /home/kuzdx/DolbotZ-Center/deploy
sudo docker compose up -d
```

### 2. usb_cam

노트북 웹캠 한 대를 직접 실행한다. 이 노트북의 Chicony 카메라는
`raw_mjpeg`에서 손상된 흑백 프레임이 확인되어, 정상 동작을 확인한
`mjpeg2rgb` 1280x720 30fps 설정을 사용한다. `image_transport_plugins`가
`image_raw/compressed`를 생성하므로 브리지 입력 토픽은 그대로 유지된다.

```bash
source /opt/ros/humble/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/side/left \
  -p video_device:=/dev/video0 \
  -p pixel_format:=mjpeg2rgb \
  -p image_width:=1280 -p image_height:=720 -p framerate:=30.0
```

`dolbotz` 패키지가 이미 설치된 워크스페이스에서는 프로젝트의 기존
`side_cameras.launch.py`를 대신 사용할 수 있다. 이 유틸리티 때문에 패키지를
새로 빌드할 필요는 없다.

토픽을 확인한다.

```bash
ros2 topic hz /side/left/image_raw/compressed
ros2 topic info -v /side/left/image_raw/compressed
```

### 3. 브리지

좌측 카메라를 MediaMTX `sub1` 경로로 보낸다.

```bash
source /opt/ros/humble/setup.bash
python3 /home/kuzdx/dolbotZ/util/ros_compressed_to_rtsp.py --ros-args \
  -r __node:=left_camera_rtsp_bridge \
  -p image_topic:=/side/left/image_raw/compressed \
  -p rtsp_url:=rtsp://127.0.0.1:8554/sub1 \
  -p fps:=30 -p bitrate_kbps:=1500 -p keyframe_interval:=30
```

우측 카메라는 별도 프로세스로 `sub2`에 보낸다.

```bash
source /opt/ros/humble/setup.bash
python3 /home/kuzdx/dolbotZ/util/ros_compressed_to_rtsp.py --ros-args \
  -r __node:=right_camera_rtsp_bridge \
  -p image_topic:=/side/right/image_raw/compressed \
  -p rtsp_url:=rtsp://127.0.0.1:8554/sub2 \
  -p fps:=15 -p bitrate_kbps:=1500 -p keyframe_interval:=15
```

`fps`는 입력 토픽의 실제 발행률과 같게 설정한다. MediaMTX가 늦게 시작되거나
재시작되면 브리지는 기본 5초 간격으로 RTSP 연결을 다시 시도한다.

## 확인

```bash
sudo docker compose -f /home/kuzdx/DolbotZ-Center/deploy/docker-compose.yml logs \
  --tail=100 mediamtx
curl -fsS http://127.0.0.1:8888/sub1/index.m3u8 | head
```

웹 UI의 기본 주소는 다음과 같다.

```text
WHEP: http://localhost:8889/sub1/whep
HLS:  http://localhost:8888/sub1/index.m3u8
```

## 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `image_topic` | `/side/left/image_raw/compressed` | JPEG CompressedImage 입력 |
| `rtsp_url` | `rtsp://127.0.0.1:8554/sub1` | MediaMTX publish URL |
| `fps` | `15` | 입력 프레임률과 GStreamer caps |
| `bitrate_kbps` | `1500` | H.264 목표 비트레이트(kbit/s) |
| `keyframe_interval` | `15` | H.264 키프레임 간격 |
| `restart_delay_sec` | `5.0` | RTSP 오류 후 재시도 간격 |
| `stats_interval_sec` | `10.0` | 프레임 통계 출력 간격 |

일반 JPEG/MJPEG만 받는다. `compressedDepth`와 PNG 이미지는 의도적으로 거부한다.
