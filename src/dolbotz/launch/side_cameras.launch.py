"""사이드캠(좌/우 웹캠) 공용 브링업 — usb_cam 패키지(ros-humble-usb-cam, 실행파일
usb_cam_node_exe)로 웹캠 2대를 띄운다. spring/summer/fall 미션 launch가
include해서 쓰는 공용 파일이다.

[2026-09-01] pixel_format:="raw_mjpeg"(카메라가 뱉는 MJPEG 바이트를 재인코딩
없이 그대로 CompressedImage로 통과시키는 모드)를 잠깐 썼었는데, 실기에서
카메라를 바꿔도/단일 캠으로 대역폭 경합을 배제해도 재현되는 손상된 프레임
(위쪽 노이즈 + 아래쪽 검정, cv2/ffmpeg 둘 다 동일하게 깨짐 — 디코더 문제
아님 확인)이 나서 폐기함 — 지금 설치된 usb_cam 버전의 raw_mjpeg passthrough
코드 자체 버그로 결론([[raw-mjpeg-corrupted-frames]] 참고). 같은 카메라를
pixel_format:="mjpeg2rgb"(usb_cam이 직접 디코드해서 raw Image로 발행)로
돌리면 멀쩡했으므로 지금은 이 값을 쓴다.

[수정, 2026-09-01] mjpeg2rgb로 바꾸면서 한동안 "raw Image만 나가니 별도
image_transport republish 노드로 <ns>/image_raw/compressed를 재인코딩해야
한다"고 잘못 판단해 republish 노드 2개를 추가했었는데 — 실제로는
필요 없었다. usb_cam_node는 image_transport::ImageTransport::advertise()로
raw 이미지를 발행하는데, ROS2 image_transport는 이때 설치된 모든 플러그인
(compressed_image_transport, theora_image_transport 등)의 발행자를
"자동으로, 요청 없이" 같이 만든다 — 그래서 pixel_format과 무관하게
usb_cam_node 혼자서 이미 <ns>/image_raw/compressed(그리고 덤으로
/compressedDepth, /theora까지)를 내고 있었다. 거기에 republish 노드를
더 붙였더니 같은 토픽에 발행자가 2개가 돼서(`ros2 topic info -v`로 직접
확인), 구독 쪽에서 두 발행자의 프레임이 번갈아 들어와 "최신/과거 프레임이
겹쳐서 왔다갔다 하는" 증상이 났다 — republish 노드를 다시 제거해서 해결.
이 리포 컨벤션상 모든 카메라 입력이 CompressedImage인 것
`sensor_msgs/CompressedImage` 구독 노드와 자동으로 호환되고,
spring_ifof.py/summer_traffic.py/fall_marker.py 등 구독 쪽 코드는 토픽
이름이 그대로라 전혀 안 건드려도 된다.

[수정 이력, 2026-09-01] 원래 값 "mjpeg"는 usb_cam 0.8.1 기준으로 작성됐던
건데, JECS에 설치된 usb_cam 버전은 그 문자열을 지원하지 않고(`terminate
called ... Specified format 'mjpeg' is unsupported by this ROS driver`로
노드가 즉시 죽음) 대신 "raw_mjpeg"/"mjpeg2rgb"만 받는다 - 로그에 찍힌
지원 포맷 목록 기준. "raw_mjpeg"로 먼저 바꿨다가 위 이유로 다시
"mjpeg2rgb"+republish 조합으로 교체함.

카메라가 실제로 MJPEG 모드를 지원하는지는 `v4l2-ctl --list-formats-ext -d
/dev/videoX`로 확인할 것 — 대부분의 UVC 웹캠(C920/C922 포함)은 지원한다.

[2026-09-01] left_device/right_device 기본값을 `/dev/videoN`(재부팅/재연결
시 열거 순서가 바뀌면 번호가 밀림) 대신 `/dev/v4l/by-id/...` 고정 심볼릭
경로로 바꿈 — `v4l2-ctl --list-devices`로 매번 확인하던 걸 없앰. 시리얼
`26FFF4DF`=좌측, `B8C0586F`=우측으로 실기에서 화면 보고 직접 확인함
(둘 다 같은 C922 모델이라 vendor:product는 같고 시리얼로만 구분됨).
[2026-09-03] DOLBOT CENTER UI에서 LEFT/RIGHT가 반대로 나온다는 보고로
잠깐 이 매핑을 서로 바꿨었으나, 실기 재확인 결과 원래(위) 매핑이 맞았음 —
되돌림. usb_cam_node는 재시작해야 device 파라미터가 바뀌므로, 이후 비슷한
증상은 pane을 완전히 재시작했는지부터 확인할 것.
by-id 경로를 처음 시도했을 때 `Device specified is not available`
(`/dev/../../video12`처럼 안 풀린 경로) 에러가 났었는데, 이건 젝슨의
`install/usb_cam`이 심볼릭 링크를 올바르게 푸는 커밋(usb_cam 서브모듈
`772b25f`, 그 안의 `c62771c Correctly resolve symlinks to device paths`)
이 들어가기 전 바이너리로 안 재빌드된 채 남아있어서였다 —
`colcon build --packages-select usb_cam`으로 재빌드해서 해결. 이후 같은
증상 재발하면 usb_cam 재빌드 여부부터 확인할 것.

발행 토픽 (usb_cam_node.cpp: BASE_TOPIC_NAME="image_raw", image_transport의
advertise()가 설치된 플러그인마다 자동으로 같이 냄 — 아래는 그중 이 리포가
실제로 쓰는 것만 나열, /compressedDepth·/theora도 덤으로 나가지만 구독자
없음. QoS는 rclcpp::QoS(100) = 기본 RELIABLE — DDS 호환 규칙상 "구독자가
요구하는 수준 <= 발행자가 제공하는 수준"이면 매칭되므로, 구독 쪽을
BEST_EFFORT로 해도(dolbotz.utils.qos.SENSOR_DATA_QOS_DEPTH1 등) 이 RELIABLE
발행자와 문제없이 연결된다 — 반대로 구독 쪽을 RELIABLE로 고정했는데
발행자가 BEST_EFFORT인 조합이 비호환 사례다(qos.py 모듈 docstring과 동일한
근거)):
  /side/left/image_raw              sensor_msgs/Image (usb_cam이 직접 디코드)
  /side/left/image_raw/compressed   sensor_msgs/CompressedImage (image_transport가 자동 재인코딩)
  /side/left/camera_info            sensor_msgs/CameraInfo (미보정 — camera_info_url
                                     안 줘서 K/D 전부 0으로 나옴, 캘리브레이션 전)
  /side/right/image_raw             sensor_msgs/Image
  /side/right/image_raw/compressed  sensor_msgs/CompressedImage
  /side/right/camera_info           sensor_msgs/CameraInfo (미보정, 위와 동일)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    left_device = LaunchConfiguration('left_device')
    right_device = LaunchConfiguration('right_device')

    return LaunchDescription([
        # [2026-09-01] /dev/v4l/by-id/ 고정 심볼릭 경로 — 시리얼로 카메라
        # 개체를 식별하므로 재부팅/재연결로 /dev/videoN 번호가 밀려도 안
        # 바뀐다(모듈 docstring 참고). 카메라를 실제로 교체하면(고장 등)
        # 새 카메라의 시리얼로 이 문자열도 같이 바꿔야 한다 —
        # `ls -la /dev/v4l/by-id/`로 확인.
        DeclareLaunchArgument(
            'left_device',
            default_value=(
                '/dev/v4l/by-id/'
                'usb-046d_C922_Pro_Stream_Webcam_26FFF4DF-video-index0'),
            description="좌측 사이드캠(C922, 시리얼 26FFF4DF) V4L2 장치 경로 "
                        "— 다른 카메라로 교체 시 이 인자로 오버라이드할 것."),
        DeclareLaunchArgument(
            'right_device',
            default_value=(
                '/dev/v4l/by-id/'
                'usb-046d_C922_Pro_Stream_Webcam_B8C0586F-video-index0'),
            description="우측 사이드캠(C922, 시리얼 B8C0586F) V4L2 장치 경로 "
                        "— 다른 카메라로 교체 시 이 인자로 오버라이드할 것."),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            namespace='side/left',
            output='screen',
            parameters=[{
                'video_device': left_device,
                'pixel_format': 'mjpeg2rgb',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'io_method': 'mmap',
                'frame_id': 'side_left_camera',
                'camera_name': 'side_left',
                # [2026-09-01] usb_cam_node.cpp 기본값은 brightness=50으로
                # 고정 발행(contrast/saturation/sharpness/gain은 전부 -1
                # "건드리지 않음"인데 brightness만 다름 — src/usb_cam/src/
                # usb_cam_node.cpp:75) — 화면이 너무 어둡게 나와서 -1로
                # 오버라이드해 카메라 자체 기본값을 쓰게 함.
                'brightness': -1,
                # camera_info_url 의도적으로 미설정 — 캘리브레이션 파일 아직 없음
                # (config/calibration/README.md 참고, D455 쪽과 같은 상태).
            }],
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            namespace='side/right',
            output='screen',
            parameters=[{
                'video_device': right_device,
                'pixel_format': 'mjpeg2rgb',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'io_method': 'mmap',
                'frame_id': 'side_right_camera',
                'camera_name': 'side_right',
                'brightness': -1,  # 왼쪽 캠과 동일 이유 — 위 주석 참고.
            }],
        ),
    ])
