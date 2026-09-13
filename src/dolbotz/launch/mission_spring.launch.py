"""봄 미션 — perception_common(segmentation) + side_cameras(좌/우 사이드캠) +
flat_drive + elevation_map + gradient_map + slope_decision + spring_ifof
+ led_bridge(시리얼->아두이노).

elevation_map/gradient_map은 순서상 elevation_map -> gradient_map으로
입력이 이어지지만, 둘 다 노드 시작 시점에 구독만 걸어두고 이후 메시지가
들어오면 처리하는 구조라 launch에서 실행 순서를 강제할 필요는 없다.

노드 이름(name=)은 mission_winter.launch.py와 동일하다(flat_drive_node,
elevation_map_node, gradient_map_node, side_slope_trigger_node) — 같은
executable을 여러 launch 파일이 공유하지만, 한 번에 하나의 mission_*.launch.py만
실행하는 게 정상 운영 방식이라 이름 충돌은 없다. 다만 나중에 여러
mission_*.launch.py를 한 프로세스/네임스페이스에서 동시에 include하는
시나리오가 생기면 이 노드들이 이름 충돌로 죽을 수 있으니 주의할 것.

[예외 케이스 안내] spring_ifof_node -> led_bridge_node 체인은 봄 미션
(피아식별)만의 예외 — 이 미션에 한해 인식부터 LED 하드웨어 제어(아두이노)
까지 한 사람이 전 구간을 담당하기로 했다(dolbotz/missions/spring_ifof.py
상단 docstring 참고). 다른 계절 미션 launch 파일에는 이런 하드웨어 직결
체인이 없다. [2026-08-30] 원래는 spring_ifof_node -> led_relay_node ->
led_bridge_node 3단이었는데, spring_ifof_node가 판정(디바운싱+락)까지
이미 하고 있어서 led_relay_node 없이 spring_ifof_node가 /led_control을
직접 발행하도록 단순화함 — 그래서 이 launch에는 led_relay_node가 없다
(led_relay.py 파일 자체는 매핑 함수 재사용을 위해 남아있음, 그 파일
docstring 참고).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    perception_common = os.path.join(
        get_package_share_directory('dolbotz'),
        'launch', 'perception_common.launch.py')
    side_cameras = os.path.join(
        get_package_share_directory('dolbotz'),
        'launch', 'side_cameras.launch.py')
    # config/models/ifofv1.pt — 실행 환경(colcon install)에서의 기본 위치.
    # 소스에 절대경로로 리터럴 박지 않고 launch 인자로 노출하는 건
    # force_mode 하드코딩 사고 이후의 프로젝트 관례(mission_summer.launch.py의
    # traffic_model_path와 같은 패턴) — 머신마다 모델 파일을 다른 경로에
    # 둬야 하면 이 인자로 오버라이드하면 된다.
    default_ifof_model_path = os.path.join(
        get_package_share_directory('dolbotz'),
        'config', 'models', 'ifofv1.pt')
    enable_visualizer = LaunchConfiguration('enable_visualizer')
    led_port = LaunchConfiguration('led_port')
    ifof_model_path = LaunchConfiguration('ifof_model_path')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_visualizer',
            default_value='false',
            description=(
                "slope_decision의 OpenCV 디버그 창(ROI+슬로프 표시) 사용 여부. "
                "헤드리스 환경(SSH, 무헤드 로봇 본체 등)을 위해 기본값은 false. "
                "GUI 환경에서 "
                "디버그 창 보고 싶으면 'true'로 넘길 것 — 꺼도 /path, "
                "/terrain/side_slope_angle_deg, /drive/status 토픽 발행은 그대로다."
            ),
        ),
        DeclareLaunchArgument(
            'led_port',
            default_value='/dev/LED',
            description=(
                "LED 아두이노 시리얼 포트. udev 규칙으로 /dev/LED 고정 심볼릭 "
                "링크를 만들어뒀다는 전제 — 안 만들었으면 `ls /dev/ttyACM*`/"
                "`ls /dev/ttyUSB*`로 실제 장치를 확인해서 이 인자로 오버라이드할 "
                "것(재부팅/재연결 시 번호가 바뀔 수 있음, side_cameras.launch.py의 "
                "video_device 주석과 같은 이유). 소스에 리터럴로 안 박고 launch "
                "인자로 노출한 건 force_mode 하드코딩 사고 이후의 프로젝트 관례."
            ),
        ),
        DeclareLaunchArgument(
            'ifof_model_path',
            default_value=default_ifof_model_path,
            description=(
                "spring_ifof_node의 YOLO 모델 경로. 기본값은 "
                "config/models/ifofv1.pt(spring_ifof.py의 "
                "IFOF_MODEL_RELATIVE_PATH와 동일한 파일, 아군/적군 판별 전용 "
                "임시 모델) — 다른 경로의 모델을 쓰려면 이 인자로 오버라이드할 것."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_common)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(side_cameras)),
        Node(
            package='dolbotz',
            executable='flat_drive',
            name='flat_drive_node',
            output='screen',
        ),
        Node(
            package='dolbotz',
            executable='elevation_map',
            name='elevation_map_node',
            output='screen',
        ),
        Node(
            package='dolbotz',
            executable='gradient_map',
            name='gradient_map_node',
            output='screen',
        ),
        Node(
            package='dolbotz',
            executable='slope_decision',
            name='side_slope_trigger_node',
            output='screen',
            parameters=[{
                'enable_visualizer': ParameterValue(enable_visualizer, value_type=bool),
            }],
        ),
        Node(
            package='dolbotz',
            executable='spring_ifof',
            name='spring_ifof_node',
            output='screen',
            parameters=[{
                'model_path': ParameterValue(ifof_model_path, value_type=str),
            }],
        ),

        # [예외 케이스] led_bridge_node가 spring_ifof_node의 /led_control을
        # 받아 실제 LED 하드웨어로 내보낸다 — 파일 상단 docstring 참고
        # (led_relay_node는 더 이상 여기서 안 띄움).
        Node(
            package='dolbotz',
            executable='led_bridge_node',
            name='led_bridge_node',
            output='screen',
            parameters=[{
                'port': ParameterValue(led_port, value_type=str),
            }],
        ),
    ])
