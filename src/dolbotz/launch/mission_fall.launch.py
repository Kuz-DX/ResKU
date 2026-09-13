"""가을 미션 — perception_common(segmentation) + side_cameras(좌/우 사이드캠) +
flat_drive + elevation_map + gradient_map + slope_decision + fall_marker.

elevation_map/gradient_map은 순서상 elevation_map -> gradient_map으로
입력이 이어지지만, 둘 다 노드 시작 시점에 구독만 걸어두고 이후 메시지가
들어오면 처리하는 구조라 launch에서 실행 순서를 강제할 필요는 없다.

노드 이름(name=)은 mission_winter.launch.py와 동일하다(flat_drive_node,
elevation_map_node, gradient_map_node, side_slope_trigger_node) — 같은
executable을 여러 launch 파일이 공유하지만, 한 번에 하나의 mission_*.launch.py만
실행하는 게 정상 운영 방식이라 이름 충돌은 없다. 다만 나중에 여러
mission_*.launch.py를 한 프로세스/네임스페이스에서 동시에 include하는
시나리오가 생기면 이 노드들이 이름 충돌로 죽을 수 있으니 주의할 것.
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
    # config/models/vision_marker_final_int8_openvino_model — 실행 환경에서의
    # 기본 위치. [2026-09-05] vision_marker_final.pt(구 vision_makerv4.pt,
    # 사용자 요청으로 반입)를 OpenVINO INT8로 변환(이전 vision_marker.v3.pt
    # 교체, fall_marker.py 모듈 docstring 참고) — 11클래스 모델이라
    # fall_marker.py 쪽에서 8종 마커만 필터링해서 쓴다. 소스에
    # 절대경로로 리터럴 박지 않고 launch 인자로 노출하는 건
    # force_mode 하드코딩 사고 이후의 프로젝트 관례(mission_summer.launch.py의
    # traffic_model_path, mission_spring.launch.py의 ifof_model_path와 같은
    # 패턴) — 머신마다 모델 파일을 다른 경로에 둬야 하면 이 인자로
    # 오버라이드하면 된다.
    default_marker_model_path = os.path.join(
        get_package_share_directory('dolbotz'),
        'config', 'models', 'vision_marker_final_int8_openvino_model')
    enable_visualizer = LaunchConfiguration('enable_visualizer')
    marker_model_path = LaunchConfiguration('marker_model_path')
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
            'marker_model_path',
            default_value=default_marker_model_path,
            description=(
                "fall_marker_node의 YOLO 마커 모델 경로. 기본값은 "
                "config/models/vision_marker_final_int8_openvino_model"
                "(fall_marker.py의 MARKER_MODEL_RELATIVE_PATH와 동일한 파일) — "
                "다른 경로의 "
                "모델을 쓰려면 이 인자로 "
                "오버라이드할 것."
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
            executable='fall_marker',
            name='fall_marker_node',
            output='screen',
            parameters=[{
                'model_path': ParameterValue(marker_model_path, value_type=str),
            }],
        ),
    ])
