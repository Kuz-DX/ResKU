"""가을 미션 — side_cameras + fall_marker."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
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
    marker_model_path = LaunchConfiguration('marker_model_path')
    return LaunchDescription([
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
            PythonLaunchDescriptionSource(side_cameras)),
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
