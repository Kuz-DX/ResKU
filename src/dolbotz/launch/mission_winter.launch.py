"""겨울(주행) 미션 — perception_common(segmentation) + flat_drive + elevation_map
+ gradient_map + slope_decision.

elevation_map/gradient_map은 순서상 elevation_map -> gradient_map으로
입력이 이어지지만, 둘 다 노드 시작 시점에 구독만 걸어두고 이후 메시지가
들어오면 처리하는 구조라 launch에서 실행 순서를 강제할 필요는 없다.

[2026-09-03] 겨울 트랙(빙판길/제설 구간)에서 area 세그멘테이션 모델 단독
으로는 놓칠 수 있는 영역을 보강하려고, snow 세그멘테이션 모델
({0: 'snow'} 단일 클래스)을 이 미션에서만 같이 돌린다 —
perception_common.launch.py의 snow_model_path 인자로 전달,
segmentation_node 안에서 area 마스크와 OR로 합쳐 /perception/drivable_mask
하나로 그대로 나간다(재현율 우선, [2026-09-03 사용자 결정] —
segmentation.py 모듈 docstring 참고). 다른 계절 mission_*.launch.py는
이 인자를 안 넘기므로 영향 없음.

기본값은 OpenVINO INT8 export(config/models/snow_v1_int8_openvino_model,
imgsz=640 — area 모델의 320과 다름, /home/j/vision_marker/snow/
train_snow.py IMGSZ 확인). /home/j/vision_marker/snow/snow-1(원본 학습
데이터셋 그 자체)로 캘리브레이션/검증: mask mAP50 0.99415->0.98163,
mask mAP50-95 0.97782->0.93203, recall 0.98462->0.92271(다른 모델들보다
하락 폭이 좀 더 크지만 OR 결합이라 area 모델이 보완함) — 추론시간은
68.74ms->13.31ms(약 5.2배). 이전 FP32로 되돌리려면 아래
default_snow_model_path 정의에서 주석 처리해둔 줄로 바꿀 것
(config/models/README.md 참고).
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
    default_snow_model_path = os.path.join(
        get_package_share_directory('dolbotz'),
        'config', 'models', 'snow_v1_int8_openvino_model')
    # default_snow_model_path = os.path.join(
    #     get_package_share_directory('dolbotz'),
    #     'config', 'models', 'snow_v1.pt')
    enable_visualizer = LaunchConfiguration('enable_visualizer')
    snow_model_path = LaunchConfiguration('snow_model_path')
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
            'snow_model_path',
            default_value=default_snow_model_path,
            description=(
                "segmentation_node가 area 모델과 함께 돌릴 눈(snow) "
                "세그멘테이션 모델 경로 — 기본값 "
                "config/models/snow_v1_int8_openvino_model(imgsz=640, "
                "area 모델의 320과 다름). 겨울 미션 전용, 모듈 docstring 참고."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_common),
            launch_arguments={'snow_model_path': snow_model_path}.items()),
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
    ])
