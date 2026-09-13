"""상시 필요한 공용 인지 노드 — segmentation만 포함.

flat_drive/elevation_map/slope_decision/미션 노드들이 다 이 노드가 내는
/perception/drivable_mask 또는 컬러 이미지를 구독하므로, 계절별
mission_*.launch.py들이 이 launch를 include해서 먼저 띄운다.

segmentation.py의 추론 디바이스는 CPU로 고정되어 있으므로, 여기서
GPU/AUTO를 넘길 수 있던 경로 자체를 없앴다. 다시 GPU로 돌리고 싶으면
segmentation.py의 self._device 리터럴을 직접 고쳐야 한다.

[2026-09-03] snow_model_path 인자 추가 — 기본값 빈 문자열(비활성),
겨울 미션 전용(segmentation.py 모듈 docstring 참고). spring/summer/fall용
mission_*.launch.py는 이 include를 그대로 쓰되 이 인자를 안 넘기므로
기본값(비활성)이 유지되고, mission_winter.launch.py만 실제 경로를 넘긴다.

[2026-09-03] segmentation_node를 taskset -c 0,1,4,5(물리 코어 0,1)로
고정하는 건 spring/summer/fall(모델 1개, area만)에만 적용하고 겨울은
제외한다 — 겨울은 area+snow 두 모델을 프레임마다 순서대로 돌리는 구조라
(segmentation.py의 _on_image() 참고, area 다음에 snow 추론을 이어서
호출) 코어 2개로는 부족할 수 있다(트랙 구간과 눈 구간을 오가며 두 추론이
번갈아/같이 걸림). snow_model_path 값으로 분기 — 비어있으면(spring/
summer/fall) taskset 버전, 값이 있으면(겨울) 코어 제한 없는 버전을 각각
condition으로 둘 다 선언해두고 launch가 조건에 맞는 것 하나만 실행한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    snow_model_path = LaunchConfiguration('snow_model_path')
    # snow_model_path가 빈 문자열인지 여부로 spring/summer/fall(단일 모델)과
    # 겨울(area+snow 두 모델)을 가른다 -- 위 모듈 docstring 참고.
    no_snow = PythonExpression(["'", snow_model_path, "' == ''"])
    has_snow = PythonExpression(["'", snow_model_path, "' != ''"])

    # [2026-09-03] OpenVINO CPU 플러그인(TBB 링크 확인)이
    # PERFORMANCE_HINT=LATENCY로 컴파일되면 "한 번의 추론을 최대한 빨리
    # 끝내려고 논리 코어 전부를 그 한 요청에" 쓴다(ultralytics
    # OpenVINOBackend.load_model() 하드코딩, ov_config로 스레드 수를 조정하는
    # 훅이 노출돼 있지 않음). spring_ifof_node 등 다른 OpenVINO 노드와 동시에
    # 뜨면 서로 8코어를 다 쓰려고 경합해서, 격리 벤치마크(3.45~8.3ms,
    # config/models/README.md)보다 실측 infer가 20~30배(107~149ms) 느려지는
    # 걸 확인함(ps 상 segmentation_node 456% CPU). 모델/코드는 안 건드리고 OS
    # affinity로 물리 코어 0,1(=논리 0,1,4,5, i7-1165G7 4P/8T, `lscpu -e`로
    # HT 짝 확인)만 쓰게 캡 — 다른 프로세스와의 경합을 줄여서 오히려
    # 지연시간이 벤치마크에 가까워질 걸로 기대. 실측(vision_spring, 단일
    # area 모델): 375~456% -> 15.4%로 확인.
    common_kwargs = dict(
        package='dolbotz',
        executable='segmentation',
        name='segmentation_node',
        output='screen',
        parameters=[{
            'snow_model_path': snow_model_path,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'snow_model_path',
            default_value='',
            description=(
                "[겨울 미션 전용] segmentation_node가 area 모델과 함께 돌릴 "
                "눈(snow) 세그멘테이션 모델 경로. 빈 문자열(기본값)이면 "
                "비활성 — spring/summer/fall은 이 인자를 안 넘겨서 항상 "
                "비활성 상태다. segmentation.py 모듈 docstring 참고."
            ),
        ),
        Node(
            condition=IfCondition(no_snow),
            prefix='taskset -c 0,1,4,5',
            **common_kwargs,
        ),
        Node(
            condition=IfCondition(has_snow),
            **common_kwargs,
        ),
    ])
