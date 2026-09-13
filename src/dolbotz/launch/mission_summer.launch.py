"""여름 미션 — perception_common(segmentation) + side_cameras(좌/우 사이드캠) +
flat_drive + elevation_map + gradient_map + slope_decision + summer_traffic +
summer_supply.

summer_supply는 사이드캠이 아니라 로봇팔 카메라(/arm/camera/...)를 쓰므로
side_cameras.launch.py와는 무관하다 — side_cameras는 summer_traffic(좌측
사이드캠 단일)용으로만 쓰인다.

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
    # config/models/trafficlightv1.pt — 실행 환경(colcon install)에서의 기본
    # 위치. 소스에 절대경로로 리터럴 박지 않고 launch 인자로 노출하는 건
    # force_mode 하드코딩 사고 이후의 프로젝트 관례(mission_spring.launch.py의
    # led_port/ifof_model_path 주석 참고) — 머신마다 모델 파일을 다른 경로에
    # 둬야 하면 이 인자로 오버라이드하면 된다.
    default_traffic_model_path = os.path.join(
        get_package_share_directory('dolbotz'),
        'config', 'models', 'trafficlightv1.pt')
    enable_visualizer = LaunchConfiguration('enable_visualizer')
    traffic_model_path = LaunchConfiguration('traffic_model_path')
    supply_stop_base_x_m = LaunchConfiguration('supply_stop_base_x_m')
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
            'traffic_model_path',
            default_value=default_traffic_model_path,
            description=(
                "summer_traffic_node의 신호등 YOLO 모델 경로. 기본값은 "
                "config/models/trafficlightv1.pt(summer_traffic.py의 "
                "TRAFFIC_MODEL_RELATIVE_PATH와 동일한 파일) — 다른 경로의 "
                "모델을 쓰려면 이 인자로 오버라이드할 것."
            ),
        ),
        DeclareLaunchArgument(
            'supply_stop_base_x_m',
            default_value='0.32',
            description=(
                '로봇팔 카메라 supplybox 좌표의 base_actuator 기준 |X|가 '
                '이 값(m) 이하로 연속 확인되면 /arm/target_point(IK 목표)를 전달.'
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
            parameters=[{
                # 여름 미션에서만 트랙 안쪽에 놓인 구조물(램프 등)이
                # row별 마스크를 두 세그먼트로 끊어서, 픽셀 수가 많은 쪽으로
                # centroid가 쏠리는(경로가 한쪽으로 치우침) 문제가 실기에서
                # 확인돼 lateral_gap_correction을 여기서만 켠다 — depth 신호가
                # 없어서 "진짜 구조물 vs 원래 좁아지는 트랙"을 구분 못 하므로
                # 다른 미션에는 기본값(False)인 채로 두고 안 켬.
                'lateral_gap_correction': True,
                # BEV 하단(로봇 근처)에 segmentation 유효 row가 없어서
                # /path가 통째로 안 나가거나 로봇 바로 앞 구간이 빈 채로
                # 나가는 문제 대응(사용자 요청, 2026-09-05) — 로봇(BEV 하단
                # 중앙, 0,0)에서 자갈 너머 첫 신뢰 가능 경로 구간까지 직선
                # 연결한다(extend_path_to_robot 참고). lateral_gap_correction과
                # 같은 이유로 여름 미션에만 켠다 — 다른 미션은 기본값(False)인
                # 채 안 건드림.
                'extend_path_to_robot': True,
                # 자갈 구간처럼 트랙 중간에 큰 segmentation 공백이 있어도
                # (실제로는 주행 가능한 구간이라는 걸 아는 여름 미션이므로)
                # truncate_at_large_gap()으로 그 지점부터 경로를 끊지 않고
                # bridge_interior_gaps()로 앞뒤를 이어붙인다(사용자 요청,
                # 2026-09-05). 다른 미션은 기본값(False)인 채 안 건드림 —
                # 실측 근거 없는 구간을 지어내면 안 되는 게 기본이므로.
                'bridge_interior_gaps': True,
                # 화면 전체에서 segmentation이 완전히 사라지는 짧은 순간
                # (자갈이 화면을 통째로 덮는 등) 대응 — odom/IMU로 좌표를
                # 갱신하지 않고 마지막 정상 경로를 이 시간(초) 동안만 그대로
                # 재사용한다. 그보다 오래 안 돌아오면 빈 Path로 안전 정지
                # (사용자 요청, 2026-09-05 — 직전 경로 무기한 사용 금지).
                # 다른 미션은 기본값(0.0=비활성)인 채 안 건드림.
                'hold_last_path_sec': 1.0,
            }],
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
                # 여름 미션에서만 매니퓰레이터 그리퍼/기타 근접
                # 장애물이 카메라 하단 시야를 가리는 게 확인돼 dynamic_occlusion_guard
                # 를 여기서만 켠다 — 다른 미션(mission_winter 등)은 이 파라미터를
                # 안 건드리므로 기본값(False)인 채 기존과 동일하게 동작.
                'dynamic_occlusion_guard': True,
                # 기본 threshold=0.3은 그리퍼 절벽(실측 181행)보다
                # 훨씬 이른 166행부터 잘라버려서 ROI 밴드가 너무 좁아지고,
                # sample_step 기본값(8)과 겹치면 median_height()의 80개 유효
                # 샘플 문턱을 그리퍼 유무와 무관하게 항상 밑도는 문제가 실기
                # (jecs)에서 확인됨 — /path, /drive/status 완전 무발행으로
                # 이어짐. threshold를 0.1로 낮춰 실제 절벽 근처까지만 자르고,
                # sample_step을 3으로 낮춰 격자 밀도(=절대 유효 샘플 수)를
                # 올려서 해결 확인(실기 hz 15Hz 검증 완료). 다른 관제/날씨
                # 조건에서 재검증 필요하면 이 값들부터 의심할 것.
                'occlusion_fill_ratio_threshold': 0.1,
                'sample_step': 3,
                # flat_drive_node의 lateral_gap_correction과 같은
                # 이유(트랙 안쪽 구조물) — 여기는 depth가 있어서 끊긴 지점의
                # depth 불연속으로 "진짜 구조물"인지 확인한 경우에만 복원한다
                # (find_lateral_gap()/reconstruct_track_span() 참고). 다른
                # 미션은 기본값(False)로 안 건드림.
                'lateral_occlusion_guard': True,
                # slope_decision의 전 구간 기본값과 동일하게 /path를
                # flat_drive 경로로 고정한다. 여름 launch에도 명시해 두어
                # 미션별 설정을 읽을 때 동작을 바로 확인할 수 있게 한다.
                'always_relay_flat': True,
            }],
        ),
        Node(
            package='dolbotz',
            executable='summer_traffic',
            name='summer_traffic_node',
            output='screen',
            parameters=[{
                'model_path': ParameterValue(traffic_model_path, value_type=str),
            }],
        ),
        Node(
            package='dolbotz',
            executable='summer_supply',
            name='summer_supply_node',
            output='screen',
            parameters=[{
                'stop_base_x_m': ParameterValue(
                    supply_stop_base_x_m, value_type=float),
            }],
        ),
        # 박스는 처음부터 가동범위 안에 배치한다. 차체 접근 없이 파지한다.
    ])
