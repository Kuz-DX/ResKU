"""
mission_escort_drive.launch.py

5구간(정찰·동행) 미션 전체 진입점 -- mission_escort.launch.py(escort_follow_node,
인지) + purepursuit.launch.py(purepursuit_node, 제어)를 한 번에 묶고, escort
전용 거리 유지 파라미터를 기본값으로 박아둔다.

[왜 이 파일이 필요한가] purepursuit.launch.py의 use_distance_scaled_speed/
goal_tolerance_m/max_wheel_speed_dps는 escort_follow 전용 값(각각
true/2.0/600.0)으로 커맨드라인에서 매번 오버라이드해야 했다(purepursuit.py
모듈 docstring의 "수동 동기화 필요" 경고 참고) -- 실행할 때마다 사람이 그
세 값을 다 기억해서 붙여야 하는 게 실수하기 쉬운 지점이라, 이 launch
파일에 고정해서 `ros2 launch dolbotz mission_escort_drive.launch.py` 한
줄로 항상 같은 설정이 뜨게 한다.

[2026-08-30] escort_follow_node의 track_lost_grace_sec도 8.0 -> 0.3으로
줄임(escort_follow.py 파라미터 선언부 주석 참고) -- 선도 로봇(로봇개)이
시야에서 사라지면 등속 외삽으로 몇 초씩 계속 움직이지 않고 즉시 정지하도록
사용자가 재지시함. 재포착 후 벌어진 거리는 purepursuit_node의
use_distance_scaled_speed(distance_speed_max_m_s=1.8m/s 상한)가 가속해서
좁힌다 -- 두 변경이 한 세트로 맞물려 있어 같이 하나의 launch로 묶었다.

[2026-08-30] rmd_x8_driver_node 기반으로 바꿨다가(purepursuit_rmd.launch.py)
사용자 재검토로 롤백 -- can_driver_node 기반(purepursuit.launch.py)을 그대로
유지하기로 함.

[2026-08-31, 버그 수정] 이 launch가 use_bearing_steering/camera_pitch_rad/
max_linear_accel_mps2를 빠뜨리고 있었다 -- purepursuit.launch.py 모듈
docstring이 escort_follow 사용 예로 명시적으로 권장하는 오버라이드인데도
여기(실제로 대회에서 쓰는 원커맨드 진입점)엔 반영이 안 돼 있었다. 그 결과:
  - use_bearing_steering이 기본값(false)으로 떠서, 로봇개가 화면 외곽에
    잡혔을 때 조향이 거의 안 나가는 문제(표준 curvature의 한계, purepursuit.py
    모듈 docstring 참고)가 그대로 남아있었다 -- "로봇개가 화면 밖으로
    빠르게 빠져나가면 대응 못 함" 문제의 상당 부분이 사실 이 설정
    누락이었을 가능성이 큼.
    - camera_pitch_rad가 당시 트랙주행 마운트 각도로 남아있어서 escort용
      카메라 실측 각도와 안 맞는 채로 base_link 좌표 변환(간격/조향 계산에
      직접 씀)이 계속 됐다. 현재는 2026-09-03 최신 실측값으로 갱신했다.
이번에 셋 다 명시적으로 오버라이드하도록 고쳤다.

[2026-08-31 신규 -> 2026-09-01, 사용자 지시로 단일화 복귀] track_lost_grace_sec
(mission_escort.launch.py/escort_follow.py 참고)도 실행과 동시에 바꿀 수
있게 이 launch에서 그대로 통과시킨다. 한때 화면 밖 이탈 vs 가림막 구분용으로
track_lost_grace_sec_edge/_center 두 개로 나눴었는데, 팀원이 이 값을
0.0/0.0으로 바꿨다가 "조향이 아예 안 됨" 증상의 유력 원인으로 지목된 전례가
있어 단일 파라미터로 되돌렸다.

사용:
    ros2 launch dolbotz mission_escort_drive.launch.py
    ros2 launch dolbotz mission_escort_drive.launch.py can_channel:=vcan0  # dry-run
    ros2 launch dolbotz mission_escort_drive.launch.py track_lost_grace_sec:=2.0

전제: robot_bringup의 autonomous*.launch.py와 동시 실행 금지
(can_driver_node/rmd_x8_driver_node CAN 충돌, purepursuit.launch.py 상단
docstring 참고).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('dolbotz'), 'launch')
    can_channel = LaunchConfiguration('can_channel')
    track_lost_grace_sec = LaunchConfiguration('track_lost_grace_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_channel',
            default_value='can_drive',
            description="CAN 인터페이스 (실물 기본값 'can_drive'; 'vcan0'로 dry-run)",
        ),
        # [2026-09-01] mission_escort.launch.py로 그대로 통과 -- 기본값은
        # mission_escort.launch.py/escort_follow.py 코드 기본값과 동일하게
        # 맞춰둠.
        DeclareLaunchArgument(
            'track_lost_grace_sec',
            default_value='3.0',
            description=(
                "탐지 실패 후 이 시간[s] 이내는 등속 외삽으로 버티고, 넘기면 "
                "LOST로 넘어가 정지시킨다 -- mission_escort.launch.py 참고."
            ),
        ),

        # 인지 -- 선도 로봇(로봇개) 탐지 + /path 발행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'mission_escort.launch.py')),
            launch_arguments={
                'track_lost_grace_sec': track_lost_grace_sec,
            }.items(),
        ),

        # 제어 -- pure pursuit. escort 전용 값 고정:
        #   use_distance_scaled_speed=true  -- 거리 비례 P제어 켬(기본은 꺼짐)
        #   max_wheel_speed_dps=600.0       -- 선도로봇 최고속도(1.0m/s)보다
        #                                      확실히 빠르게(기본 360=0.707m/s로는
        #                                      원천적으로 못 따라잡음, 실측 확인됨)
        #   use_bearing_steering=true       -- [2026-08-31, 버그 수정으로 추가]
        #                                      화면 외곽에서도 조향이 실제로
        #                                      나가게 함(위 [2026-08-31, 버그
        #                                      수정] 절 참고) -- 이게 꺼진 채로
        #                                      있었던 게 "로봇개가 빠르게
        #                                      화면 밖으로 나가면 대응 못
        #                                      한다"는 문제의 주된 원인으로
        #                                      추정됨.
        #   camera_pitch_rad=0.2737         -- [2026-09-04 재실측] 드라이브
        #                                      카메라 내장 IMU 실측값
        #                                      (+15.68deg, std=0.0015rad).
        #                                      이전 실측(2026-09-03,
        #                                      0.2734)과 오차범위 내 동일.
        #   camera_roll_rad=0.0053          -- [2026-09-04 재실측] 같은 조건의
        #                                      내장 IMU 실측값(+0.30deg,
        #                                      std=0.0016rad). 이전 실측
        #                                      (2026-09-03, 0.0159)과 차이
        #                                      있어 최신값으로 갱신.
        #   max_linear_accel_mps2=3.0       -- [2026-08-31 신규] 정지/재포착
        #                                      직후 가속 지연 문제 대응 --
        #                                      기본값(1.0)으로는 0->1.8m/s
        #                                      (distance_speed_max_m_s)까지
        #                                      1.8초 걸려서 못 따라가는
        #                                      지연이 있었음. 감속은 전류
        #                                      보호 목적이라 기본값(1.0)
        #                                      그대로 안 건드림(max_linear_
        #                                      decel_mps2). 실측 전
        #                                      placeholder, 실기 튜닝 필요.
        #   use_lost_left_turn_recovery=true -- [2026-09-04, 사용자 요청]
        #                                      로봇개가 왼쪽 가장자리로
        #                                      빠지며(=놓치기 직전 좌회전
        #                                      중) 놓친 경우만 2초간
        #                                      제자리 좌회전하며 재포착
        #                                      시도, 직진 중 놓치면(가림막
        #                                      구간) 기존처럼 즉시 정지 --
        #                                      purepursuit.py 모듈 docstring
        #                                      의 use_lost_left_turn_recovery
        #                                      참고.
        #   use_outer_wheel_boost=true      -- [2026-09-03, 사용자 지시]
        #                                      slope_traverse_node.cpp의
        #                                      computeBoostedVx()를 그대로
        #                                      쓰는 차동 조향 -- 안쪽 바퀴는
        #                                      v_target 그대로 두고 바깥쪽만
        #                                      부스트한다. 이걸 켜면서
        #                                      purepursuit.py의 "목표각 크면
        #                                      제자리 회전(피벗 턴)부터"
        #                                      분기를 없앴다(_compute_pure_
        #                                      pursuit() 참고) -- 큰 각도에서
        #                                      v_target을 0으로 죽이던 게
        #                                      전진을 멈춰서 추종 거리가
        #                                      벌어지는 문제였음. 이제 큰
        #                                      각도도 전진하며 바깥쪽 바퀴
        #                                      부스트로 돈다.
        # distance_speed_gain(2.0)/distance_speed_max_m_s(1.8)/distance_band_min_m
        # (1.5)/distance_band_max_m(2.0)/bearing_steering_gain(3.0)/
        # bearing_deadband_lateral_m(0.8)는 노드 기본값이 이미 이 목표에
        # 맞춰져 있어 오버라이드 생략(purepursuit.py 모듈 docstring 참고).
        # bearing_inplace_threshold_rad는 위 피벗 턴 분기 제거로 더 이상 안
        # 쓰임(하위 호환용으로 파라미터 자체는 남겨둠). goal_tolerance_m은
        # [2026-08-30, 재조정]으로 escort에 더 이상 안 쓰여서(distance_band_*
        # 로 대체됨) 오버라이드를 뺐다 -- 이전엔 여기 남아있었지만 no-op였음.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'purepursuit.launch.py')),
            launch_arguments={
                'can_channel': can_channel,
                'use_distance_scaled_speed': 'true',
                'max_wheel_speed_dps': '600.0',
                'use_bearing_steering': 'true',
                'use_outer_wheel_boost': 'true',
                'camera_pitch_rad': '0.2737',
                'camera_roll_rad': '0.0053',
                'max_linear_accel_mps2': '3.0',
                'use_lost_left_turn_recovery': 'true',
            }.items(),
        ),
    ])
