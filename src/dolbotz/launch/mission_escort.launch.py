"""5구간(정찰·동행) 미션 — escort_follow 단독.

공용 주행 인지나 side_cameras.launch.py를 쓰지 않는다. 이 미션은 지형
추종이 아니라 선도 로봇과의 상대위치(목표 간격 2m ± 0.5m)를
유지하는 것이 핵심이라 문제 성격 자체가 다르다 — escort_follow_node가
드라이브캠(정면, 컬러+뎁스)을 직접 구독해서 선도 로봇을 추종하는 독립
파이프라인이다.

다른 mission_*.launch.py와 동일하게 카메라 드라이버(RealSense)는 이 launch
파일이 브링업하지 않는다 — 대회 실행 시 별도로 먼저 띄운다(howtorun.md 참고).

[2026-08-31 신규 -> 2026-09-01, 사용자 지시로 단일화 복귀] track_lost_grace_sec
(escort_follow.py 파라미터 선언부 주석 참고)를 실행과 동시에 바꿀 수 있게
launch 인자로 노출했다. 한때 화면 밖 이탈/가림막 구분용으로
track_lost_grace_sec_edge/_center 두 개로 나눴었는데(0115d3b), 팀원이 이
값을 0.0/0.0으로 바꿨다가(ee56c3b) "조향이 아예 안 됨" 증상의 유력 원인으로
지목된 전례가 있어 단일 파라미터로 되돌렸다(escort_follow.py 모듈 docstring
참고).

[2026-09-03] 재포착(reacquire) 튜닝용 3개도 같은 방식으로 launch 인자로
노출 -- 코드 기본값(escort_follow.py 파라미터 선언부)과 동일값을 기본으로
둠:
  - reacquire_radius_m(0.5) -- 재발견 게이팅 반경 기저값, 올리면 초반
    반응이 빨라짐.
  - reacquire_radius_growth_m_per_sec(0.0) -- 게이팅 반경이 시간당
    넓어지는 속도, 올리면 급가속/급선회 후 재발견을 덜 놓침. [주의] 예전
    문서에 "기본 0.3"으로 적혀 있었으나 커밋 5cf0710에서 0.0으로 수정됨 --
    실제 코드 기본값은 0.0.
  - velocity_smoothing_alpha(0.5) -- 속도 추정 지수평활 계수(1에 가까울수록
    최신 값에 더 민감).
거리(distance_speed_max_m_s)/바퀴속도(max_wheel_speed_dps) 상한은
escort_follow가 아니라 주행 쪽(purepursuit) 파라미터라 이 launch엔 없다 --
mission_escort_drive.launch.py -> purepursuit.launch.py 참고.

사용:
    ros2 launch dolbotz mission_escort.launch.py
    ros2 launch dolbotz mission_escort.launch.py track_lost_grace_sec:=0.5 reacquire_radius_growth_m_per_sec:=0.3 reacquire_radius_m:=0.8 velocity_smoothing_alpha:=0.7
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    track_lost_grace_sec = LaunchConfiguration('track_lost_grace_sec')
    reacquire_radius_m = LaunchConfiguration('reacquire_radius_m')
    reacquire_radius_growth_m_per_sec = LaunchConfiguration(
        'reacquire_radius_growth_m_per_sec')
    velocity_smoothing_alpha = LaunchConfiguration('velocity_smoothing_alpha')

    return LaunchDescription([
        # [2026-09-01] escort_follow.py 파라미터 선언부 주석 참고 -- 기본값은
        # 코드 기본값과 동일하게 맞춰둠.
        DeclareLaunchArgument(
            'track_lost_grace_sec',
            default_value='0.3',
            description=(
                "탐지 실패 후 이 시간[s] 이내는 등속 외삽으로 버티고(PREDICTING), "
                "넘기면 LOST로 넘어가 빈 Path를 발행해 정지시킨다. 실측 전 "
                "placeholder."
            ),
        ),
        # [2026-09-03] escort_follow.py 코드 기본값(0.5)과 동일하게 맞춰둠.
        DeclareLaunchArgument(
            'reacquire_radius_m',
            default_value='0.5',
            description=(
                "재발견(REACQUIRING) 게이팅 반경[m] 기저값 -- 이보다 가까운 "
                "재검출만 같은 트랙으로 인정한다. 올리면 초반 반응이 빨라지는 "
                "대신 다른 물체를 오인식할 여지도 커진다."
            ),
        ),
        # [2026-09-03] escort_follow.py 코드 기본값(0.0)과 동일하게 맞춰둠.
        DeclareLaunchArgument(
            'reacquire_radius_growth_m_per_sec',
            default_value='0.0',
            description=(
                "재발견 게이팅 반경이 LOST 경과 시간에 비례해 넓어지는 "
                "속도[m/s]. 급가속/급선회 직후 재발견을 덜 놓치게 하려면 "
                "올린다. 0.0이면 반경이 reacquire_radius_m으로 고정."
            ),
        ),
        # [2026-09-03] escort_follow.py 코드 기본값(0.5)과 동일하게 맞춰둠.
        DeclareLaunchArgument(
            'velocity_smoothing_alpha',
            default_value='0.5',
            description=(
                "선도 로봇 속도 추정 지수평활 계수(0~1). 1에 가까울수록 "
                "최신 관측치에 더 민감하게(노이즈에도 더 민감하게) 반응한다."
            ),
        ),

        Node(
            package='dolbotz',
            executable='escort_follow',
            name='escort_follow_node',
            output='screen',
            parameters=[{
                'track_lost_grace_sec': ParameterValue(
                    track_lost_grace_sec, value_type=float),
                'reacquire_radius_m': ParameterValue(
                    reacquire_radius_m, value_type=float),
                'reacquire_radius_growth_m_per_sec': ParameterValue(
                    reacquire_radius_growth_m_per_sec, value_type=float),
                'velocity_smoothing_alpha': ParameterValue(
                    velocity_smoothing_alpha, value_type=float),
            }],
        ),
    ])
