"""
purepursuit.launch.py

Pure Pursuit로 MPPI(controller_server) 역할을 대체하는 최소 구성. 아래
세 가지만 띄우며, `/path`를 내는 경로 소스는 별도로 실행해야 한다:

    1) base_link -> imu_link / camera_link 정적 TF (reduced_odom_bringup.launch.py의 CAD 실측값과
       동일 -- purepursuit_node가 /path(camera_link)를 base_link로 변환하는 데 필요)
    2) can_driver_node (can_driver 패키지, manual.launch.py가 조종 모드에서 쓰는
       그 노드) -- /motor_speed_cmd(dps) -> CAN
    3) purepursuit_node (dolbotz 패키지) -- /path -> pure pursuit -> 스키드조향
       역기구학 -> /motor_speed_cmd

[중요] reduced_odom_bringup.launch.py를 통째로 include하지 않는다 -- 그 파일은 정적 TF와
같이 rmd_x8_driver_node(RMD-X8 프로토콜 CAN 드라이버)도 띄우는데, 그건
can_driver_node와 똑같은 CAN 버스(can_drive)의 똑같은 CAN ID(left=1/right=2)를
잡고 서로 다른 속도 명령을 동시에 쏘게 된다 -- 두 드라이버가 같은 모터를
동시에 제어하려는 충돌이라 실물에서 위험하다. purepursuit_node 자체는
/odometry/filtered(EKF)도 안 쓴다 (/path가 매 프레임 로봇 기준 상대좌표로
갱신되기 때문, purepursuit.py 모듈 docstring 참고) -- 그래서 myahrs_driver_node/
ekf_node도 필요 없어 정적 TF 두 개만 뽑아왔다.

전제: robot_bringup의 autonomous.launch.py(reduced_odom_bringup.launch.py를 통해
rmd_x8_driver_node가 이미 떠 있음)와 동시에 띄우지 않는다. 둘 다 뜨면 위에서
말한 CAN 충돌이 그대로 발생한다.

사용:
    ros2 launch dolbotz purepursuit.launch.py
    ros2 launch dolbotz purepursuit.launch.py can_channel:=vcan0  # dry-run

    [2026-08-27 신규] 경사 주행 모드(안쪽 바퀴는 그대로, 바깥쪽만 부스트하는
    차동조향, purepursuit.py 모듈 docstring/slope_traverse_node.cpp의
    computeBoostedVx() 참고):
    ros2 launch dolbotz purepursuit.launch.py use_outer_wheel_boost:=true target_linear_speed_m_s:=0.4

    [2026-08-30 신규, 2026-08-30 대회 규정 반영] escort_follow(5구간, 선도로봇
    추종) 전용 거리 비례 속도 — 기본 꺼짐(다른 미션 영향 없음), purepursuit.py
    모듈 docstring 참고. [2026-08-30, 재조정] 예전엔 goal_tolerance_m을
    "이 정도 거리를 유지"하는 setpoint로 재사용했는데, 그러면 후진 없이
    "가까우면 그냥 정지"만 해서 대회 규정 하한(1.5m)을 밑도는 걸 막을
    방법이 없었다 — distance_band_min_m/distance_band_max_m(기본 1.5/2.0,
    2026-08-30 재조정: 상한 2.5->2.0, 대회 규정 목표 간격 그대로) 밴드로
    바뀌었고, goal_tolerance_m은 이제
    escort_follow에 안 씀(distance_speed_gain/max_m_s는 노드 기본값 2.0/1.8이
    이미 이 목표에 맞춰져 있어 생략 가능). [중요] max_wheel_speed_dps(기본
    360, 아래 참고)도 같이 올려야 실제로 효과가 있다 — 360dps는
    wheel_radius_m=0.1125 기준 최대 선속도 0.707m/s로 클램프되는데, 이건
    선도로봇 최고속도(로봇개 1.0m/s)보다 낮아서 이 파라미터를 안 올리면 위
    distance_speed_* 튜닝과 무관하게 원천적으로 못 따라잡는다(2026-08-30
    실측으로 확인됨). 600dps ≈ 1.18m/s로, 선도로봇보다는 빠르되 기존 수동
    조종 검증 범위(800dps) 안에 안전하게 들어오는 값으로 사용자와 협의해
    정함. use_bearing_steering(로봇개가 화면 외곽에 잡혔을 때 조향이 거의
    안 나가는 문제 대응)도 같이 켤 것. [2026-09-03 실측] 드라이브 카메라
    IMU 기준 pitch=+0.2734rad, roll=+0.0159rad이고 base_link 기준 위치는
    x=+0.090461m, y=0, z=+0.805188m다. 아래 정적 TF와 기본 launch 인자는
    이 값으로 통일한다:
    ros2 launch dolbotz purepursuit.launch.py use_distance_scaled_speed:=true max_wheel_speed_dps:=600.0 use_bearing_steering:=true camera_pitch_rad:=0.2737 camera_roll_rad:=0.0053
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    can_channel = LaunchConfiguration('can_channel')
    use_outer_wheel_boost = LaunchConfiguration('use_outer_wheel_boost')
    target_linear_speed_m_s = LaunchConfiguration('target_linear_speed_m_s')
    lookahead_distance_m = LaunchConfiguration('lookahead_distance_m')
    goal_tolerance_m = LaunchConfiguration('goal_tolerance_m')
    use_distance_scaled_speed = LaunchConfiguration('use_distance_scaled_speed')
    distance_speed_gain = LaunchConfiguration('distance_speed_gain')
    distance_speed_min_m_s = LaunchConfiguration('distance_speed_min_m_s')
    distance_speed_max_m_s = LaunchConfiguration('distance_speed_max_m_s')
    distance_band_min_m = LaunchConfiguration('distance_band_min_m')
    distance_band_max_m = LaunchConfiguration('distance_band_max_m')
    distance_speed_reverse_max_m_s = LaunchConfiguration('distance_speed_reverse_max_m_s')
    max_wheel_speed_dps = LaunchConfiguration('max_wheel_speed_dps')
    max_linear_accel_mps2 = LaunchConfiguration('max_linear_accel_mps2')
    max_linear_decel_mps2 = LaunchConfiguration('max_linear_decel_mps2')
    max_angular_accel_rad_s2 = LaunchConfiguration('max_angular_accel_rad_s2')
    use_bearing_steering = LaunchConfiguration('use_bearing_steering')
    bearing_steering_gain = LaunchConfiguration('bearing_steering_gain')
    bearing_inplace_threshold_rad = LaunchConfiguration('bearing_inplace_threshold_rad')
    bearing_deadband_lateral_m = LaunchConfiguration('bearing_deadband_lateral_m')
    use_single_wheel_pivot = LaunchConfiguration('use_single_wheel_pivot')
    use_lost_left_turn_recovery = LaunchConfiguration('use_lost_left_turn_recovery')
    lost_left_turn_w_threshold_rad_s = LaunchConfiguration('lost_left_turn_w_threshold_rad_s')
    lost_left_turn_w_rad_s = LaunchConfiguration('lost_left_turn_w_rad_s')
    lost_left_turn_bearing_gain = LaunchConfiguration('lost_left_turn_bearing_gain')
    lost_left_turn_duration_sec = LaunchConfiguration('lost_left_turn_duration_sec')
    camera_pitch_rad = LaunchConfiguration('camera_pitch_rad')
    camera_roll_rad = LaunchConfiguration('camera_roll_rad')

    return LaunchDescription([

        # [2026-09-01 신규] purepursuit_node 파라미터 전체를 문서화/정리한
        # yaml -- config/purepursuit_params.yaml 상단 주석 참고. Node의
        # parameters 리스트에서 이 파일을 먼저 넣고 그 뒤에 기존 CLI
        # (DeclareLaunchArgument) dict를 이어붙이므로, 이 yaml에 있고 CLI
        # 인자로도 노출된 키는 커맨드라인이 항상 이긴다(기존 동작 100%
        # 그대로 유지) -- max_linear_accel_while_turning_mps2처럼 CLI 인자로
        # 안 뺀 키만 이 파일 값이 실제로 적용된다. 실험 버전마다 이런 값만
        # 바꾼 yaml을 만들어서 params_file:=<경로>로 통째로 스왑하는 용도.
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('dolbotz'), 'config', 'purepursuit_params.yaml'),
            description=(
                "purepursuit_node 파라미터 yaml 경로 -- config/"
                "purepursuit_params.yaml 상단 주석 참고. CLI 인자로 노출된 "
                "키는 이 파일 값을 덮어쓴다(커맨드라인이 항상 이김)."
            ),
        ),
        DeclareLaunchArgument(
            'can_channel',
            default_value='can_drive',
            description="CAN 인터페이스 (실물 기본값 'can_drive'; 'vcan0'로 dry-run)",
        ),

        # [2026-08-27 신규] 경사 주행 모드 -- 기본값(false/0.3/0.5)은 기존
        # 평지 주행과 동일하게 유지, 경사 주행 시 커맨드라인에서 오버라이드.
        # target_linear_speed_m_s=0.4는 slope_traverse_node.cpp의 climb_speed
        # 확정값과 동일(안쪽 바퀴 목표 속도, 약 200dps).
        DeclareLaunchArgument(
            'use_outer_wheel_boost',
            default_value='false',
            description=(
                "true면 회전 시 안쪽 바퀴를 깎는 대신 바깥쪽만 부스트(경사 "
                "주행용, 매뉴얼 실측 300/200dps 조합 참고). false면 기존 "
                "대칭 스큐-스티어 그대로."
            ),
        ),
        DeclareLaunchArgument(
            'target_linear_speed_m_s',
            default_value='0.3',
            description="목표 선속도[m/s] -- 경사 주행 시 안쪽 바퀴 목표(slope_traverse_node climb_speed와 동일하게 0.4 권장)",
        ),
        DeclareLaunchArgument(
            'lookahead_distance_m',
            default_value='0.5',
            description="pure pursuit lookahead 거리[m]",
        ),
        DeclareLaunchArgument(
            'goal_tolerance_m',
            default_value='0.3',
            description=(
                "경로 끝점까지 이 거리 이내면 도착으로 보고 정지. "
                "use_distance_scaled_speed=true일 땐 거리 비례 속도의 "
                "setpoint 역할도 겸함(escort_follow용, 아래 참고)."
            ),
        ),

        # [2026-08-30 신규] escort_follow 전용, 기본 꺼짐 — 모듈 docstring
        # 참고. 일반적인 긴 경로에서는 켜지
        # 말 것.
        DeclareLaunchArgument(
            'use_distance_scaled_speed',
            default_value='false',
            description=(
                "true면 목표점까지 거리(goal_tolerance_m 기준 P제어)로 "
                "속도를 정한다 -- escort_follow(2점짜리 경로)에서만 의미가 "
                "맞음. false(기본)면 기존과 동일하게 target_linear_speed_m_s "
                "고정속도."
            ),
        ),
        DeclareLaunchArgument(
            'distance_speed_gain',
            default_value='2.0',
            description=(
                "거리 비례 속도 P게인[m/s per m] -- 대회 규정(간격 1.5~2.0m, "
                "선도로봇 최고속도 1.0m/s) 기준. [2026-08-30, 재조정] 상한이 "
                "규정값(2.0)과 같아져서 선도 이동 중엔 정상상태 거리가 항상 "
                "2.0m를 살짝 넘는다 -- purepursuit.py 모듈 docstring의 "
                "distance_speed_gain 설명 참고. 실측 전 placeholder."
            ),
        ),
        DeclareLaunchArgument(
            'distance_speed_min_m_s',
            default_value='0.0',
            description="거리 비례 속도 하한[m/s] -- 실측 전 placeholder.",
        ),
        DeclareLaunchArgument(
            'distance_speed_max_m_s',
            default_value='1.8',
            description=(
                "거리 비례 속도 상한[m/s](따라잡기 캡) -- 선도로봇 최고속도"
                "(1.0m/s)보다 반드시 커야 함, 실측 전 placeholder."
            ),
        ),
        # [2026-08-30 신규, 2026-08-30 재조정: 상한 2.5->2.0] 대회 규정 목표
        # 간격 1.5~2.0m를 그대로 밴드로 씀 -- purepursuit.py 모듈 docstring 참고.
        DeclareLaunchArgument(
            'distance_band_min_m',
            default_value='1.5',
            description="목표거리 밴드 하한[m] -- 이보다 가까우면 후진(distance_speed_reverse_max_m_s로 캡).",
        ),
        DeclareLaunchArgument(
            'distance_band_max_m',
            default_value='2.0',
            description=(
                "목표거리 밴드 상한[m] -- 이보다 멀면 전진(distance_speed_max_m_s로 캡). "
                "규정 상한과 같은 값이라 선도 이동 중엔 정상상태 거리가 이보다 "
                "살짝 넘는 지점에서 수렴함(purepursuit.py 모듈 docstring 참고)."
            ),
        ),
        DeclareLaunchArgument(
            'distance_speed_reverse_max_m_s',
            default_value='0.3',
            description=(
                "밴드 하한보다 가까울 때 후진 속도 상한[m/s]. [안전 주의] "
                "드라이브 카메라는 전방만 봐서 후방 장애물 감지가 전혀 없음 "
                "-- 전진 캡보다 훨씬 보수적으로 잡혀있음, 실기 후방 여유 "
                "공간 확인 후 조정할 것."
            ),
        ),
        # [2026-08-30, 대회 규정 반영] 기본값(360)은 reduced_odom_bringup.launch.py의
        # 자율주행 공통 하드 클램프와 맞춘 값이라 wheel_radius_m=0.1125 기준
        # 최대 선속도가 0.707m/s로 제한됨 -- 다른 자율주행 미션(주로 0.3~0.4m/s
        # 주행)엔 충분하지만, escort_follow(선도로봇 0.8~1.0m/s 추종)엔 이
        # 기본값 그대로면 distance_speed_* 튜닝과 무관하게 원천적으로 못
        # 따라잡는다(실측 확인됨). escort 미션 실행 시에만 오버라이드할 것 --
        # 다른 미션에 쓰는 기본값(360)은 그대로 안전하게 유지.
        DeclareLaunchArgument(
            'max_wheel_speed_dps',
            default_value='360.0',
            description=(
                "좌우 바퀴 최대 각속도[dps] 하드 클램프 -- 기본값은 "
                "reduced_odom_bringup.launch.py의 자율주행 공통값과 동일(360, "
                "wheel_radius_m=0.1125 기준 0.707m/s). escort_follow처럼 "
                "0.707m/s를 넘는 목표 속도가 필요한 미션에서만 올려서 쓸 것 "
                "(예: 600 ≈ 1.18m/s, manual_joy_control_node 검증 범위(800) "
                "이내)."
            ),
        ),

        # [2026-08-30 신규] escort_follow 전용, 기본 꺼짐 -- 모듈 docstring
        # 참고. 일반적인 긴 경로에서는 켜지
        # 말 것(표준 pure pursuit curvature를 쓰는 게 맞음).
        DeclareLaunchArgument(
            'use_bearing_steering',
            default_value='false',
            description=(
                "true면 조향을 목표까지 거리(L) 기반 curvature(2y/L^2) 대신 "
                "각도(atan2(y,x))만으로 직접 계산 -- 로봇개가 화면 외곽에 "
                "잡혀 depth 오차로 L이 부풀려져도 조향이 확실히 나가게 함"
                "(purepursuit.py 모듈 docstring 참고)."
            ),
        ),
        DeclareLaunchArgument(
            'bearing_steering_gain',
            default_value='3.0',
            description=(
                "use_bearing_steering=true일 때 각도 오차(rad) 대비 목표 "
                "각속도(rad/s) 게인 -- [2026-08-30, 재조정] 제자리 회전이 "
                "너무 굼뜨다는 실기 피드백으로 1.5에서 2배(3.0)로 올림. "
                "실측 전 placeholder, 너무 크면 제자리 좌우 진동 가능."
            ),
        ),
        # [2026-08-30, 사용자 결정] 목표각이 크면 전진하며 도는 대신 제자리
        # 회전(v=0)으로 먼저 맞춘다 -- purepursuit.py 모듈 docstring 참고.
        DeclareLaunchArgument(
            'bearing_inplace_threshold_rad',
            default_value='0.05',
            description=(
                "use_bearing_steering=true일 때, 목표각이 이 값(rad)보다 "
                "크면 전진/후진 없이 제자리 회전만 한다 -- [2026-08-30, "
                "사용자 결정] '회전은 항상 제자리 회전으로' 요청으로 낮게 "
                "잡음(기본 약 3°, 거의 항상 제자리 회전부터 하고 정면 "
                "근처에서만 순수 전진/후진). 실측 전 placeholder."
            ),
        ),
        # [2026-08-31, 사용자 결정] 좌우 오프셋이 이 범위 안이면 조향 자체를
        # 아예 안 함 -- purepursuit.py 모듈 docstring의 bearing_deadband_lateral_m
        # 참고. bearing_inplace_threshold_rad(각도 기준)보다 먼저 검사됨.
        DeclareLaunchArgument(
            'bearing_deadband_lateral_m',
            default_value='0.8',
            description=(
                "use_bearing_steering=true일 때, 좌우 오프셋(m)의 절댓값이 "
                "이 값보다 작으면 각도가 얼마든 조향을 아예 안 함(w=0) -- "
                "'카메라 x좌표 -0.8~+0.8은 조향 안 하는 구간' 요청. 실측 전 "
                "placeholder."
            ),
        ),
        # [2026-08-30, 사용자 결정] 제자리 회전 구간만 한쪽 바퀴 고정 + 반대쪽만
        # 굴리는 피벗 턴으로 -- purepursuit.py 모듈 docstring/
        # _single_wheel_pivot_to_dps() 참고.
        DeclareLaunchArgument(
            'use_single_wheel_pivot',
            default_value='false',
            description=(
                "true면 제자리 회전(bearing_inplace_threshold_rad 초과 구간)을 "
                "양쪽 바퀴 반대회전 대신 한쪽 바퀴 고정 + 반대쪽만 굴리는 "
                "피벗 턴으로 한다(그 바퀴 속도는 표준 방식의 2배 dps가 됨 -- "
                "ICR이 중심에서 바퀴로 옮겨가 반경이 2배). 전진/후진하며 "
                "도는 구간에는 영향 없음."
            ),
        ),

        # [2026-09-04 신규, 사용자 요청] escort_follow 전용, 기본 꺼짐 --
        # purepursuit.py 모듈 docstring의 use_lost_left_turn_recovery 참고.
        # /path가 비면(LOST) 놓치기 직전 좌회전 중이었을 때만 잠깐 좌회전을
        # 유지하며 재포착 시도, 아니면(직진 중 놓침=가림막) 기존처럼 즉시 정지.
        # [2026-09-05 수정] 트리거 조건이 "놓치기 직전 경로가 좌측으로
        # 휘어 있었는지" OR로 확장됨 -- lost_left_turn_bearing_gain 참고.
        DeclareLaunchArgument(
            'use_lost_left_turn_recovery',
            default_value='false',
            description=(
                "true면 /path가 빌 때(LOST) 놓치기 직전 경로가 좌측으로 휘어 "
                "있었거나(alpha>0) 실제로 좌회전 중이었던 경우(둘 중 하나, "
                "lost_left_turn_w_threshold_rad_s 이상)에 한해 즉시 정지 대신 "
                "lost_left_turn_duration_sec 동안 제자리 좌회전하며 재포착을 "
                "시도한다. 직진하다 놓친 경우(가림막 구간 판단)는 이 옵션과 "
                "무관하게 항상 즉시 정지."
            ),
        ),
        DeclareLaunchArgument(
            'lost_left_turn_w_threshold_rad_s',
            default_value='0.1',
            description=(
                "use_lost_left_turn_recovery=true일 때, 놓치는 순간 마지막 "
                "발행 각속도(rad/s, 양수=좌회전)가 이 값 이상이어야 '좌회전 "
                "중 놓침'으로 본다(경로 좌측 곡률 조건과 OR). 실측 전 placeholder."
            ),
        ),
        DeclareLaunchArgument(
            'lost_left_turn_w_rad_s',
            default_value='1.0',
            description=(
                "좌회전 재포착 시도 중 목표 각속도[rad/s](v=0, 제자리)의 하한 -- "
                "lost_left_turn_bearing_gain 기반 계산값이 이보다 작으면 이 값을 "
                "대신 쓴다. 실측 전 placeholder."
            ),
        ),
        DeclareLaunchArgument(
            'lost_left_turn_bearing_gain',
            default_value='8.0',
            description=(
                "[2026-09-05 신규] 놓치는 순간 마지막 경로 목표점 각도(alpha, "
                "좌측=양수)에 곱해서 재포착 회전 목표각속도를 낸다 -- 로봇개가 "
                "화면 왼쪽 얼마나 바깥까지 나가 있었는지에 비례해서 더 세게 "
                "돈다. 계산값이 lost_left_turn_w_rad_s보다 작으면 그 하한이 대신 쓰임."
            ),
        ),
        DeclareLaunchArgument(
            'lost_left_turn_duration_sec',
            default_value='1.0',
            description=(
                "좌회전 재포착을 시도하는 최대 시간[초] -- 이 안에 재포착되면 "
                "즉시 취소되고 정상 추종으로 복귀, 넘기면 기존처럼 정지."
            ),
        ),

        # [2026-08-30 신규, 2026-08-31 가감속 분리] v 변화율 상한 --
        # purepursuit.py 모듈 docstring 참고. 이전엔 이 launch에 아예
        # 노출이 안 돼서(코드 기본값 1.0 고정) escort_follow용으로 올릴
        # 방법이 없었다 -- "정지 상태에서 로봇개를 바로 못 따라간다" 문제
        # 대응으로 노출/분리함. mission_escort_drive.launch.py가 accel만
        # 올려서 씀(감속은 전류 보호 목적이라 보수적 기본값 유지).
        DeclareLaunchArgument(
            'max_linear_accel_mps2',
            default_value='1.0',
            description=(
                "가속(|v|가 커지는) 방향 변화율 상한[m/s^2]. escort_follow는 "
                "정지/재포착 직후 빠르게 붙어야 해서 mission_escort_drive."
                "launch.py에서 더 높은 값으로 오버라이드한다 -- 실측 전 "
                "placeholder."
            ),
        ),
        DeclareLaunchArgument(
            'max_linear_decel_mps2',
            default_value='1.0',
            description=(
                "감속(|v|가 0 또는 반대 부호 쪽으로 다가가는) 방향 변화율 "
                "상한[m/s^2] -- 전류 스파이크 방지 목적이라 기본적으로 "
                "보수적으로 유지할 것. 실측 전 placeholder."
            ),
        ),
        # [2026-09-01 신규, 재도입] "회전이 갑자기 확 꺾인다"는 피드백 대응 --
        # 직진 가속(max_linear_accel_mps2)과 별개로 회전만 완만하게.
        # purepursuit.py 모듈 docstring/_ramp_w() 참고. 각속도 "자체"의
        # 최댓값이 아니라 각속도가 "변하는 속도"만 제한한다는 점에 주의.
        DeclareLaunchArgument(
            'max_angular_accel_rad_s2',
            default_value='6.0',
            description=(
                "w(각속도) 변화율 상한[rad/s^2] -- w_override(제자리 회전)든 "
                "v*curvature(전진하며 회전)든 매 틱 목표 각속도로 바로 "
                "튀지 않고 이 상한 이내로만 다가가게 한다. 각속도 자체의 "
                "상한은 아님(그건 bearing_steering_gain*각도와 "
                "max_wheel_speed_dps가 정함). 실측 전 placeholder, 너무 "
                "작으면 조향 반응이 느려짐."
            ),
        ),

        # base_link<-camera_link 정적 TF pitch/roll. imu_pitch_roll_probe.py가
        # 출력한 body-frame 부호를 그대로 static_transform_publisher에 쓴다.
        # 측정 조건: 로봇 몸체가 수평인 상태에서 IMU 가속도계로 쟀다(로봇
        # 자체가 기울어져 있으면 이 값도 같이 오염됨).
        DeclareLaunchArgument(
            'camera_pitch_rad',
            default_value='0.2737',
            description=(
                "base_link<-camera_link 정적 TF의 pitch[rad] -- [2026-09-04 "
                "재실측] 드라이브 카메라 내장 IMU로 수평 정지 상태에서 "
                "실측한 +0.2737rad(추종구간 마운트, std=0.0015rad). 이전 "
                "실측(2026-09-03, +0.2734rad)과 사실상 동일 -- 오차범위 내."
            ),
        ),
        DeclareLaunchArgument(
            'camera_roll_rad',
            default_value='0.0053',
            description=(
                "base_link<-camera_link 정적 TF의 roll[rad] -- [2026-09-04 "
                "재실측] 드라이브 카메라 내장 IMU로 수평 정지 상태에서 "
                "실측한 +0.0053rad(추종구간 마운트, std=0.0016rad). 이전 "
                "실측(2026-09-03, +0.0159rad)과 차이 있어 최신값으로 갱신."
            ),
        ),

        # ---- 정적 TF (reduced_odom_bringup.launch.py [단계 3]/[단계 3.5]와 동일 CAD 실측값) ----
        # 노드 이름을 reduced_odom_bringup.launch.py 쪽과 다르게 둬서, 혹시라도 실수로 두 launch가
        # 같이 뜨더라도 노드 이름 충돌(같은 이름 노드 중복 기동 에러)만은 안 나게 함.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='purepursuit_base_to_imu_tf',
            arguments=[
                '--x', '-0.180', '--y', '0', '--z', '0.375',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='purepursuit_base_to_camera_tf',
            arguments=[
                # [2026-09-03] base_link 기준 드라이브 카메라 실측 위치.
                '--x', '0.090461', '--y', '0', '--z', '0.805188',
                '--roll', camera_roll_rad, '--pitch', camera_pitch_rad, '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
            output='screen',
        ),

        # ---- CAN 드라이버 (manual.launch.py와 동일 파라미터) ----
        Node(
            package='can_driver',
            executable='can_driver_node',
            name='can_driver_node',
            parameters=[{
                'can_channel': can_channel,
                'left_can_id': 1,
                'right_can_id': 2,
                'cmd_timeout_sec': 0.3,
                'control_rate_hz': 50.0,
                'cmd_safety_timeout_sec': 0.5,
            }],
            output='screen',
        ),

        # ---- purepursuit_node ----
        # track_width_m/max_wheel_speed_dps는 코드 기본값(rmd_x8_driver_node의
        # *구버전* 기본값 0.50/800.0) 대신, reduced_odom_bringup.launch.py가 실제로 쓰는 캘리브레이션
        # 값(0.4904)과 자율주행 하드 클램프(기본 360.0 dps, max_wheel_speed_dps
        # launch 인자로 오버라이드 가능 -- 위 [중요]/escort_follow 사용 예 참고)를
        # 명시적으로 맞춘다 -- 같은 실물 로봇/모터니까 자율주행 쪽과 다른 값을
        # 쓸 이유가 없다.
        Node(
            package='dolbotz',
            executable='purepursuit',
            name='purepursuit_node',
            parameters=[
                params_file,
                {
                    'track_width_m': 0.4904,
                    'wheel_radius_m': 0.1125,
                    'max_wheel_speed_dps': ParameterValue(max_wheel_speed_dps, value_type=float),
                    # [주의] LaunchConfiguration은 항상 문자열로 해석되므로,
                    # bool/float로 선언된 파라미터에 그냥 넘기면 rclpy가 String
                    # 타입으로 잘못 설정해 declare_parameter의 bool/float 기본값과
                    # 타입이 안 맞아 예외가 난다 -- ParameterValue(value_type=...)로
                    # 명시 변환(mission_summer.launch.py의 enable_visualizer와
                    # 동일 패턴).
                    'use_outer_wheel_boost': ParameterValue(use_outer_wheel_boost, value_type=bool),
                    'target_linear_speed_m_s': ParameterValue(target_linear_speed_m_s, value_type=float),
                    'lookahead_distance_m': ParameterValue(lookahead_distance_m, value_type=float),
                    'goal_tolerance_m': ParameterValue(goal_tolerance_m, value_type=float),
                    'use_distance_scaled_speed': ParameterValue(use_distance_scaled_speed, value_type=bool),
                    'distance_speed_gain': ParameterValue(distance_speed_gain, value_type=float),
                    'distance_speed_min_m_s': ParameterValue(distance_speed_min_m_s, value_type=float),
                    'distance_speed_max_m_s': ParameterValue(distance_speed_max_m_s, value_type=float),
                    'distance_band_min_m': ParameterValue(distance_band_min_m, value_type=float),
                    'distance_band_max_m': ParameterValue(distance_band_max_m, value_type=float),
                    'distance_speed_reverse_max_m_s': ParameterValue(distance_speed_reverse_max_m_s, value_type=float),
                    'use_bearing_steering': ParameterValue(use_bearing_steering, value_type=bool),
                    'bearing_steering_gain': ParameterValue(bearing_steering_gain, value_type=float),
                    'bearing_inplace_threshold_rad': ParameterValue(bearing_inplace_threshold_rad, value_type=float),
                    'bearing_deadband_lateral_m': ParameterValue(bearing_deadband_lateral_m, value_type=float),
                    'use_single_wheel_pivot': ParameterValue(use_single_wheel_pivot, value_type=bool),
                    'use_lost_left_turn_recovery': ParameterValue(use_lost_left_turn_recovery, value_type=bool),
                    'lost_left_turn_w_threshold_rad_s': ParameterValue(lost_left_turn_w_threshold_rad_s, value_type=float),
                    'lost_left_turn_w_rad_s': ParameterValue(lost_left_turn_w_rad_s, value_type=float),
                    'lost_left_turn_bearing_gain': ParameterValue(lost_left_turn_bearing_gain, value_type=float),
                    'lost_left_turn_duration_sec': ParameterValue(lost_left_turn_duration_sec, value_type=float),
                    'max_linear_accel_mps2': ParameterValue(max_linear_accel_mps2, value_type=float),
                    'max_linear_decel_mps2': ParameterValue(max_linear_decel_mps2, value_type=float),
                    'max_angular_accel_rad_s2': ParameterValue(max_angular_accel_rad_s2, value_type=float),
                    # max_linear_accel_while_turning_mps2는 여기 없다 --
                    # config/purepursuit_params.yaml(params_file)에서만
                    # 설정한다(위 params_file DeclareLaunchArgument 주석
                    # 참고) -- 실험 버전마다 이 값만 바꾼 yaml을 통째로
                    # 스왑하는 용도.
                },
            ],
            output='screen',
        ),
    ])
