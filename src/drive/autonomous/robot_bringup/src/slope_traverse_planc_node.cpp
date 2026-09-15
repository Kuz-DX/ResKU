#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>

using namespace std::chrono_literals;

// slope_traverse_planc_node: slope_traverse_node.cpp의 "PlanC" 대안 노드.
//
// [2026-09-04 신규, 사용자 요청] 커밋 c5aef4d1b0c374abda2a688b04c020ff34b62e8a
// ((탈출구간 명시)feat: SLOPE_EXIT 및 RECOVERY 상태 추가, 2026-08-27) 시점의
// slope_traverse_node.cpp를 그대로 되살려 별도 노드로 분리했다 -- 클래스/노드
// 이름만 바꿨을 뿐(SlopeTraverseNode -> SlopeTraversePlanCNode, node name
// "slope_traverse_node" -> "slope_traverse_planc_node") 로직은 그 시점 그대로
// 손대지 않았다(autonomous.planz.py를 옛 purepursuit.py 코드로 되살린 것과
// 동일한 패턴, autonomous.planz.launch.py 상단 docstring 참고).
//
// 이 시점 코드와 현재 slope_traverse_node.cpp(PlanA)의 핵심 차이: SLOPE_EXIT
// 조향이 TF 기반 pure pursuit(2026-08-28 도입, path_base_xy_/tf_buffer_)이
// 아니라, kp_exit_(고정 게인 P제어) + computeExitWzMax()(slope_exit_side_
// motor_dps_/slope_exit_other_motor_dps_ 400/150dps 조합에서 역산한 wz 상한)
// 조합이다 -- slope_traverse_blend_node.cpp(PlanB, TF 없이 pure pursuit
// 곡률식 재사용)와도 다른, TF 자체가 코드에 전혀 없는(tf2_ros/
// tf2_geometry_msgs 미의존) 더 이전 버전. path_control_planc.launch.py가
// slope_traverse_node 대신 이 노드를 띄운다.
//
// slope_traverse_node: 인지팀이 발행하는 signed left/right 경사 신호를 받아
// "짧게 직진 -> pitch가 완만해지는 추세에 맞춰 /path 방향으로 서서히
// 차동조향" 흐름을 직접 /cmd_vel_safety로 발행한다.
//
// [2026-08-27 전면 재설계, 3번째] 이전 버전(ALIGNING에서 위치도 모른 채
// IMU만 보고 즉시 목표 각도로 회전 -> CLIMBING 연속 블렌딩 -> EXITING은
// /path가 20도 꺾일 때만 트리거)에서, 사용자 지적(신호를 받는 순간
// 로봇이 트랙 어디 있는지 모르는 채로 도는 게 위험함)에 따라 다음을
// 제거했다:
//   - ALIGNING: 진입 즉시 특정 헤딩으로 도는 동작 자체를 없앰. 신호가
//     오면 그 순간 헤딩 그대로 유지한 채 SLOPE_DRIVE로 바로 진입한다.
//   - EXITING의 /path 곡률 트리거: 더 이상 "몇 도 꺾이면 탈출 모드로
//     전환" 같은 이산적 전환이 없다. 대신 SLOPE_DRIVE 하나가 처음부터
//     끝까지 연속적으로 /path 방향을 따라간다.
// 남은 것은 CLIMBING의 pitch 기반 연속 블렌딩 아이디어(grace period +
// smoothstep)뿐이며, 이걸 SLOPE_DRIVE 전체에 그대로 적용한다.
//
// 상태머신 (단순화):
//   IDLE -> SLOPE_DRIVE -> SLOPE_EXIT -> RECOVERY -> IDLE
//
//   - IDLE: slope_side_topic_(부호 규약은 확정, 토픽명/메시지 타입은 아직
//     PLACEHOLDER -- 아래 참고)에서 side != 0이 side_enter_dwell_sec_ 이상
//     유지되면 진입. entered_side_를 그 시점 값으로 고정하고,
//     entry_yaw_deg_ = 그 순간의 현재 yaw(오프셋 없음 -- 헤딩을 특정
//     방향으로 트는 동작이 없으므로 그냥 "지금 향하고 있는 방향").
//   - SLOPE_DRIVE: vx=climb_speed_ 기반, target heading은 entry_yaw_deg_에서
//     시작해서 매 tick 그 순간 /path 접선 방향 쪽으로 블렌딩 계수만큼
//     당겨진다(computeBlendedTargetDeg() 참고). 진입 직후
//     climb_straight_grace_sec_(기본 3초) 동안은 pitch/추세와 무관하게
//     블렌딩 계수 강제 0(무조건 직진). grace 이후엔 pitch를 EMA로
//     필터링해 그 값(레벨)과 변화율(하강 방향만)을 함께 보고, 완만해지는
//     추세면 서서히(smoothstep) 블렌딩 계수를 올린다 -- "정상부 도착 후
//     회전"이 아니라 "정상부 접근하며 미리 조금씩 회전량 확보"가 목표.
//     조향(wz)은 kp_steer_/wz_steer_max_로 계산하고, computeBoostedVx()로
//     "안쪽 바퀴는 climb_speed_ 그대로, 바깥쪽만 부스트"하는 vx를 낸다
//     (매뉴얼 주행 실측 참고). myAHRS+ pitch(부호 있는 raw 값,
//     last_pitch_signed_deg_ -- CLIMBING 블렌딩용 절댓값 last_pitch_deg_와는
//     별개)가 slope_exit_entry_pitch_deg_(기본 -9deg) 이하로 내려가면
//     slope_exit_trigger_dwell_sec_ 디바운스 후 SLOPE_EXIT로 전환.
//   - SLOPE_EXIT("경사로 탈출 구간", [2026-08-27] 신규, 28일 재설계):
//     SLOPE_DRIVE와 동일하게 updateBlendedTarget()로 /path 블렌딩을 계속
//     쓴다(사용자 요청 -- 처음엔 /path를 아예 안 쓰게 했다가, 블렌딩 자체는
//     유지하고 조향 강도만 다르게 하는 쪽으로 변경). 다만 조향 wz 상한을
//     kp_steer_/wz_steer_max_ 대신 kp_exit_(기본 0.8, 더 큰 게인)과
//     computeExitWzMax()(매뉴얼 실측 기준 slope_exit_side_motor_dps_(400)/
//     slope_exit_other_motor_dps_(150) 조합에서 역산한 상한, 예를 들어
//     이 조합이면 wz~1.0rad/s)로 훨씬 세게 잡는다 -- 같은 /path 방향을
//     보되 "얼마나 세게 돌 수 있는지"만 이 구간에서 대폭 완화. vx는
//     computeBoostedVx(computeExitBaseSpeed(), wz)로, 안쪽 바퀴는
//     slope_exit_other_motor_dps_(150) 기준을 유지하고 바깥쪽만 부스트.
//     탈출 조건은 세 센서를 동시에(AND) 만족해야 함
//     (slope_exit_end_dwell_sec_ 디바운스): ① RealSense IMU(camera_imu_topic_)
//     roll이 slope_exit_end_roll_deg_(기본 3deg) 이내, ② myAHRS+ pitch가
//     slope_exit_end_pitch_deg_(기본 -6deg) 이상, ③ /path가 직진
//     (computeRelativePathOffsetDeg() 절댓값이 path_straight_threshold_deg_
//     이내, 단 이건 조향용이 아니라 탈출 판정 확인용으로만 계속 읽음).
//     셋 다 갖춰지면(혹은 side 신호 자체가 사라지면) RECOVERY로 전환.
//     slope_exit_timeout_sec_(기본 15초) 넘도록 조건이 안 갖춰지면
//     안전망으로 정지-래치.
//   - RECOVERY([2026-08-27] 신규): SLOPE_EXIT의 공격적 차동(400/150)에서
//     바로 MPPI로 넘기지 않고, 좌우 동일 recovery_motor_dps_(기본 200)로
//     recovery_dwell_sec_(기본 1초) 동안 직진 정착한 뒤 IDLE 복귀
//     (entered_side_ 리셋, MPPI가 이어받음).
//
// 안전 fallback: SLOPE_DRIVE 중 yaw_error가 climb_large_yaw_error_deg_를
// 넘거나(경사 위 재정렬은 위험), climb_timeout_sec_(신호가 계속 살아있는데
// 너무 오래 SLOPE_DRIVE에 머무는 경우의 최후 안전망) 넘으면 그대로 정지 --
// timeout_stop_latched_로 래치하고 자동으로 안 풀리게 해서 사람이 원인
// 확인 후 재시작하는 걸 전제로 한다.
//
// slope_side_topic_ 인터페이스: 부호 규약은 인지팀 확정 완료(2026-08-27) --
// 오른쪽이 높은 경사=+, 왼쪽이 높은 경사=-. 입력 신호의
// dh=hR-hL("+면 오른쪽이 더 높다") 규약과도 일치해서 +1=right_slope,
// -1=left_slope로 그대로 구현. 단, 토픽명/정확한 메시지 타입(Int8 카테고리
// 값인지, side_slope_angle_deg류 연속 Float32 각도인지)은 아직 PLACEHOLDER
// -- 확정되면 slopeSideCallback()과 멤버 타입만 바꾸면 되도록 다른 로직과
// 분리해뒀다.
//
// computeBoostedVx(): rmd_x8_driver_node의 대칭 스큐-스티어 역기구학
// (v_left=v-wz*halftrack, v_right=v+wz*halftrack)에 vx를 그대로 넘기면
// 회전할수록 안쪽 바퀴가 climb_speed_ 밑으로 깎인다. 매뉴얼 주행 실측
// (한쪽 300~350dps/반대쪽 200dps 조합이 잘 됐다는 사용자 피드백)을
// 참고해 "안쪽은 climb_speed_ 그대로, 바깥쪽만 부스트"하도록
// v = climb_speed_+|wz|*halftrack으로 올려서 보낸다.
//
// /cmd_vel_safety 재사용 근거: rmd_x8_driver_node.py의 _control_loop()를
// 직접 읽고 확인함 -- cmd_vel_safety_timeout(기본 0.5s) 이내 최근 수신되면
// /cmd_vel(MPPI 출력)을 완전히 무시하고 그 값을 그대로 쓰는 토픽 기반
// 범용 최우선순위 메커니즘. IDLE일 때는 publish를 멈춰서 timeout이 자연히
// 만료되며 MPPI 제어로 복귀한다.
class SlopeTraversePlanCNode : public rclcpp::Node
{
public:
  SlopeTraversePlanCNode()
  : Node("slope_traverse_planc_node")
  {
    track_width_m_ = declare_parameter<double>("track_width_m", 0.4904);

    // --- 진입: 인지팀 signed left/right 신호 (PLACEHOLDER, 파일 상단 참고) --
    slope_side_topic_ = declare_parameter<std::string>(
      "slope_side_topic", "/terrain/slope_side_signal");
    slope_side_signal_timeout_sec_ =
      declare_parameter<double>("slope_side_signal_timeout_sec", 1.0);
    side_enter_dwell_sec_ = declare_parameter<double>("side_enter_dwell_sec", 0.25);
    side_exit_dwell_sec_ = declare_parameter<double>("side_exit_dwell_sec", 0.25);

    // --- 주행/조향 -----------------------------------------------------
    climb_speed_ = declare_parameter<double>("climb_speed", 0.4);
    // [2026-08-27] 예전 EXITING 전용이었던 더 적극적인 게인(kp_exit=0.4,
    // wz_exit_max=0.5)을 그대로 가져옴 -- 이제 EXITING이 따로 없고
    // SLOPE_DRIVE 하나가 처음부터 끝까지 이 게인으로 조향하므로.
    kp_steer_ = declare_parameter<double>("kp_steer", 0.4);
    wz_steer_max_ = declare_parameter<double>("wz_steer_max", 0.5);
    climb_timeout_sec_ = declare_parameter<double>("climb_timeout_sec", 600.0);
    climb_large_yaw_error_deg_ = declare_parameter<double>("climb_large_yaw_error_deg", 90.0);

    // --- target heading 연속 블렌딩 ---------------------------------------
    climb_pitch_low_threshold_deg_ =
      declare_parameter<double>("climb_pitch_low_threshold_deg", 8.0);
    climb_pitch_high_threshold_deg_ =
      declare_parameter<double>("climb_pitch_high_threshold_deg", 22.0);
    climb_pitch_rate_threshold_deg_s_ =
      declare_parameter<double>("climb_pitch_rate_threshold_deg_s", 3.0);
    climb_pitch_rate_boost_gain_ =
      declare_parameter<double>("climb_pitch_rate_boost_gain", 0.5);
    climb_min_blend_factor_ = declare_parameter<double>("climb_min_blend_factor", 0.0);
    climb_pitch_filter_alpha_ = declare_parameter<double>("climb_pitch_filter_alpha", 0.3);
    // SLOPE_DRIVE 진입 직후 이 시간(초) 동안은 pitch 상태와 무관하게
    // 무조건 직진 유지(블렌딩 강제 억제). [2026-08-27] 3.5 -> 3.0 -> 0.0
    // (사용자 요청으로 강제 직진 구간 자체를 없앰 -- 처음부터 바로 /path
    // 따라 블렌딩. 코드는 안 지우고 0으로 둬서, elapsed(항상 >=0) <
    // 0.0이 첫 tick부터 거짓이라 자연히 no-op -- 필요하면 언제든
    // ros2 param set으로 다시 켤 수 있음).
    climb_straight_grace_sec_ = declare_parameter<double>("climb_straight_grace_sec", 0.0);

    // --- [2026-08-27 재설계] 경사로 탈출 구간(SLOPE_EXIT) -------------------
    // 경사 마지막 구간에서 평지로 넘어갈 때 /path를 못 믿어서(신뢰도 낮음)
    // 계속 직진해버리고 못 도는 문제 대응.
    //
    // 진입: myAHRS+ pitch(부호 있는 raw 값 -- 기존 CLIMBING 블렌딩에 쓰는
    // 절댓값과 별개, imuCallback()의 last_pitch_signed_deg_ 참고)가
    // slope_exit_entry_pitch_deg_(기본 -9deg) 이하로 내려가면(slope_exit_trigger_dwell_sec_
    // 디바운스) SLOPE_EXIT로 전환. /path 블렌딩(updateBlendedTarget())은
    // 계속 쓰되, kp_exit_/computeExitWzMax()로 조향을 훨씬 세게(매뉴얼
    // 실측 기준 slope_exit_side_motor_dps_(400)/slope_exit_other_motor_dps_
    // (150) 조합에서 역산한 wz 상한까지) 밀어붙인다.
    //
    // 탈출: 세 조건을 동시에(AND) 만족해야 함(slope_exit_end_dwell_sec_
    // 디바운스) -- ① RealSense IMU(camera_imu_topic_)의 roll이
    // slope_exit_end_roll_deg_(기본 3deg) 이내, ② myAHRS+ pitch가
    // slope_exit_end_pitch_deg_(기본 -6deg) 이상(즉 -9보다 완만해짐),
    // ③ /path가 직진(computeRelativePathOffsetDeg()의 절댓값이
    // path_straight_threshold_deg_ 이내). 셋 다 확인돼야 RECOVERY로 전환 --
    // 서로 다른 센서 세 개를 같이 봐서 하나가 노이즈로 튀어도 오탈출을
    // 막기 위함.
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.1125);
    slope_exit_entry_pitch_deg_ =
      declare_parameter<double>("slope_exit_entry_pitch_deg", -5.0);
    slope_exit_end_pitch_deg_ =
      declare_parameter<double>("slope_exit_end_pitch_deg", -6.0);
    slope_exit_end_roll_deg_ = declare_parameter<double>("slope_exit_end_roll_deg", 3.0);
    path_straight_threshold_deg_ =
      declare_parameter<double>("path_straight_threshold_deg", 5.0);
    slope_exit_trigger_dwell_sec_ =
      declare_parameter<double>("slope_exit_trigger_dwell_sec", 0.2);
    slope_exit_end_dwell_sec_ = declare_parameter<double>("slope_exit_end_dwell_sec", 0.3);
    slope_exit_side_motor_dps_ = declare_parameter<double>("slope_exit_side_motor_dps", 400.0);
    slope_exit_other_motor_dps_ = declare_parameter<double>("slope_exit_other_motor_dps", 150.0);
    slope_exit_timeout_sec_ = declare_parameter<double>("slope_exit_timeout_sec", 15.0);
    camera_imu_topic_ = declare_parameter<std::string>("camera_imu_topic", "/drive/camera/imu");
    // [2026-08-27 신규] SLOPE_EXIT 조향 게인 -- kp_steer_(SLOPE_DRIVE용,
    // 0.4)보다 크게 잡아서 같은 yaw_error에도 더 적극적으로 wz 상한
    // (computeExitWzMax(), 400/150dps 기준)까지 밀어붙이도록 함.
    kp_exit_ = declare_parameter<double>("kp_exit", 0.8);

    // --- [2026-08-27 신규] RECOVERY: 탈출 후 정착 구간 ----------------------
    // SLOPE_EXIT이 끝나면 곧장 IDLE(=MPPI로 넘김)로 가지 않고, 좌우 동일
    // recovery_motor_dps_(기본 200)로 recovery_dwell_sec_ 동안 잠깐 직진
    // 안정화한 뒤 IDLE로 넘어간다 -- 400/150 같은 공격적 차동에서 바로
    // MPPI로 넘기지 않고 완충 구간을 둠.
    recovery_motor_dps_ = declare_parameter<double>("recovery_motor_dps", 200.0);
    recovery_dwell_sec_ = declare_parameter<double>("recovery_dwell_sec", 1.0);

    // --- 블렌딩 목표: /path 접선 방향 -------------------------------------
    path_topic_ = declare_parameter<std::string>("path_topic", "/path");
    path_timeout_sec_ = declare_parameter<double>("path_timeout_sec", 1.0);
    path_tangent_lookahead_m_ = declare_parameter<double>("path_tangent_lookahead_m", 0.5);

    recovery_state_topic_ = declare_parameter<std::string>(
      "recovery_state_topic", "/drive/slope_traverse_state");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/slope_traverse/debug");
    cmd_vel_safety_topic_ = declare_parameter<std::string>(
      "cmd_vel_safety_topic", "/cmd_vel_safety");
    // [2026-08-27] 100.0 -> 20.0 -- 이 노드가 이제 순간 개입용 안전장치가
    // 아니라 SLOPE_DRIVE 동안 계속 운전하는 제어 루프라, MPPI(controller_frequency)
    // 와 같은 20Hz로 맞춤(사용자 확정). cmd_vel_safety_timeout(0.5s) 대비
    // 여전히 10배 여유가 있어 /cmd_vel_safety 우선권 유지엔 문제없음.
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);

    recovery_state_pub_ = create_publisher<std_msgs::msg::String>(
      recovery_state_topic_, rclcpp::QoS(1).transient_local());
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(debug_topic_, 10);
    cmd_vel_safety_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_safety_topic_, 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&SlopeTraversePlanCNode::imuCallback, this, std::placeholders::_1));
    // [2026-08-27 신규] SLOPE_EXIT 종료 조건 중 하나(roll) 전용. myAHRS+(/imu)
    // 와는 별개 센서 -- realsense2_camera_node가 enable_gyro/enable_accel로
    // 내는 D455 자체 IMU(perception 파이프라인이 이미 띄우는 카메라, autonomous.launch.py
    // 와 함께 떠 있어야 값이 들어옴).
    camera_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      camera_imu_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&SlopeTraversePlanCNode::cameraImuCallback, this, std::placeholders::_1));
    slope_side_sub_ = create_subscription<std_msgs::msg::Int8>(
      slope_side_topic_, 10,
      std::bind(&SlopeTraversePlanCNode::slopeSideCallback, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1),
      std::bind(&SlopeTraversePlanCNode::pathCallback, this, std::placeholders::_1));

    state_entered_time_ = get_clock()->now();

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SlopeTraversePlanCNode::controlTimerCallback, this));

    // [2026-08-27] 실차 튜닝 중 재시작 없이 ros2 param set으로 값을 바로
    // 반영하기 위한 동적 콜백. 토픽명(string)/control_rate_hz(타이머 주기가
    // 생성 시점에 고정)는 제외.
    tunable_params_ = {
      {"track_width_m", &track_width_m_},
      {"slope_side_signal_timeout_sec", &slope_side_signal_timeout_sec_},
      {"side_enter_dwell_sec", &side_enter_dwell_sec_},
      {"side_exit_dwell_sec", &side_exit_dwell_sec_},
      {"climb_speed", &climb_speed_},
      {"kp_steer", &kp_steer_},
      {"wz_steer_max", &wz_steer_max_},
      {"climb_timeout_sec", &climb_timeout_sec_},
      {"climb_large_yaw_error_deg", &climb_large_yaw_error_deg_},
      {"climb_pitch_low_threshold_deg", &climb_pitch_low_threshold_deg_},
      {"climb_pitch_high_threshold_deg", &climb_pitch_high_threshold_deg_},
      {"climb_pitch_rate_threshold_deg_s", &climb_pitch_rate_threshold_deg_s_},
      {"climb_pitch_rate_boost_gain", &climb_pitch_rate_boost_gain_},
      {"climb_min_blend_factor", &climb_min_blend_factor_},
      {"climb_pitch_filter_alpha", &climb_pitch_filter_alpha_},
      {"climb_straight_grace_sec", &climb_straight_grace_sec_},
      {"wheel_radius_m", &wheel_radius_m_},
      {"slope_exit_entry_pitch_deg", &slope_exit_entry_pitch_deg_},
      {"slope_exit_end_pitch_deg", &slope_exit_end_pitch_deg_},
      {"slope_exit_end_roll_deg", &slope_exit_end_roll_deg_},
      {"path_straight_threshold_deg", &path_straight_threshold_deg_},
      {"slope_exit_trigger_dwell_sec", &slope_exit_trigger_dwell_sec_},
      {"slope_exit_end_dwell_sec", &slope_exit_end_dwell_sec_},
      {"slope_exit_side_motor_dps", &slope_exit_side_motor_dps_},
      {"slope_exit_other_motor_dps", &slope_exit_other_motor_dps_},
      {"slope_exit_timeout_sec", &slope_exit_timeout_sec_},
      {"kp_exit", &kp_exit_},
      {"recovery_motor_dps", &recovery_motor_dps_},
      {"recovery_dwell_sec", &recovery_dwell_sec_},
      {"path_timeout_sec", &path_timeout_sec_},
      {"path_tangent_lookahead_m", &path_tangent_lookahead_m_},
    };
    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&SlopeTraversePlanCNode::onParamUpdate, this, std::placeholders::_1));

    publishState();

    RCLCPP_INFO(
      get_logger(),
      "slope_traverse_planc_node started: slope_side_topic=%s climb_speed=%.2f "
      "kp_steer=%.2f wz_steer_max=%.2f climb_straight_grace_sec=%.1f",
      slope_side_topic_.c_str(), climb_speed_, kp_steer_, wz_steer_max_,
      climb_straight_grace_sec_);
  }

private:
  enum class State { IDLE, SLOPE_DRIVE, SLOPE_EXIT, RECOVERY };

  static double wrapDeg180(double deg)
  {
    while (deg > 180.0) {deg -= 360.0;}
    while (deg < -180.0) {deg += 360.0;}
    return deg;
  }

  // 0 (x<=lo), 1 (x>=hi), 그 사이는 3t^2-2t^3 (t=(x-lo)/(hi-lo)) -- hard
  // switching 없이 threshold 근처에서 기울기 0으로 부드럽게 붙는 보간.
  static double smoothstep(double x, double lo, double hi)
  {
    if (hi <= lo) {return x >= hi ? 1.0 : 0.0;}
    const double t = std::clamp((x - lo) / (hi - lo), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
  }

  double yawErrorDeg(double target_yaw_deg) const
  {
    return wrapDeg180(target_yaw_deg - last_yaw_deg_);
  }

  // SLOPE_DRIVE에서 wz로 조향할 때 publish할 vx -- "안쪽 바퀴는
  // climb_speed_ 그대로, 바깥쪽만 부스트"(파일 상단 docstring 참고).
  double computeBoostedVx(double base_speed, double wz) const
  {
    return base_speed + std::abs(wz) * (track_width_m_ / 2.0);
  }

  // [2026-08-27 재설계] SLOPE_EXIT은 이제 /path 블렌딩(updateBlendedTarget())을
  // 그대로 쓰고, wz 상한만 여기서 400/150dps 기준으로 derive한다 --
  // computeBoostedVx(exitBaseSpeedMps(), wz)를 wz=이 값으로 saturate시키면
  // 정확히 안쪽=slope_exit_other_motor_dps_, 바깥쪽=slope_exit_side_motor_dps_
  // 가 나오도록 역산한 값(대입해서 검산 가능: v_outer = base + wz*track_width_m_,
  // base=other_mps로 두면 wz_max*track_width_m_ = side_mps-other_mps).
  // 매 tick 현재 파라미터 값으로 다시 계산하므로 ros2 param set으로
  // slope_exit_side/other_motor_dps_를 바꾸면 바로 반영된다(별도 캐시 없음).
  double computeExitWzMax() const
  {
    const double side_mps = slope_exit_side_motor_dps_ * (M_PI / 180.0) * wheel_radius_m_;
    const double other_mps = slope_exit_other_motor_dps_ * (M_PI / 180.0) * wheel_radius_m_;
    return std::abs(side_mps - other_mps) / track_width_m_;
  }

  // SLOPE_EXIT에서 computeBoostedVx()의 base_speed로 쓸 값(안쪽 바퀴 목표,
  // slope_exit_other_motor_dps_ 기준).
  double computeExitBaseSpeed() const
  {
    return slope_exit_other_motor_dps_ * (M_PI / 180.0) * wheel_radius_m_;
  }

  // tunable_params_에 등록된 double 파라미터가 ros2 param set으로 바뀌면
  // 해당 멤버 변수에 즉시 반영 -- 재시작 없이 실차 튜닝하기 위함.
  rcl_interfaces::msg::SetParametersResult onParamUpdate(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      auto it = tunable_params_.find(p.get_name());
      if (it != tunable_params_.end() &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        *(it->second) = p.as_double();
        RCLCPP_INFO(
          get_logger(), "파라미터 실시간 반영: %s = %f", p.get_name().c_str(), p.as_double());
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    last_yaw_deg_ = yaw * 180.0 / M_PI;
    last_pitch_deg_ = std::abs(pitch) * 180.0 / M_PI;
    // [2026-08-27 신규] SLOPE_EXIT 진입/탈출 조건은 절댓값이 아니라 부호
    // 있는 값을 본다(-9 진입, -6 탈출) -- CLIMBING 블렌딩용 last_pitch_deg_
    // (절댓값)와는 별개로 유지.
    last_pitch_signed_deg_ = pitch * 180.0 / M_PI;
    has_imu_data_ = true;
  }

  // [2026-08-27 신규] RealSense(D455) 자체 IMU -- SLOPE_EXIT 탈출 조건의
  // roll 값 전용.
  void cameraImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    last_camera_roll_deg_ = roll * 180.0 / M_PI;
    has_camera_imu_data_ = true;
  }

  // target heading 블렌딩 계수(0~1) 계산. pitch를 EMA로 필터링해(raw IMU
  // 노이즈가 미분에 그대로 실리는 걸 막음) 그 필터링된 값과, 변화율(하강
  // 방향만) 둘 다 반영. dt는 이 노드의 control_timer_(100Hz, 단일 콜백
  // 스레드)가 매 tick 실제로 호출된 간격을 그대로 쓰므로 안전하다.
  double updatePitchFilterAndComputeBlend()
  {
    const rclcpp::Time now = get_clock()->now();
    if (!has_pitch_filter_state_) {
      pitch_filtered_deg_ = last_pitch_deg_;
      pitch_rate_deg_s_ = 0.0;
      has_pitch_filter_state_ = true;
      last_pitch_filter_time_ = now;
      return climb_min_blend_factor_;
    }
    const double dt = (now - last_pitch_filter_time_).seconds();
    const double prev_filtered = pitch_filtered_deg_;
    pitch_filtered_deg_ =
      climb_pitch_filter_alpha_ * last_pitch_deg_ + (1.0 - climb_pitch_filter_alpha_) * pitch_filtered_deg_;
    if (dt > 1e-3) {
      pitch_rate_deg_s_ = (pitch_filtered_deg_ - prev_filtered) / dt;
      last_pitch_filter_time_ = now;
    }

    const double level_factor = 1.0 - smoothstep(
      pitch_filtered_deg_, climb_pitch_low_threshold_deg_, climb_pitch_high_threshold_deg_);
    const double descending = std::max(0.0, -pitch_rate_deg_s_);
    const double rate_factor = smoothstep(descending, 0.0, climb_pitch_rate_threshold_deg_s_);
    const double base = climb_min_blend_factor_ + (1.0 - climb_min_blend_factor_) * level_factor;
    const double boosted = base + climb_pitch_rate_boost_gain_ * rate_factor * (1.0 - base);
    const double blend = std::clamp(boosted, climb_min_blend_factor_, 1.0);

    // SLOPE_DRIVE 진입 직후 climb_straight_grace_sec_ 동안은 pitch 상태와
    // 무관하게 무조건 직진 유지(블렌딩 계수 강제 최소치).
    const double elapsed = (now - state_entered_time_).seconds();
    if (elapsed < climb_straight_grace_sec_) {
      return climb_min_blend_factor_;
    }
    return blend;
  }

  // [2026-08-27 신규] SLOPE_DRIVE/SLOPE_EXIT 공용 -- target heading을
  // entry_yaw_deg_에서 그 순간 /path 접선 방향 쪽으로 블렌딩 계수만큼
  // 당긴 blended_target_deg_를 갱신한다. last_rel_offset_deg_에 이번 tick의
  // 원본 signed 오프셋도 같이 저장해서(nullopt=path 신뢰 불가), SLOPE_EXIT의
  // path_straight 탈출 판정이 이 함수를 다시 호출할 필요 없이 재사용할 수
  // 있게 한다.
  void updateBlendedTarget()
  {
    last_rel_offset_deg_ = computeRelativePathOffsetDeg();
    const double blend = updatePitchFilterAndComputeBlend();
    if (last_rel_offset_deg_.has_value()) {
      const double path_tangent_yaw_deg = wrapDeg180(last_yaw_deg_ + *last_rel_offset_deg_);
      const double diff = wrapDeg180(path_tangent_yaw_deg - entry_yaw_deg_);
      blended_target_deg_ = wrapDeg180(entry_yaw_deg_ + blend * diff);
    } else {
      // path 신뢰 불가 -- 안전하게 진입 헤딩 유지(블렌딩 안 함).
      blended_target_deg_ = entry_yaw_deg_;
    }
  }

  // side: +1=right_slope, -1=left_slope, 0=없음
  void slopeSideCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    last_side_signal_ = msg->data;
    has_side_signal_ = true;
    last_side_signal_time_ = get_clock()->now();
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_path_ = msg;
    last_path_time_ = get_clock()->now();
  }

  // /path(camera_link 상대, x=forward/y=left) 앞 path_tangent_lookahead_m_
  // 구간의 접선 방향을 로봇 헤딩 기준 signed 상대 오프셋(deg)으로 반환.
  // path가 없거나/오래됐거나/너무 짧으면 nullopt.
  std::optional<double> computeRelativePathOffsetDeg()
  {
    if (!latest_path_ || latest_path_->poses.size() < 2) {
      return std::nullopt;
    }
    if ((get_clock()->now() - last_path_time_).seconds() > path_timeout_sec_) {
      return std::nullopt;
    }

    const auto & poses = latest_path_->poses;
    double acc = 0.0;
    size_t end_idx = poses.size() - 1;
    for (size_t i = 1; i < poses.size(); ++i) {
      const double dx = poses[i].pose.position.x - poses[i - 1].pose.position.x;
      const double dy = poses[i].pose.position.y - poses[i - 1].pose.position.y;
      acc += std::hypot(dx, dy);
      if (acc >= path_tangent_lookahead_m_) {
        end_idx = i;
        break;
      }
    }

    const double dx = poses[end_idx].pose.position.x - poses[0].pose.position.x;
    const double dy = poses[end_idx].pose.position.y - poses[0].pose.position.y;
    if (std::hypot(dx, dy) < 0.05) {
      return std::nullopt;
    }

    return std::atan2(dy, dx) * 180.0 / M_PI;
  }

  void updateState()
  {
    if (timeout_stop_latched_) {
      return;
    }
    if (!has_imu_data_) {
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    const State prev = state_;

    const bool side_fresh = has_side_signal_ &&
      (now - last_side_signal_time_).seconds() <= slope_side_signal_timeout_sec_;
    const int8_t side = side_fresh ? last_side_signal_ : 0;

    switch (state_) {
      case State::IDLE: {
        if (side != 0) {
          if (!enter_cond_since_ || pending_side_ != side) {
            enter_cond_since_ = now;
            pending_side_ = side;
          }
          if ((now - *enter_cond_since_).seconds() >= side_enter_dwell_sec_) {
            entered_side_ = side;
            // [2026-08-27] 더 이상 특정 헤딩으로 트는 동작이 없음 --
            // entry_yaw_deg_는 진입 시점 yaw 그대로(오프셋 0).
            entry_yaw_deg_ = last_yaw_deg_;
            state_ = State::SLOPE_DRIVE;
            has_pitch_filter_state_ = false;  // 진입마다 pitch 필터 새로 초기화
            enter_cond_since_.reset();
            exit_cond_since_.reset();
          }
        } else {
          enter_cond_since_.reset();
        }
        break;
      }
      case State::SLOPE_DRIVE: {
        // target heading을 entry_yaw_deg_에 고정하지 않고, pitch가
        // 완만해지는 추세면 그 순간 /path 접선 방향 쪽으로 서서히
        // 블렌딩한다(updateBlendedTarget() 참고). SLOPE_EXIT에서도 그대로
        // 재사용(아래) -- /path 블렌딩 자체는 안 끊고, 조향 강도(wz 상한)만
        // 그쪽에서 다르게 잡는다([2026-08-27] 사용자 요청).
        updateBlendedTarget();

        // [2026-08-27 신규] myAHRS+ pitch(부호 있는 값)가 slope_exit_entry_pitch_deg_
        // (기본 -9deg) 이하로 내려가면 SLOPE_EXIT(경사로 탈출 구간)로 전환 --
        // 조향은 계속 /path 블렌딩을 쓰되, wz 상한을 400/150dps 기준으로
        // 훨씬 크게 잡아서 더 세게 돌 수 있게 한다(controlTimerCallback() 참고).
        if (last_pitch_signed_deg_ <= slope_exit_entry_pitch_deg_) {
          if (!slope_exit_trigger_cond_since_) {slope_exit_trigger_cond_since_ = now;}
          if ((now - *slope_exit_trigger_cond_since_).seconds() >= slope_exit_trigger_dwell_sec_) {
            state_ = State::SLOPE_EXIT;
            slope_exit_trigger_cond_since_.reset();
            break;
          }
        } else {
          slope_exit_trigger_cond_since_.reset();
        }

        // 신호가 사라지면(side==0) side_exit_dwell_sec_ 디바운스 후 IDLE.
        if (side == 0) {
          if (!exit_cond_since_) {exit_cond_since_ = now;}
          if ((now - *exit_cond_since_).seconds() >= side_exit_dwell_sec_) {
            state_ = State::IDLE;
            entered_side_ = 0;
            exit_cond_since_.reset();
          }
        } else {
          exit_cond_since_.reset();
          entered_side_ = side;  // 반대쪽으로 바뀌면 디바운스 없이 즉시 반영
        }

        const double yaw_err = yawErrorDeg(blended_target_deg_);
        if (std::abs(yaw_err) >= climb_large_yaw_error_deg_) {
          RCLCPP_ERROR(
            get_logger(),
            "SLOPE_DRIVE 중 yaw_error=%.1fdeg >= %.1fdeg -- 정지 후 래치(재시작 전까지 유지)",
            yaw_err, climb_large_yaw_error_deg_);
          timeout_stop_latched_ = true;
          publishStoppedState();
        } else if (
          state_ == State::SLOPE_DRIVE &&
          (now - state_entered_time_).seconds() >= climb_timeout_sec_)
        {
          RCLCPP_ERROR(
            get_logger(),
            "SLOPE_DRIVE timeout(%.1fs) -- 신호가 계속 살아있는데 너무 오래 지속, 정지 후 래치"
            "(재시작 전까지 유지)",
            climb_timeout_sec_);
          timeout_stop_latched_ = true;
          publishStoppedState();
        }
        break;
      }
      case State::SLOPE_EXIT: {
        // [2026-08-27] SLOPE_DRIVE와 동일하게 /path 블렌딩 계속 -- 조향
        // wz 상한만 controlTimerCallback()에서 400/150dps 기준으로 더 크게
        // 잡는다(사용자 요청, computeBlendedTarget()가 이미 저장한
        // last_rel_offset_deg_를 아래 path_straight 판정에 재사용).
        updateBlendedTarget();
        const bool roll_ok = has_camera_imu_data_ &&
          std::abs(last_camera_roll_deg_) <= slope_exit_end_roll_deg_;
        const bool pitch_ok = last_pitch_signed_deg_ >= slope_exit_end_pitch_deg_;
        const bool path_straight_ok = last_rel_offset_deg_.has_value() &&
          std::abs(*last_rel_offset_deg_) <= path_straight_threshold_deg_;

        if (roll_ok && pitch_ok && path_straight_ok) {
          if (!slope_exit_end_cond_since_) {slope_exit_end_cond_since_ = now;}
          if ((now - *slope_exit_end_cond_since_).seconds() >= slope_exit_end_dwell_sec_) {
            state_ = State::RECOVERY;
            slope_exit_end_cond_since_.reset();
            break;
          }
        } else {
          slope_exit_end_cond_since_.reset();
        }

        // 신호 자체가 사라지는 경우의 fallback -- side==0이 side_exit_dwell_sec_
        // 이상 지속돼도 RECOVERY로(위 3조건 트리거와 동일한 출구).
        if (side == 0) {
          if (!exit_cond_since_) {exit_cond_since_ = now;}
          if ((now - *exit_cond_since_).seconds() >= side_exit_dwell_sec_) {
            state_ = State::RECOVERY;
            exit_cond_since_.reset();
            break;
          }
        } else {
          exit_cond_since_.reset();
        }

        if (state_ == State::SLOPE_EXIT &&
          (now - state_entered_time_).seconds() >= slope_exit_timeout_sec_)
        {
          RCLCPP_ERROR(
            get_logger(),
            "SLOPE_EXIT timeout(%.1fs) -- 탈출 조건이 안 갖춰지는데 너무 오래 지속, 정지 후 래치"
            "(재시작 전까지 유지)",
            slope_exit_timeout_sec_);
          timeout_stop_latched_ = true;
          publishStoppedState();
        }
        break;
      }
      case State::RECOVERY: {
        // 400/150 같은 공격적 차동에서 바로 MPPI로 넘기지 않고, 좌우 동일
        // recovery_motor_dps_로 recovery_dwell_sec_ 동안 직진 정착 후 IDLE.
        if ((now - state_entered_time_).seconds() >= recovery_dwell_sec_) {
          state_ = State::IDLE;
          entered_side_ = 0;
        }
        break;
      }
    }

    if (state_ != prev) {
      state_entered_time_ = now;
      publishState();
      RCLCPP_INFO(
        get_logger(),
        "state %s -> %s (yaw=%.1fdeg entered_side=%d)",
        stateName(prev), stateName(state_), last_yaw_deg_, entered_side_);
    }
  }

  void controlTimerCallback()
  {
    updateState();

    if (timeout_stop_latched_) {
      geometry_msgs::msg::Twist cmd;  // 0,0 -- 안전 정지 유지
      cmd_vel_safety_pub_->publish(cmd);
      publishDebug(-1, 0.0, 0.0, 0.0);
      return;
    }

    // IDLE이면 cmd_vel_safety에 아무 것도 안 보낸다 -- rmd_x8_driver_node의
    // cmd_vel_safety_timeout이 자연히 만료되며 MPPI(/cmd_vel_auto) 제어로
    // 매끄럽게 복귀한다.
    if (state_ == State::IDLE) {
      publishDebug(0, 0.0, 0.0, entry_yaw_deg_);
      return;
    }

    geometry_msgs::msg::Twist cmd;
    int state_enum = 0;
    double target_for_debug = blended_target_deg_;
    if (state_ == State::SLOPE_DRIVE) {
      state_enum = 1;
      const double yaw_err_deg = yawErrorDeg(blended_target_deg_);
      double wz = kp_steer_ * (yaw_err_deg * M_PI / 180.0);
      wz = std::clamp(wz, -wz_steer_max_, wz_steer_max_);
      cmd.linear.x = computeBoostedVx(climb_speed_, wz);
      cmd.angular.z = wz;
    } else if (state_ == State::SLOPE_EXIT) {
      // [2026-08-27] SLOPE_DRIVE와 동일하게 /path 블렌딩된 target을 쫓아가되
      // (사용자 요청으로 블렌딩 유지), wz 상한과 kp만 400/150dps 기준으로
      // 훨씬 크게 잡아서(computeExitWzMax()) 더 세게 돌 수 있게 한다.
      state_enum = 2;
      const double yaw_err_deg = yawErrorDeg(blended_target_deg_);
      double wz = kp_exit_ * (yaw_err_deg * M_PI / 180.0);
      wz = std::clamp(wz, -computeExitWzMax(), computeExitWzMax());
      cmd.linear.x = computeBoostedVx(computeExitBaseSpeed(), wz);
      cmd.angular.z = wz;
    } else {  // RECOVERY -- 좌우 동일 recovery_motor_dps_로 직진
      state_enum = 3;
      target_for_debug = entry_yaw_deg_;
      cmd.linear.x = recovery_motor_dps_ * (M_PI / 180.0) * wheel_radius_m_;
      cmd.angular.z = 0.0;
    }
    cmd_vel_safety_pub_->publish(cmd);
    publishDebug(state_enum, cmd.linear.x, cmd.angular.z, target_for_debug);
  }

  void publishDebug(int state_enum, double vx_cmd, double wz_cmd, double target_yaw_deg)
  {
    std_msgs::msg::Float64MultiArray out;
    out.data = {
      static_cast<double>(state_enum),
      static_cast<double>(entered_side_),
      target_yaw_deg,
      last_yaw_deg_,
      wrapDeg180(target_yaw_deg - last_yaw_deg_),
      vx_cmd,
      wz_cmd,
      (get_clock()->now() - state_entered_time_).seconds(),
      last_pitch_deg_,        // raw pitch 절댓값(deg, CLIMBING 블렌딩용)
      pitch_filtered_deg_,    // EMA 필터링된 pitch 절댓값(deg)
      pitch_rate_deg_s_,      // 필터링된 pitch의 변화율(deg/s, 음수=감소중)
      last_pitch_signed_deg_, // myAHRS+ raw pitch 부호 있는 값(deg, SLOPE_EXIT 트리거용)
      last_camera_roll_deg_,  // RealSense IMU roll(deg, SLOPE_EXIT 탈출 조건용)
      has_camera_imu_data_ ? 1.0 : 0.0,
    };
    debug_pub_->publish(out);
  }

  void publishState()
  {
    std_msgs::msg::String out;
    out.data = stateName(state_);
    recovery_state_pub_->publish(out);
  }

  void publishStoppedState()
  {
    std_msgs::msg::String out;
    out.data = "stopped";
    recovery_state_pub_->publish(out);
  }

  static const char * stateName(State s)
  {
    switch (s) {
      case State::IDLE: return "idle";
      case State::SLOPE_DRIVE: return "slope_drive";
      case State::SLOPE_EXIT: return "slope_exit";
      case State::RECOVERY: return "recovery";
    }
    return "unknown";
  }

  double track_width_m_;

  // 진입 신호 (PLACEHOLDER 인터페이스)
  std::string slope_side_topic_;
  double slope_side_signal_timeout_sec_;
  double side_enter_dwell_sec_;
  double side_exit_dwell_sec_;
  int8_t last_side_signal_{0};
  bool has_side_signal_{false};
  rclcpp::Time last_side_signal_time_;
  std::optional<rclcpp::Time> enter_cond_since_;
  std::optional<rclcpp::Time> exit_cond_since_;
  int8_t pending_side_{0};

  // 주행/조향
  double climb_speed_;
  double kp_steer_;
  double wz_steer_max_;
  double climb_timeout_sec_;
  double climb_large_yaw_error_deg_;

  // target heading 연속 블렌딩
  double climb_pitch_low_threshold_deg_;
  double climb_pitch_high_threshold_deg_;
  double climb_pitch_rate_threshold_deg_s_;
  double climb_pitch_rate_boost_gain_;
  double climb_min_blend_factor_;
  double climb_pitch_filter_alpha_;
  double climb_straight_grace_sec_;
  double blended_target_deg_{0.0};
  std::optional<double> last_rel_offset_deg_;
  double pitch_filtered_deg_{0.0};
  double pitch_rate_deg_s_{0.0};
  bool has_pitch_filter_state_{false};
  rclcpp::Time last_pitch_filter_time_;

  // 경사로 탈출 구간(SLOPE_EXIT)
  double wheel_radius_m_;
  double slope_exit_entry_pitch_deg_;
  double slope_exit_end_pitch_deg_;
  double slope_exit_end_roll_deg_;
  double path_straight_threshold_deg_;
  double slope_exit_trigger_dwell_sec_;
  double slope_exit_end_dwell_sec_;
  double slope_exit_side_motor_dps_;
  double slope_exit_other_motor_dps_;
  double slope_exit_timeout_sec_;
  double kp_exit_;
  std::string camera_imu_topic_;
  std::optional<rclcpp::Time> slope_exit_trigger_cond_since_;
  std::optional<rclcpp::Time> slope_exit_end_cond_since_;
  double last_pitch_signed_deg_{0.0};
  double last_camera_roll_deg_{0.0};
  bool has_camera_imu_data_{false};

  // RECOVERY(탈출 후 정착 구간)
  double recovery_motor_dps_;
  double recovery_dwell_sec_;

  // 블렌딩 목표: /path
  std::string path_topic_;
  double path_timeout_sec_;
  double path_tangent_lookahead_m_;
  nav_msgs::msg::Path::SharedPtr latest_path_;
  rclcpp::Time last_path_time_;

  std::string recovery_state_topic_;
  std::string debug_topic_;
  std::string cmd_vel_safety_topic_;
  double control_rate_hz_;

  State state_{State::IDLE};
  rclcpp::Time state_entered_time_;
  double entry_yaw_deg_{0.0};
  int8_t entered_side_{0};
  bool timeout_stop_latched_{false};

  double last_yaw_deg_{0.0};
  double last_pitch_deg_{0.0};
  bool has_imu_data_{false};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr camera_imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr slope_side_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_safety_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 실차 튜닝용 동적 파라미터 콜백(onParamUpdate() 참고)
  std::unordered_map<std::string, double *> tunable_params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlopeTraversePlanCNode>());
  rclcpp::shutdown();
  return 0;
}
