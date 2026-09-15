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

// slope_traverse_blend_node: slope_traverse_node.cpp의 "PlanB" 대안 노드.
//
// [2026-08-28 신규, 2차 재설계] slope_traverse_node.cpp(PlanA)는 SLOPE_EXIT
// 진입(pitch 부호 전환) 즉시 pure pursuit 곡률 제어로 전환하는데, 이건
// dolbotz 패키지의 purepursuit_node를 이식한 것이라 TF(camera_link->
// base_link) 변환에 의존한다(빌드 의존성 tf2_ros/tf2_geometry_msgs 신규
// 추가, path_base_xy_ 등 새 경로). 실차 테스트 없이 만든 코드라 TF 설정
// (정적 TF, 좌표계 가정 등)에 문제가 있을 위험을 배제할 수 없어서, "테스트를
// 못 하는 상황에서의 차선책"으로 TF에 전혀 의존하지 않는 이 노드를 별도로
// 만들었다.
//
// [1차 -> 2차 수정] 처음엔 SLOPE_DRIVE와 같은 blend+kp_steer_ P제어
// 구조를 유지하면서 SLOPE_EXIT 동안만 blend를 강제로 1.0 고정하는 방식
// (force_full_blend_)으로 만들었는데, 사용자 지적대로 이러면 "target
// 각도(alpha)를 100% 그대로 쓴다"는 점에서 이미 pure pursuit과 사실상
// 같아지면서도, wz 계산은 여전히 vx와 무관한 고정 게인 P제어(kp_steer_*alpha)
// 라 pure pursuit의 곡률 결합(wz=vx*curvature)만큼 정확하지 않은 "어설픈
// 흉내"였다. PID로 P제어를 보강하는 대안도 검토했으나, 원래 문제(blend가
// 낮을 때 목표 자체가 entry_yaw_deg_ 쪽으로 쏠려버리는 것)는 목표 계산이
// 틀린 것이지 추종 정밀도 문제가 아니라서 PID로는 해결이 안 된다고 판단.
// 그래서 2차로 아예 SLOPE_EXIT의 조향을 PlanA와 동일한 pure pursuit
// 곡률식(wz=vx*2y/L^2, purepursuit.py와 동일 공식)으로 바꾸되, TF 변환만
// 뺐다 -- /path의 raw x/y를 그대로 robot-relative 좌표(x=전방, y=좌측)로
// 간주한다(SLOPE_DRIVE의 computeRelativePathOffsetDeg()가 이미 쓰던 것과
// 동일한 근사, path_relay_params.yaml 주석 참고: frame_id는 보통
// camera_link이고 TF 변환의 정확도는 아직 검증 전이라는 전제가 이미
// 깔려 있었음). 즉 PlanB = "PlanA의 pure pursuit 곡률식은 그대로 쓰되
// TF 변환만 생략한 버전"이다 -- SLOPE_DRIVE 쪽은 두 Plan이 완전히 동일.
//
// 상태머신/센서/토픽 인터페이스는 PlanA(slope_traverse_node.cpp)와 완전히
// 동일하다(IDLE -> SLOPE_DRIVE -> SLOPE_EXIT -> RECOVERY -> IDLE). 유일한
// 차이는 SLOPE_EXIT의 곡률 계산에 쓰는 목표점 좌표가 TF로 base_link 변환된
// 것이냐(PlanA), /path의 raw 좌표를 그대로 쓰느냐(PlanB, 이 파일)뿐이다.
//
// 상태머신 (PlanA와 동일, 아래는 요약):
//   IDLE -> SLOPE_DRIVE -> SLOPE_EXIT -> RECOVERY -> IDLE
//
//   - IDLE: slope_side_topic_(기본 '/terrain/slope_side_signal',
//     +1=right_slope/-1=left_slope)에서 side != 0이 side_enter_dwell_sec_
//     이상 유지되면 진입. entry_yaw_deg_ = 진입 시점 yaw(오프셋 없음).
//   - SLOPE_DRIVE: vx=climb_speed_ 기반, target heading은 entry_yaw_deg_에서
//     시작해서 매 tick /path 접선 방향 쪽으로 블렌딩 계수만큼 당겨진다
//     (updateBlendedTarget() 참고). 블렌딩 계수는 pitch를 EMA로 필터링해
//     그 값(레벨)과 변화율(하강 방향만)을 함께 보고 smoothstep으로 계산
//     (updatePitchFilterAndComputeBlend()). 조향(wz)은 kp_steer_/
//     wz_steer_max_로 계산하고, computeBoostedVx()로 "안쪽 바퀴는
//     climb_speed_ 그대로, 바깥쪽만 부스트"하는 vx를 낸다. myAHRS+ pitch
//     (부호 있는 값, last_pitch_signed_deg_)가 slope_exit_entry_pitch_deg_
//     (기본 0deg, 즉 부호 전환 감지) 이하로 내려가면
//     slope_exit_trigger_dwell_sec_ 디바운스 후 SLOPE_EXIT로 전환.
//   - SLOPE_EXIT("경사로 탈출 구간"): PlanA와 동일하게 pure pursuit 곡률
//     제어로 전환한다 -- computeSlopeExitPursuitCurvature()가 /path의
//     raw poses에서 "로봇 원점(0,0) 기준 직선거리"가
//     path_tangent_lookahead_m_ 이상인 첫 목표점을 찾고(짧으면 마지막
//     점으로 폴백), 그 점까지의 "실제" 거리 L로 kappa=2y/L^2를 계산한다
//     (purepursuit.py _compute_pure_pursuit()와 동일 공식). wz = vx *
//     kappa, vx=slope_exit_speed_(기본 0.4m/s, climb_speed_와 동일)를
//     computeBoostedVx() 부스트 없이 그대로 발행 -- 부스트하면 곡률
//     계산에 쓴 vx와 실제 발행 vx가 달라져서 pure pursuit의 전제가
//     깨지기 때문(PlanA와 동일한 이유). wz 안전 상한은
//     computeSlopeExitWzMax()(slope_exit_wz_base_*_dps_ 기준, 기본
//     500/0dps -> ~2.0rad/s). SLOPE_DRIVE의 updateBlendedTarget()
//     호출/blend 계산과 last_rel_offset_deg_(SLOPE_EXIT의
//     path_straight_ok 판정용)는 이 변경과 무관하게 그대로 유지된다.
//     탈출 조건은 두 센서를 동시에(AND) 만족해야 함
//     (slope_exit_end_dwell_sec_ 디바운스): ① RealSense IMU
//     (camera_imu_topic_)의 roll이 slope_exit_end_roll_deg_(기본 3deg)
//     이내, ② /path가 직진(computeRelativePathOffsetDeg() 절댓값이
//     path_straight_threshold_deg_ 이내). 둘 다 갖춰지면(혹은 side 신호
//     자체가 사라지면) RECOVERY로 전환. slope_exit_timeout_sec_(기본
//     15초) 넘도록 조건이 안 갖춰지면 안전망으로 정지-래치.
//   - RECOVERY: SLOPE_EXIT에서 바로 MPPI로 넘기지 않고, 좌우 동일
//     recovery_motor_dps_(기본 200)로 recovery_dwell_sec_(기본 1초) 동안
//     직진 정착한 뒤 IDLE 복귀(entered_side_ 리셋, MPPI가 이어받음).
//
// 안전 fallback: SLOPE_DRIVE 중 yaw_error가 climb_large_yaw_error_deg_를
// 넘거나, climb_timeout_sec_ 넘으면 그대로 정지 -- timeout_stop_latched_로
// 래치하고 자동으로 안 풀리게 해서 사람이 원인 확인 후 재시작하는 걸
// 전제로 한다.
//
// computeBoostedVx(): rmd_x8_driver_node의 대칭 스큐-스티어 역기구학
// (v_left=v-wz*halftrack, v_right=v+wz*halftrack)에 vx를 그대로 넘기면
// 회전할수록 안쪽 바퀴가 base_speed 밑으로 깎인다. 매뉴얼 주행 실측(한쪽
// 300~350dps/반대쪽 200dps 조합이 잘 됐다는 사용자 피드백)을 참고해
// "안쪽은 base_speed 그대로, 바깥쪽만 부스트"하도록
// v = base_speed+|wz|*halftrack으로 올려서 보낸다(SLOPE_DRIVE에서만 사용
// -- SLOPE_EXIT은 pure pursuit이라 부스트하지 않음, 이유는
// controlTimerCallback() 참고).
//
// /cmd_vel_safety 재사용 근거: rmd_x8_driver_node.py의 _control_loop()를
// 직접 읽고 확인함 -- cmd_vel_safety_timeout(기본 0.5s) 이내 최근 수신되면
// /cmd_vel(MPPI 출력)을 완전히 무시하고 그 값을 그대로 쓰는 토픽 기반
// 범용 최우선순위 메커니즘. IDLE일 때는 publish를 멈춰서 timeout이 자연히
// 만료되며 MPPI 제어로 복귀한다.
class SlopeTraverseBlendNode : public rclcpp::Node
{
public:
  SlopeTraverseBlendNode()
  : Node("slope_traverse_blend_node")
  {
    track_width_m_ = declare_parameter<double>("track_width_m", 0.4904);

    // --- 진입: 인지팀 signed left/right 신호 ------------------------------
    slope_side_topic_ = declare_parameter<std::string>(
      "slope_side_topic", "/terrain/slope_side_signal");
    slope_side_signal_timeout_sec_ =
      declare_parameter<double>("slope_side_signal_timeout_sec", 1.0);
    side_enter_dwell_sec_ = declare_parameter<double>("side_enter_dwell_sec", 0.25);
    side_exit_dwell_sec_ = declare_parameter<double>("side_exit_dwell_sec", 0.25);

    // --- 주행/조향 (SLOPE_DRIVE) -----------------------------------------
    climb_speed_ = declare_parameter<double>("climb_speed", 0.4);
    kp_steer_ = declare_parameter<double>("kp_steer", 0.4);
    wz_steer_max_ = declare_parameter<double>("wz_steer_max", 0.5);
    climb_timeout_sec_ = declare_parameter<double>("climb_timeout_sec", 600.0);
    climb_large_yaw_error_deg_ = declare_parameter<double>("climb_large_yaw_error_deg", 90.0);

    // --- target heading 연속 블렌딩 (SLOPE_DRIVE 전용) --------------------
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
    climb_straight_grace_sec_ = declare_parameter<double>("climb_straight_grace_sec", 0.0);

    // --- SLOPE_EXIT (pure pursuit, TF 미사용) -----------------------------
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.1125);
    slope_exit_entry_pitch_deg_ =
      declare_parameter<double>("slope_exit_entry_pitch_deg", 0.0);
    slope_exit_speed_ = declare_parameter<double>("slope_exit_speed", 0.4);
    // [2026-08-29] computeSlopeExitWzMax()의 dps 기준값 -- 300/200dps
    // (wz≈0.40rad/s)에서 500/0dps(wz≈2.0rad/s)로 올림. pure pursuit
    // 곡률식이 vx=slope_exit_speed_/Ld=path_tangent_lookahead_m_ 기준으로
    // 낼 수 있는 이론상 최대 wz(alpha=90도일 때 2*vx/Ld, 기본값 기준
    // ≈1.6rad/s)보다 옛 클램프(0.40)가 훨씬 작아서, path가 큰 조향을
    // 요구해도 못 따라가는 문제가 있었다(사용자 확인). 500/0dps는 이론상
    // 최대치보다 25% 여유를 둬서, 평소엔 클램프가 거의 안 걸리고 path가
    // 요구하는 wz를 그대로 통과시키며 극단적인 상황에만 안전망으로
    // 작동한다.
    slope_exit_wz_base_side_dps_ =
      declare_parameter<double>("slope_exit_wz_base_side_dps", 500.0);
    slope_exit_wz_base_other_dps_ =
      declare_parameter<double>("slope_exit_wz_base_other_dps", 0.0);
    slope_exit_end_roll_deg_ = declare_parameter<double>("slope_exit_end_roll_deg", 3.0);
    path_straight_threshold_deg_ =
      declare_parameter<double>("path_straight_threshold_deg", 5.0);
    slope_exit_trigger_dwell_sec_ =
      declare_parameter<double>("slope_exit_trigger_dwell_sec", 0.2);
    slope_exit_end_dwell_sec_ = declare_parameter<double>("slope_exit_end_dwell_sec", 0.3);
    slope_exit_timeout_sec_ = declare_parameter<double>("slope_exit_timeout_sec", 15.0);
    camera_imu_topic_ = declare_parameter<std::string>("camera_imu_topic", "/drive/camera/imu");
    // [PlanB, 3차] /path(카메라 파이프라인이 이미 레벨링해서 내보내는 값)를
    // base_link 기준으로 옮기기 위한 순수 평행이동량 -- base_link->camera_link
    // static TF의 x translation과 동일(reduced_odom_bringup.launch.py의
    // base_to_camera_tf 참고, 마운트 재측정 시 같이 갱신할 것). y는 좌우
    // 오프셋이 0(카메라가 base_link 중앙축에 정렬 마운트)이라 별도 파라미터
    // 없이 0으로 고정.
    camera_to_base_x_offset_m_ =
      declare_parameter<double>("camera_to_base_x_offset_m", 0.34815);

    // --- RECOVERY: 탈출 후 정착 구간 --------------------------------------
    recovery_motor_dps_ = declare_parameter<double>("recovery_motor_dps", 200.0);
    recovery_dwell_sec_ = declare_parameter<double>("recovery_dwell_sec", 1.0);

    // --- 블렌딩 목표 / pure pursuit lookahead 공용: /path -----------------
    path_topic_ = declare_parameter<std::string>("path_topic", "/path");
    path_timeout_sec_ = declare_parameter<double>("path_timeout_sec", 1.0);
    path_tangent_lookahead_m_ = declare_parameter<double>("path_tangent_lookahead_m", 0.5);

    recovery_state_topic_ = declare_parameter<std::string>(
      "recovery_state_topic", "/drive/slope_traverse_state");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/slope_traverse/debug");
    cmd_vel_safety_topic_ = declare_parameter<std::string>(
      "cmd_vel_safety_topic", "/cmd_vel_safety");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);

    recovery_state_pub_ = create_publisher<std_msgs::msg::String>(
      recovery_state_topic_, rclcpp::QoS(1).transient_local());
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(debug_topic_, 10);
    cmd_vel_safety_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_safety_topic_, 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&SlopeTraverseBlendNode::imuCallback, this, std::placeholders::_1));
    camera_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      camera_imu_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&SlopeTraverseBlendNode::cameraImuCallback, this, std::placeholders::_1));
    slope_side_sub_ = create_subscription<std_msgs::msg::Int8>(
      slope_side_topic_, 10,
      std::bind(&SlopeTraverseBlendNode::slopeSideCallback, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1),
      std::bind(&SlopeTraverseBlendNode::pathCallback, this, std::placeholders::_1));

    state_entered_time_ = get_clock()->now();

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SlopeTraverseBlendNode::controlTimerCallback, this));

    // 실차 튜닝 중 재시작 없이 ros2 param set으로 값을 바로 반영하기 위한
    // 동적 콜백. 토픽명(string)/control_rate_hz(타이머 주기가 생성 시점에
    // 고정)는 제외.
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
      {"slope_exit_speed", &slope_exit_speed_},
      {"slope_exit_wz_base_side_dps", &slope_exit_wz_base_side_dps_},
      {"slope_exit_wz_base_other_dps", &slope_exit_wz_base_other_dps_},
      {"slope_exit_end_roll_deg", &slope_exit_end_roll_deg_},
      {"path_straight_threshold_deg", &path_straight_threshold_deg_},
      {"slope_exit_trigger_dwell_sec", &slope_exit_trigger_dwell_sec_},
      {"slope_exit_end_dwell_sec", &slope_exit_end_dwell_sec_},
      {"slope_exit_timeout_sec", &slope_exit_timeout_sec_},
      {"recovery_motor_dps", &recovery_motor_dps_},
      {"recovery_dwell_sec", &recovery_dwell_sec_},
      {"path_timeout_sec", &path_timeout_sec_},
      {"path_tangent_lookahead_m", &path_tangent_lookahead_m_},
      {"camera_to_base_x_offset_m", &camera_to_base_x_offset_m_},
    };
    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&SlopeTraverseBlendNode::onParamUpdate, this, std::placeholders::_1));

    publishState();

    RCLCPP_INFO(
      get_logger(),
      "slope_traverse_blend_node started (PlanB, TF-free pure pursuit): "
      "slope_side_topic=%s climb_speed=%.2f kp_steer=%.2f wz_steer_max=%.2f "
      "slope_exit_speed=%.2f",
      slope_side_topic_.c_str(), climb_speed_, kp_steer_, wz_steer_max_, slope_exit_speed_);
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
  // base_speed 그대로, 바깥쪽만 부스트"(파일 상단 docstring 참고).
  double computeBoostedVx(double base_speed, double wz) const
  {
    return base_speed + std::abs(wz) * (track_width_m_ / 2.0);
  }

  // dps 차동 조합(side/other)을 skid-steer 역산해 wz로 환산 --
  // computeBoostedVx()의 역방향 공식.
  double wzFromDpsPair(double side_dps, double other_dps) const
  {
    const double side_mps = side_dps * (M_PI / 180.0) * wheel_radius_m_;
    const double other_mps = other_dps * (M_PI / 180.0) * wheel_radius_m_;
    return std::abs(side_mps - other_mps) / track_width_m_;
  }

  // SLOPE_EXIT 전용 wz 상한 -- slope_exit_wz_base_*_dps_(기본 500/0)
  // 기준 wz 하나로 고정.
  double computeSlopeExitWzMax() const
  {
    return wzFromDpsPair(slope_exit_wz_base_side_dps_, slope_exit_wz_base_other_dps_);
  }

  // [PlanB, 3차] PlanA(slope_traverse_node.cpp)의
  // computeSlopeExitPursuitCurvature()와 동일한 알고리즘(purepursuit.py
  // 이식)이지만 TF 변환 없이 /path를 그대로 쓴다. 단, /path의 x/y를 곧바로
  // "로봇 원점(0,0) 기준"으로 두지는 않는다 -- /path는 외부 인지 시스템이
  // 지면 기준으로 레벨링해서 내보내는 값이라(frame_id는
  // "camera_link"라고 찍혀 나가지만 실제로는 회전이 이미 다 빠진 상태),
  // 여기서 base_link->camera_link static TF의 회전(47도 마운트 pitch 등)을
  // 다시 적용하면 이미 레벨링된 좌표에 회전을 이중으로 거는 꼴이 된다(TF를
  // 쓰는 PlanA가 이 문제를 안고 있을 수 있음). 대신 남는 차이는 카메라
  // 원점과 base_link 원점 사이의 순수 평행이동뿐이므로(카메라가
  // base_link보다 camera_to_base_x_offset_m_만큼 앞에 마운트, 좌우 오프셋은
  // 0), x에 그 상수만 더해서 base_link 기준으로 옮긴다. "로봇 원점(0,0,
  // base_link 기준) 기준 직선거리"가 path_tangent_lookahead_m_ 이상인 첫
  // 지점을 목표점으로 찾고, 경로 전체가 그보다 짧으면 마지막 점으로
  // 폴백한다. 그 점까지의 "실제" 거리 L로 kappa=2y/L^2를 계산
  // (y/L=sin(alpha) 관계로 2*sin(alpha)/L과 동치, atan2/sin 왕복 없이
  // 바로 계산).
  std::optional<double> computeSlopeExitPursuitCurvature()
  {
    if (!latest_path_ || latest_path_->poses.empty()) {
      return std::nullopt;
    }
    if ((get_clock()->now() - last_path_time_).seconds() > path_timeout_sec_) {
      return std::nullopt;
    }

    const auto & poses = latest_path_->poses;
    double target_x = 0.0;
    double target_y = 0.0;
    bool found = false;
    for (const auto & pose : poses) {
      const double x = pose.pose.position.x + camera_to_base_x_offset_m_;
      const double y = pose.pose.position.y;
      if (std::hypot(x, y) >= path_tangent_lookahead_m_) {
        target_x = x;
        target_y = y;
        found = true;
        break;
      }
    }
    if (!found) {
      target_x = poses.back().pose.position.x + camera_to_base_x_offset_m_;
      target_y = poses.back().pose.position.y;
    }

    const double look_dist = std::max(std::hypot(target_x, target_y), 1e-3);
    return 2.0 * target_y / (look_dist * look_dist);
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
    last_pitch_signed_deg_ = pitch * 180.0 / M_PI;
    has_imu_data_ = true;
  }

  // RealSense(D455) 자체 IMU -- SLOPE_EXIT 탈출 조건의 roll 값 전용.
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

  // target heading 블렌딩 계수(0~1) 계산 -- SLOPE_DRIVE 전용(SLOPE_EXIT은
  // pure pursuit이라 이 함수를 쓰지 않는다). pitch를 EMA로 필터링해(raw
  // IMU 노이즈가 미분에 그대로 실리는 걸 막음) 그 필터링된 값과, 변화율
  // (하강 방향만) 둘 다 반영. dt는 이 노드의 control_timer_가 매 tick
  // 실제로 호출된 간격을 그대로 쓰므로 안전하다.
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

  // SLOPE_DRIVE/SLOPE_EXIT 공용 -- target heading을 entry_yaw_deg_에서
  // 그 순간 /path 접선 방향 쪽으로 블렌딩 계수만큼 당긴 blended_target_deg_를
  // 갱신한다. last_rel_offset_deg_에 이번 tick의 원본 signed 오프셋도
  // 같이 저장해서(nullopt=path 신뢰 불가), SLOPE_EXIT의 path_straight
  // 탈출 판정이 이 함수를 다시 호출할 필요 없이 재사용할 수 있게 한다.
  // (SLOPE_EXIT은 조향 자체는 이 함수의 결과를 안 쓰고
  // computeSlopeExitPursuitCurvature()를 따로 쓰지만, last_rel_offset_deg_/
  // blended_target_deg_는 탈출 판정/디버그 표시에 계속 필요해서 그대로
  // 호출한다 -- PlanA와 동일한 구조.)
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
  // path가 없거나/오래됐거나/너무 짧으면 nullopt. (SLOPE_DRIVE의 블렌딩
  // target용 -- SLOPE_EXIT의 실제 조향은
  // computeSlopeExitPursuitCurvature()를 따로 쓴다.)
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
        // 블렌딩한다(updateBlendedTarget() 참고).
        updateBlendedTarget();

        // myAHRS+ pitch(부호 있는 값)가 slope_exit_entry_pitch_deg_(기본
        // 0deg, 즉 부호 전환) 이하로 내려가면 SLOPE_EXIT(경사로 탈출
        // 구간, pure pursuit)로 전환한다.
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
        // 탈출 판정에 필요한 last_rel_offset_deg_/blended_target_deg_
        // (디버그 표시용)는 SLOPE_DRIVE와 동일하게 계속 갱신한다 -- 실제
        // 조향(wz)은 controlTimerCallback()에서
        // computeSlopeExitPursuitCurvature()로 따로 계산한다.
        updateBlendedTarget();
        const bool roll_ok = has_camera_imu_data_ &&
          std::abs(last_camera_roll_deg_) <= slope_exit_end_roll_deg_;
        const bool path_straight_ok = last_rel_offset_deg_.has_value() &&
          std::abs(*last_rel_offset_deg_) <= path_straight_threshold_deg_;

        if (roll_ok && path_straight_ok) {
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
        // 공격적 조향에서 바로 MPPI로 넘기지 않고, 좌우 동일
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
      // [PlanB, 2차] PlanA와 동일한 pure pursuit 곡률 제어(TF만 생략) --
      // wz = vx * curvature (curvature = 2*y/L^2, L=목표점까지 실거리,
      // computeSlopeExitPursuitCurvature() 참고). vx는 slope_exit_speed_를
      // 그대로 쓰고 computeBoostedVx()로 부스트하지 않는다 -- 부스트하면
      // 곡률 계산에 쓴 vx와 실제 발행 vx가 달라져서 "이 wz는 이 vx를
      // 전제로 계산됨"이라는 pure pursuit의 전제가 깨지기 때문(PlanA와
      // 동일한 이유). wz 안전 상한은 computeSlopeExitWzMax()
      // (slope_exit_wz_base_*_dps_ 기준) 재사용 -- 실측에서 곡선이 자주
      // 잘리면 ros2 param set으로 그 값을 올릴 것. path가 없거나 오래됐으면
      // wz=0으로 안전하게 직진.
      state_enum = 2;
      double wz = 0.0;
      const auto curvature = computeSlopeExitPursuitCurvature();
      if (curvature.has_value()) {
        wz = slope_exit_speed_ * (*curvature);
      }
      const double slope_exit_wz_max = computeSlopeExitWzMax();
      wz = std::clamp(wz, -slope_exit_wz_max, slope_exit_wz_max);
      cmd.linear.x = slope_exit_speed_;
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
      last_pitch_deg_,        // raw pitch 절댓값(deg, SLOPE_DRIVE 블렌딩용)
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

  // 진입 신호
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

  // 주행/조향 (SLOPE_DRIVE)
  double climb_speed_;
  double kp_steer_;
  double wz_steer_max_;
  double climb_timeout_sec_;
  double climb_large_yaw_error_deg_;

  // target heading 연속 블렌딩 (SLOPE_DRIVE 전용)
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

  // 경사로 탈출 구간(SLOPE_EXIT, pure pursuit)
  double wheel_radius_m_;
  double slope_exit_entry_pitch_deg_;
  double slope_exit_speed_;
  double slope_exit_wz_base_side_dps_;
  double slope_exit_wz_base_other_dps_;
  double slope_exit_end_roll_deg_;
  double path_straight_threshold_deg_;
  double slope_exit_trigger_dwell_sec_;
  double slope_exit_end_dwell_sec_;
  double slope_exit_timeout_sec_;
  std::string camera_imu_topic_;
  double camera_to_base_x_offset_m_;
  std::optional<rclcpp::Time> slope_exit_trigger_cond_since_;
  std::optional<rclcpp::Time> slope_exit_end_cond_since_;
  double last_pitch_signed_deg_{0.0};
  double last_camera_roll_deg_{0.0};
  bool has_camera_imu_data_{false};

  // RECOVERY(탈출 후 정착 구간)
  double recovery_motor_dps_;
  double recovery_dwell_sec_;

  // 블렌딩 목표 / pure pursuit lookahead 공용: /path
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
  rclcpp::spin(std::make_shared<SlopeTraverseBlendNode>());
  rclcpp::shutdown();
  return 0;
}
