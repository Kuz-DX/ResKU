#include "our_mppi_critics/slope_critic.hpp"
#include <algorithm>
#include <cmath>
#include "nav2_mppi_controller/tools/utils.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xnoalias.hpp"

namespace mppi::critics
{

void SlopeCritic::initialize()
{
  auto node = parent_.lock();

  node->declare_parameter(name_ + ".cost_power", 1);
  node->declare_parameter(name_ + ".cost_weight", 5.0f);
  node->declare_parameter(name_ + ".max_slope_deg", 30.0f);
  node->declare_parameter(name_ + ".critical_slope_deg", 25.0f);
  // [2026-08-23 v2] 예측 기반 차동조향. "지금 roll을 상쇄"하는 반응형 방식
  // (실기 부호 검증 필요했던 v1) 대신, 현재 roll/pitch/yaw로 지금 이 순간
  // 경사면이 월드 좌표계에서 어느 방향으로 기울어져 있는지(하강 방향
  // azimuth)를 역산하고, 각 후보 궤적의 미래 yaw(data.trajectories.yaws,
  // 후보마다 다름)에 대해 "그 방향으로 돌면 roll이 얼마나 될지"를 예측해서
  // 그 예측 roll이 작은(=경사가 완만해지는) 후보를 선호하게 만든다. roll/
  // pitch/yaw 전부 같은 IMU/같은 부호 규약으로 계산하므로 자체 일관성이
  // 있어 장착 방향에 따른 부호 뒤집기가 필요 없다(v1은 필요했음).
  //
  // turn_gate: "path가 실제로 옆으로 휘기 시작할 때만" 이 보정이 개입하게
  // 만드는 게이트. 현재 헤딩과 path 위 lookahead 지점 방향 사이 각도
  // (PathAngleCritic과 동일한 posePointAngle 계산, 단발성 -- 후보별 아님)가
  // turn_gate_start_deg 밑이면 게이트 0(완전 비활성 -- 직진 중엔 후보들의
  // 미세한 yaw 샘플링 노이즈만으로 편향이 걸리는 걸 방지), turn_gate_full_deg
  // 이상이면 게이트 1(완전 활성), 그 사이는 선형 보간. 급격한 on/off 전환은
  // 오늘 겪은 "비용 불연속점에서 MPPI가 후보 평균 내며 헤매는" 문제와 같은
  // 종류의 함정이 될 수 있어 일부러 선형 램프로 만듦.
  node->declare_parameter(name_ + ".predicted_roll_cost_weight", 3.0f);
  node->declare_parameter(name_ + ".turn_gate_start_deg", 8.0f);
  node->declare_parameter(name_ + ".turn_gate_full_deg", 20.0f);
  node->declare_parameter(name_ + ".turn_gate_offset_from_furthest", 4);
  node->declare_parameter(name_ + ".min_active_deg", 3.0f);

  power_ = node->get_parameter(name_ + ".cost_power").as_int();
  weight_ = node->get_parameter(name_ + ".cost_weight").as_double();
  max_slope_deg_ = node->get_parameter(name_ + ".max_slope_deg").as_double();
  critical_slope_deg_ = node->get_parameter(name_ + ".critical_slope_deg").as_double();
  predicted_roll_weight_ = node->get_parameter(name_ + ".predicted_roll_cost_weight").as_double();
  turn_gate_start_deg_ = node->get_parameter(name_ + ".turn_gate_start_deg").as_double();
  turn_gate_full_deg_ = node->get_parameter(name_ + ".turn_gate_full_deg").as_double();
  turn_gate_offset_from_furthest_ =
    node->get_parameter(name_ + ".turn_gate_offset_from_furthest").as_int();
  min_active_deg_ = node->get_parameter(name_ + ".min_active_deg").as_double();

  RCLCPP_INFO(
    logger_,
    "SlopeCritic instantiated: cost_power=%d cost_weight=%f predicted_roll_cost_weight=%f "
    "turn_gate=[%.1f,%.1f]deg min_active_deg=%f",
    power_, weight_, predicted_roll_weight_, turn_gate_start_deg_, turn_gate_full_deg_,
    min_active_deg_);
}

void SlopeCritic::score(CriticData & data)
{
  // ⚠️ 절대 이 안에서 /imu(또는 다른 토픽)를 직접 구독하지 말 것. 예전에
  // critic 내부에서 직접 /imu 구독을 만들었을 때 controller_server가
  // 재현 가능한 SIGSEGV로 죽는 문제가 있었음(MPPI critic은 옵티마이저
  // 스레드에서 호출되는데, 별도로 만든 구독의 콜백은 executor 스레드에서
  // 호출되어 두 스레드가 얽히는 게 원인으로 보임). 대신 이미 매 score()
  // 호출마다 전달되는 현재 로봇 pose(data.state.pose, EKF가 융합한
  // orientation이 odom->base_link TF를 통해 이미 반영됨)에서 roll/pitch/yaw를
  // 뽑아 쓴다 -- 추가 구독이 전혀 필요 없어 이 문제 자체가 안 생긴다.
  if (!enabled_) {return;}

  const auto & q = data.state.pose.pose.orientation;
  const float qw = static_cast<float>(q.w);
  const float qx = static_cast<float>(q.x);
  const float qy = static_cast<float>(q.y);
  const float qz = static_cast<float>(q.z);
  const float roll_now = std::atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
  const float pitch_now = std::asin(std::clamp(2.0f * (qw * qy - qz * qx), -1.0f, 1.0f));
  const float yaw_now = tf2::getYaw(data.state.pose.pose.orientation);
  const float pitch_deg = std::abs(pitch_now) * 180.0f / M_PI;
  const float slope_magnitude_rad = std::hypot(roll_now, pitch_now);
  const float slope_magnitude_deg = slope_magnitude_rad * 180.0f / M_PI;

  if (slope_magnitude_deg < min_active_deg_) {return;}

  using xt::evaluation_strategy::immediate;

  // --- [1] 예측 기반 차동조향: 어느 쪽으로 돌면 roll이 완만해지는지 -------
  // 하강 방향(fall line) azimuth를 현재 roll/pitch/yaw로 역산 (small-angle
  // 평면 근사: roll ~= slope*sin(yaw-azimuth), pitch ~= slope*cos(yaw-azimuth)
  // 이므로 azimuth = yaw - atan2(roll, pitch)).
  const float fall_line_azimuth = yaw_now - std::atan2(roll_now, pitch_now);

  // turn_gate: path가 실제로 얼마나 옆으로 휘라고 요구하는지(현재 헤딩 vs
  // lookahead 지점 방향) -- 이 값이 작으면(직진 구간) 게이트 0, 커지면
  // (path가 휘기 시작하면) 게이트가 선형으로 켜짐. path가 아직 없거나
  // 너무 짧으면(예: 시작 직후) 게이트 0으로 안전하게 처리.
  float gate = 0.0f;
  const size_t path_size = data.path.x.shape(0);
  if (path_size >= 2) {
    utils::setPathFurthestPointIfNotSet(data);
    const size_t offseted_idx = std::min(
      static_cast<size_t>(*data.furthest_reached_path_point + turn_gate_offset_from_furthest_),
      path_size - 1);
    const double goal_x = data.path.x(offseted_idx);
    const double goal_y = data.path.y(offseted_idx);
    // vx_min=0(후진 불가)인 현재 로봇 설정 기준 forward_preference=true.
    const float turn_demand_rad =
      utils::posePointAngle(data.state.pose.pose, goal_x, goal_y, true);
    const float turn_demand_deg = turn_demand_rad * 180.0f / M_PI;
    const float span = std::max(1e-3f, turn_gate_full_deg_ - turn_gate_start_deg_);
    gate = std::clamp((turn_demand_deg - turn_gate_start_deg_) / span, 0.0f, 1.0f);
  }

  // 후보별 미래 yaw(data.trajectories.yaws, batch x time_steps)에 대해
  // 예측 roll을 계산 -- 후보마다 실제로 다른 값이 나오므로(예전 no-op
  // 버그와 달리) 소프트맥스에서 상쇄되지 않고 실제 후보 선택에 영향을 준다.
  const auto predicted_roll = slope_magnitude_rad * xt::sin(data.trajectories.yaws - fall_line_azimuth);
  const auto predictive_cost =
    gate * predicted_roll_weight_ * xt::mean(xt::abs(predicted_roll), {1}, immediate);

  // --- [2] 급경사 제동(steep-pitch brake): pitch 기준, path 방향과 무관 ---
  // 예전 버전은 모든 후보에 똑같은 비용을 더해서(no-op) 소프트맥스에서
  // 완전히 상쇄됐다. 이번엔 후보 자신의 평균 전진속도(vx)에 비례시켜서 --
  // 같은 급경사 상황에서도 "더 빨리 밀고 들어가려는" 후보일수록 더 크게
  // 벌점을 받고, "속도를 줄이려는" 후보는 상대적으로 덜 받는다.
  float severity_ratio = 0.0f;
  if (pitch_deg >= max_slope_deg_) {
    severity_ratio = 3.0f;
  } else if (pitch_deg >= critical_slope_deg_) {
    severity_ratio = (pitch_deg - critical_slope_deg_) / (max_slope_deg_ - critical_slope_deg_);
  }
  const auto forward_vx = xt::maximum(data.state.vx, 0.0f);
  const auto steep_cost = severity_ratio * weight_ * xt::mean(forward_vx, {1}, immediate);

  // doldrive_ws fix (2026-08-10, SkidCritic과 동일 패턴): data.costs에 plain
  // `+=`으로 합치면(별도로 힙 할당된 같은 크기의 xtensor를 스칼라 곱해서
  // 결합) 16바이트 정렬 버퍼가 나와서, 이후 Optimizer::updateControlSequence()
  // 의 AVX256 SIMD store(32바이트 정렬 요구)에서 SIGSEGV가 재현됨. cost를
  // 명시적으로 한 번 materialize한 뒤 noalias로 기존 costs_ 버퍼에 제자리로
  // 더해서 재할당 자체를 피한다 -- SkidCritic/기존 SlopeCritic과 동일 패턴.
  xt::xtensor<float, 1> cost = predictive_cost + steep_cost;

  if (power_ > 1u) {
    xt::noalias(data.costs) += xt::pow(cost, static_cast<float>(power_));
  } else {
    xt::noalias(data.costs) += cost;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::SlopeCritic,
  mppi::critics::CriticFunction)
