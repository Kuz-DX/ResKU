#include "our_mppi_critics/accel_critic.hpp"
#include <algorithm>
#include <cmath>
#include "xtensor/xarray.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xnoalias.hpp"

namespace mppi::critics
{

void AccelCritic::initialize()
{
  auto node = parent_.lock();

  node->declare_parameter(name_ + ".cost_power", 1);
  node->declare_parameter(name_ + ".cost_weight", 5.0f);
  // [2026-08-23 신규] our_mppi_params.yaml에 예전부터 있던 최상위 ax_max/
  // ax_min은 이 벤더링된 nav2_mppi_controller 버전에서 실제로 아무 데도
  // 안 쓰이는 죽은 파라미터였다(확인: Optimizer::applyControlSequenceConstraints
  // 은 vx를 [vx_min,vx_max] 범위로 크기만 클램프하고, DiffDriveMotionModel::
  // applyConstraints는 빈 구현이라 가속도 제한이 어디에도 없음 --
  // AckermannMotionModel만 최소회전반경 체크가 있고 이것도 가속도와 무관).
  // 그래서 직진 구간처럼 다른 critic이 속도를 안 말리는 상황에서, vx_std
  // (0.3, 후보 속도 데드락 수정용으로 상향)가 넓어진 만큼 후보 속도가
  // 순간적으로 크게 튈 수 있었음("급발진"). 이 critic이 그 빠진 자리를
  // 실제로 채운다.
  node->declare_parameter(name_ + ".ax_max", 0.5f);

  power_ = node->get_parameter(name_ + ".cost_power").as_int();
  weight_ = node->get_parameter(name_ + ".cost_weight").as_double();
  ax_max_ = node->get_parameter(name_ + ".ax_max").as_double();

  RCLCPP_INFO(
    logger_, "AccelCritic instantiated with %d power, %f weight, ax_max=%f m/s^2",
    power_, weight_, ax_max_);
}

void AccelCritic::score(CriticData & data)
{
  if (!enabled_) {return;}

  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);
  if (time_steps < 2) {return;}

  // data.state.vx(:,0)은 옵티마이저가 매 사이클 로봇의 실제 현재 속도로
  // 강제 대입해두는 값이다(optimizer.cpp의 updateStateVelocities, 이
  // score() 호출 전에 이미 끝나 있음) -- 그래서 여기서 그냥 연속 시간축
  // 차분(t와 t-1)만 계산해도 "실제 현재 속도 -> 후보가 처음 계획한 속도"
  // 변화량까지 자연히 포함된다. 후보 자신이 계획한 값끼리 비교하는 순수
  // 계산일 뿐 실측 IMU/가속도계 값이 전혀 아니므로 센서 노이즈 문제와
  // 무관하다.
  //
  // [2026-08-23 수정] 가속(dvx>0)만 벌점, 감속(dvx<0)은 완전히 자유롭게 둠.
  // 원래 |dvx| 대칭으로 벌점을 매겼더니 weight를 올리자(5->15) 경로 추종성이
  // 크게 나빠짐 -- 커브에서 PathAlignCritic이 필요로 하는 "빠른 감속"까지
  // 같이 억눌러버린 게 원인으로 추정됨. 실제로 잡고 싶었던 증상("급발진")은
  // 갑자기 빨라지는 것이지 갑자기 느려지는 게 아니고, 감속은 대체로
  // 안전한/필요한 동작이라 제한할 이유가 없음 -- 그래서 가속 방향만 비대칭
  // 적으로 제한.
  const float max_step = ax_max_ * static_cast<float>(data.model_dt);

  xt::xtensor<float, 1> cost = xt::zeros<float>({batch_size});
  for (size_t t = 0; t < batch_size; ++t) {
    float acc = 0.0f;
    for (size_t p = 1; p < time_steps; ++p) {
      const float dvx = data.state.vx(t, p) - data.state.vx(t, p - 1);
      const float accel_only = std::max(0.0f, dvx);  // 감속(dvx<0)은 0으로 무시
      acc += std::max(0.0f, accel_only - max_step);
    }
    cost(t) = (acc / static_cast<float>(time_steps - 1)) * weight_;
  }

  // doldrive_ws fix (2026-08-10, SkidCritic/SlopeCritic과 동일 패턴): plain
  // `+=`으로 합치면 32바이트 정렬 SIGSEGV가 재현될 수 있어 noalias로
  // 제자리에 더한다.
  if (power_ > 1u) {
    xt::noalias(data.costs) += xt::pow(cost, static_cast<float>(power_));
  } else {
    xt::noalias(data.costs) += cost;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::AccelCritic,
  mppi::critics::CriticFunction)
