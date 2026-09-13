#ifndef OUR_MPPI_CRITICS__SLOPE_CRITIC_HPP_
#define OUR_MPPI_CRITICS__SLOPE_CRITIC_HPP_

#include <string>
#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi::critics
{

class SlopeCritic : public mppi::critics::CriticFunction
{
public:
  SlopeCritic() = default;
  void initialize() override;
  void score(CriticData & data) override;

private:
  unsigned int power_{1u};

  // 급경사 제동(steep-pitch brake) 파라미터 -- pitch 기준, path 방향과 무관하게
  // 항상 적용.
  float weight_{5.0f};
  float max_slope_deg_{30.0f};
  float critical_slope_deg_{25.0f};

  // 예측 기반 차동조향(predictive differential-steering) 파라미터 --
  // "path가 실제로 옆으로 휘기 시작할 때만" 개입하도록 turn_gate로 게이팅.
  float predicted_roll_weight_{3.0f};
  float turn_gate_start_deg_{8.0f};
  float turn_gate_full_deg_{20.0f};
  int turn_gate_offset_from_furthest_{4};

  // 위 두 보정 다 이 각도(sqrt(roll^2+pitch^2), deg) 밑에서는 완전히 비활성.
  float min_active_deg_{3.0f};
};

}  // namespace mppi::critics

#endif  // OUR_MPPI_CRITICS__SLOPE_CRITIC_HPP_
