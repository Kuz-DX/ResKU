#ifndef OUR_MPPI_CRITICS__ACCEL_CRITIC_HPP_
#define OUR_MPPI_CRITICS__ACCEL_CRITIC_HPP_

#include <string>
#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi::critics
{

class AccelCritic : public mppi::critics::CriticFunction
{
public:
  AccelCritic() = default;
  void initialize() override;
  void score(CriticData & data) override;

private:
  unsigned int power_{1u};
  float weight_{5.0f};
  float ax_max_{0.5f};
};

}  // namespace mppi::critics

#endif  // OUR_MPPI_CRITICS__ACCEL_CRITIC_HPP_
