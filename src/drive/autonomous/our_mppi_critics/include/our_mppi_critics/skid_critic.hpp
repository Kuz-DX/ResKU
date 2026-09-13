#ifndef OUR_MPPI_CRITICS__SKID_CRITIC_HPP_
#define OUR_MPPI_CRITICS__SKID_CRITIC_HPP_

#include <string>
#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi::critics
{

class SkidCritic : public mppi::critics::CriticFunction
{
public:
  SkidCritic() = default;
  void initialize() override;
  void score(mppi::CriticData & data) override;

private:
  float track_width_{0.5f};
  unsigned int power_{1u};
  float weight_{10.0f};
};

}  // namespace our_mppi_critics

#endif  // OUR_MPPI_CRITICS__SKID_CRITIC_HPP_