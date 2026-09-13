#include "our_mppi_critics/skid_critic.hpp"
#include <cmath>
#include "xtensor/xarray.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xnoalias.hpp"

namespace mppi::critics
{

void SkidCritic::initialize()
{
  auto node = parent_.lock();

  node->declare_parameter(name_ + ".cost_power", 1);
  node->declare_parameter(name_ + ".cost_weight", 10.0f);
  node->declare_parameter(name_ + ".track_width", 0.5f);

  power_       = node->get_parameter(name_ + ".cost_power").as_int();
  weight_      = node->get_parameter(name_ + ".cost_weight").as_double();
  track_width_ = node->get_parameter(name_ + ".track_width").as_double();

  RCLCPP_INFO(logger_, "SkidCritic instantiated with %d power and %f weight", power_, weight_);
}

void SkidCritic::score(CriticData & data)
{
  if (!enabled_) {return;}

  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);

  xt::xtensor<float, 1> cost = xt::zeros<float>({batch_size});

  for (size_t t = 0; t < batch_size; t++) {
    float total_slip = 0.0f;
    for (size_t p = 0; p < time_steps; p++) {
      const float vx = data.state.vx(t, p);
      const float wz = data.state.wz(t, p);
      const float v_right = vx + wz * track_width_ / 2.0f;
      const float v_left  = vx - wz * track_width_ / 2.0f;
      total_slip += std::abs(v_right - v_left);
    }
    cost(t) = total_slip / static_cast<float>(time_steps);
  }

  // doldrive_ws fix (2026-08-10): plain `data.costs += ...` reallocates
  // costs_ (confirmed via instrumented mppi_repro_harness runs: every
  // critic's `+=` reallocates costs_ to a new address, but only this one
  // -- the only critic that first materializes a same-size, separately
  // heap-allocated named xt::xtensor `cost` and combines it with a scalar
  // multiply -- produced a 16-byte-aligned result instead of the required
  // 32-byte alignment for xsimd::aligned_allocator<float,32>, causing an
  // AVX256 SIMD store SIGSEGV later in Optimizer::updateControlSequence().
  // xt::noalias() (same pattern already used in optimizer.cpp's
  // updateControlSequence()) tells xtensor there's no aliasing between LHS
  // and RHS, so it adds in place into costs_'s existing buffer instead of
  // reallocating -- this avoids the reallocation entirely rather than
  // relying on it landing on an aligned address. The exact xtensor-internal
  // reason the reallocating path picked a misaligned buffer for this one
  // case was not fully traced; this fix is confirmed empirically (repro
  // harness: crash before, no crash after, alignment instrumentation shows
  // costs_ staying 32-byte-aligned through this critic every run).
  if (power_ > 1u) {
    xt::noalias(data.costs) += xt::pow(cost * weight_, static_cast<float>(power_));
  } else {
    xt::noalias(data.costs) += cost * weight_;
  }
}


}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::SkidCritic,
  mppi::critics::CriticFunction)