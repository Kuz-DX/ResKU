/**
 * \file conversions.hpp
 * \mainpage
 *    Contains functions for converting physical units
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef RMD_HARDWARE_INTERFACE__CONVERSIONS
#define RMD_HARDWARE_INTERFACE__CONVERSIONS
#pragma once

#include <algorithm>
#include <cmath>


namespace rmd_hardware_interface {

  /**\fn degToRad
   * \brief
   *    Convert an angle in degree to radians
   * 
   * \param[in] degrees
   *    The angle in degree
   * \return
   *    The angle in radians
  */
  constexpr double degToRad(double const degrees) noexcept {
    return degrees*(M_PI/180.0);
  }

  /**\fn radToDeg
   * \brief
   *    Convert an angle in radians to degrees
   * 
   * \param[in] radians
   *    The angle in radians
   * \return
   *    The angle in degree
  */
  constexpr double radToDeg(double const radians) noexcept {
    return radians*(180.0/M_PI);
  }

  /**\fn currentToTorque
   * \brief
   *    Convert a given current to a torque by assuming a linear relationship
   * 
   * \param[in] current
   *    The current in Ampere (A) that should be converted to a equivalent torque
   * \param[in] torque_constant
   *    The torque constant in Newton meter per Ampere (Nm/A)
   * \return
   *    The resulting torque in Newton meter (Nm)
  */
  constexpr double currentToTorque(double const current, double const torque_constant) noexcept {
    return current*torque_constant;
  }

  /**\fn torqueToCurrent
   * \brief
   *    Convert a given torque to a current by assuming a linear relationship
   * 
   * \param[in] torque
   *    The torque in Newton meter (Nm) that should be converted to a equivalent current
   * \param[in] torque_constant
   *    The torque constant in Newton meter per Ampere (Nm/A)
   * \return
   *    The resulting current in Ampere (A)
  */
  constexpr double torqueToCurrent(double const torque, double const torque_constant) noexcept {
    return torque/torque_constant;
  }

  inline double positionJumpThreshold(double const velocity_limit_rad_s,
      double const sample_dt_s, double const nominal_dt_s, double const margin_factor) noexcept {
    if (!std::isfinite(velocity_limit_rad_s) || !std::isfinite(sample_dt_s) ||
        !std::isfinite(nominal_dt_s) || !std::isfinite(margin_factor) ||
        velocity_limit_rad_s <= 0.0 || sample_dt_s <= 0.0 || nominal_dt_s <= 0.0 ||
        margin_factor <= 0.0) {
      return NAN;
    }
    // A late CAN response legitimately covers more travel than a nominal
    // sample. An early response must not make the bound tighter than one cycle.
    return velocity_limit_rad_s * std::max(sample_dt_s, nominal_dt_s) * margin_factor;
  }

  inline bool directionMismatch(double const command_delta, double const feedback_delta,
      double const command_deadband, double const feedback_deadband) noexcept {
    return std::isfinite(command_delta) && std::isfinite(feedback_delta) &&
      std::abs(command_delta) >= command_deadband &&
      std::abs(feedback_delta) >= feedback_deadband &&
      command_delta * feedback_delta < 0.0;
  }

  inline bool movesAwayFromTarget(double initial_error, double final_error,
      double feedback_delta) noexcept {
    return std::isfinite(initial_error) && std::isfinite(final_error) &&
      std::isfinite(feedback_delta) && initial_error * feedback_delta < 0.0 &&
      final_error * feedback_delta < 0.0;
  }

}  // namespace rmd_hardware_interface

#endif  // RMD_HARDWARE_INTERFACE__CONVERSIONS
