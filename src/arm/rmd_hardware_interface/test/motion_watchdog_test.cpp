#include <cmath>

#include <gtest/gtest.h>

#include "rmd_hardware_interface/conversions.hpp"

namespace rmd_hardware_interface::test {

TEST(MotionWatchdog, PositionJumpUsesActualOrNominalDtWhicheverIsLarger) {
  EXPECT_NEAR(positionJumpThreshold(0.35, 0.002, 0.002, 4.0), 0.0028, 1e-12);
  EXPECT_NEAR(positionJumpThreshold(0.35, 0.010, 0.002, 4.0), 0.014, 1e-12);
  EXPECT_TRUE(std::isnan(positionJumpThreshold(0.35, 0.0, 0.002, 4.0)));
}

TEST(MotionWatchdog, DirectionMismatchRequiresMotionAboveBothDeadbands) {
  EXPECT_TRUE(directionMismatch(0.002, -0.003, 0.001, 0.001));
  EXPECT_TRUE(directionMismatch(-0.002, 0.003, 0.001, 0.001));
  EXPECT_FALSE(directionMismatch(0.002, 0.003, 0.001, 0.001));
  EXPECT_FALSE(directionMismatch(0.0005, -0.003, 0.001, 0.001));
  EXPECT_FALSE(directionMismatch(0.002, -0.0005, 0.001, 0.001));
}

TEST(MotionWatchdog, TargetApproachDoesNotCountAsReverseMotion) {
  EXPECT_FALSE(movesAwayFromTarget(-0.03, -0.02, -0.00244345));
  EXPECT_TRUE(movesAwayFromTarget(0.03, 0.04, -0.00244345));
  EXPECT_TRUE(movesAwayFromTarget(-0.03, -0.04, 0.00244345));
  EXPECT_FALSE(movesAwayFromTarget(-0.001, 0.001, -0.002));
}

}  // namespace rmd_hardware_interface::test
