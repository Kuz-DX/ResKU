#include <gtest/gtest.h>
#include <cmath>
#include "reduced_odom/reduced_ekf.hpp"

using reduced_odom::ReducedEkf;
static ReducedEkf::V5 qzero() {return ReducedEkf::V5::Zero();}

TEST(ReducedEkf, StraightAndStop)
{
  ReducedEkf f; ReducedEkf::V5 q; q << 0.01, 0.02, 0.01, 0.10, 0.20;
  for (int i = 0; i < 200; ++i) {
    ASSERT_TRUE(f.correct(3, 1.0, 0.02, 0)); f.predict(0.01, q);
  }
  EXPECT_NEAR(f.state()(0), 2.0, 0.08); EXPECT_NEAR(f.state()(1), 0, 1e-6);
  for (int i = 0; i < 100; ++i) {ASSERT_TRUE(f.correct(3, 0.0, 0.02, 0)); f.predict(0.01, q);}
  const double stopped_x = f.state()(0);
  EXPECT_NEAR(f.state()(3), 0.0, 1e-3);
  f.predict(0.1, q); EXPECT_NEAR(f.state()(0), stopped_x, 2e-3);
}

TEST(ReducedEkf, ConstantRadiusLeftAndRight)
{
  for (double sign : {1.0, -1.0}) {
    ReducedEkf f; f.correct(3, 1.0, 1e-9, 0); f.correct(4, sign, 1e-9, 0);
    for (int i = 0; i < 100; ++i) {f.predict(0.01, qzero());}
    EXPECT_NEAR(f.state()(2), sign, 1e-3); EXPECT_GT(f.state()(0), 0.8); EXPECT_GT(sign * f.state()(1), 0.4);
  }
}

TEST(ReducedEkf, YawWrapAndInnovationGate)
{
  ReducedEkf f; EXPECT_TRUE(f.correct(2, M_PI - 0.01, 1e-6, 0, true));
  EXPECT_TRUE(f.correct(2, -M_PI + 0.01, 1e-6, 0, true));
  EXPECT_NEAR(std::abs(f.state()(2)), M_PI, 0.02);
  EXPECT_FALSE(f.correct(2, 0.0, 1e-6, 3.0, true));
}

TEST(ReducedEkf, WheelMeasurementsAndCovarianceStayFinite)
{
  ReducedEkf f; auto q = ReducedEkf::V5::Constant(0.01);
  for (int i = 0; i < 1000; ++i) {
    f.predict(0.02, q); EXPECT_TRUE(f.correct(3, 0.4, 0.02, 5)); EXPECT_TRUE(f.correct(4, 0.2, 0.1, 5));
  }
  EXPECT_TRUE(f.state().allFinite()); EXPECT_TRUE(f.covariance().allFinite());
  EXPECT_GT(f.covariance().diagonal().minCoeff(), 0.0);
}
