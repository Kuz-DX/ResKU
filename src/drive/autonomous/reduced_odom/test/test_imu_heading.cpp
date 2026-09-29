#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "reduced_odom/imu_heading.hpp"

using reduced_odom::ImuHeading;
using reduced_odom::circularMean;
static double d2r(double d) {return d * M_PI / 180.0;}
static double r2d(double r) {return r * 180.0 / M_PI;}

// 정지 구간 n개로 보정 완료시킨 tracker를 만든다. t는 20Hz.
static ImuHeading calibrated(double yaw0_deg, double bias, double & t, size_t n = 50)
{
  ImuHeading::Config cfg; cfg.zero_samples = n;
  ImuHeading h(cfg);
  for (size_t i = 0; i < n; ++i) {
    t += 0.05;
    const auto st = h.addCalibrationSample(t, d2r(yaw0_deg), bias);
    EXPECT_EQ(st, i + 1 < n ? ImuHeading::CalibStatus::kCollecting : ImuHeading::CalibStatus::kDone);
  }
  return h;
}

TEST(ImuHeading, CircularMeanAcrossWrap)
{
  EXPECT_NEAR(r2d(circularMean(std::vector<double>{d2r(359), d2r(0), d2r(1)})), 0.0, 1e-9);
  EXPECT_NEAR(std::fabs(r2d(circularMean(std::vector<double>{d2r(179), d2r(-179)}))), 180.0, 1e-9);
}

TEST(ImuHeading, StartsAtZeroWithBiasAndYaw0)
{
  double t = 0.0;
  auto h = calibrated(34.11, 0.0244, t);
  EXPECT_TRUE(h.calibrated());
  EXPECT_NEAR(r2d(h.yawZero()), 34.11, 1e-9);
  EXPECT_NEAR(h.biasZ(), 0.0244, 1e-12);
  EXPECT_NEAR(h.yawRelative(), 0.0, 1e-12);
  EXPECT_NEAR(h.yawUnwrapped(), 0.0, 1e-12);
  EXPECT_NEAR(h.yawGyroIntegrated(), 0.0, 1e-12);
}

TEST(ImuHeading, LeftAndRightTenDegrees)
{
  for (double sign : {1.0, -1.0}) {
    double t = 0.0;
    const double bias = 0.0244;
    auto h = calibrated(34.0, bias, t);
    // 1초 동안 10deg/s 로 회전 (sign=+1: CCW)
    const double rate = sign * d2r(10.0);
    for (int i = 1; i <= 20; ++i) {
      t += 0.05;
      h.update(t, d2r(34.0) + rate * 0.05 * i, rate + bias);
    }
    EXPECT_NEAR(r2d(h.yawRelative()), sign * 10.0, 1e-6);
    EXPECT_NEAR(r2d(h.yawUnwrapped()), sign * 10.0, 1e-6);
    // 첫 구간 사다리꼴 적분은 (0 + rate)/2 에서 시작하므로 반 샘플만큼 작다.
    EXPECT_NEAR(r2d(h.yawGyroIntegrated()), sign * 10.0, 0.3);
    EXPECT_NEAR(h.wzCorrected(), rate, 1e-12);
  }
}

TEST(ImuHeading, UnwrapAcrossPi)
{
  double t = 0.0;
  auto h = calibrated(160.0, 0.0, t);
  std::vector<double> seq{170, 179, -179, -170};
  for (double d : seq) {t += 0.05; h.update(t, d2r(d), 0.0);}
  // 160 -> -170 은 +30deg
  EXPECT_NEAR(r2d(h.yawUnwrapped()), 30.0, 1e-6);
  EXPECT_NEAR(r2d(h.yawRelative()), 30.0, 1e-6);
}

TEST(ImuHeading, FullTurnsAccumulate)
{
  for (double sign : {1.0, -1.0}) {
    double t = 0.0;
    auto h = calibrated(34.0, 0.0, t);
    // 두 바퀴: 5deg 스텝 144번
    for (int i = 1; i <= 144; ++i) {
      t += 0.05;
      h.update(t, reduced_odom::wrap(d2r(34.0 + sign * 5.0 * i)), sign * d2r(100.0));
    }
    EXPECT_NEAR(r2d(h.yawUnwrapped()), sign * 720.0, 1e-6);
    EXPECT_NEAR(r2d(h.yawRelative()), 0.0, 1e-6);
  }
}

TEST(ImuHeading, CalibrationRestartsOnMotion)
{
  ImuHeading::Config cfg; cfg.zero_samples = 5;
  ImuHeading h(cfg);
  double t = 0.0;
  for (int i = 0; i < 3; ++i) {h.addCalibrationSample(t += 0.05, 0.0, 0.02);}
  EXPECT_EQ(h.addCalibrationSample(t += 0.05, 0.0, 0.5), ImuHeading::CalibStatus::kRestarted);
  EXPECT_EQ(h.calibrationCount(), 1u);
  EXPECT_FALSE(h.calibrated());
}

TEST(ImuHeading, ResetZeroKeepsBias)
{
  double t = 0.0;
  const double bias = 0.02;
  auto h = calibrated(10.0, bias, t);
  for (int i = 1; i <= 20; ++i) {t += 0.05; h.update(t, d2r(10.0 + i), d2r(20.0) + bias);}
  for (int i = 0; i < 10; ++i) {t += 0.05; h.update(t, d2r(30.0), bias);}
  h.resetZero();
  EXPECT_NEAR(r2d(h.yawZero()), 30.0, 1e-9);
  EXPECT_NEAR(h.yawRelative(), 0.0, 1e-12);
  EXPECT_NEAR(h.yawUnwrapped(), 0.0, 1e-12);
  EXPECT_NEAR(h.yawGyroIntegrated(), 0.0, 1e-12);
  EXPECT_NEAR(h.biasZ(), bias, 1e-12);
}

TEST(ImuHeading, ZuptBiasConvergesAndIgnoresMotion)
{
  double t = 0.0;
  auto h = calibrated(0.0, 0.0, t);
  for (int i = 0; i < 1000; ++i) {h.updateBias(0.03);}
  EXPECT_NEAR(h.biasZ(), 0.03, 1e-4);
  EXPECT_FALSE(h.updateBias(0.5));
  EXPECT_NEAR(h.biasZ(), 0.03, 1e-4);
}

TEST(ImuHeading, LongGapNotIntegrated)
{
  double t = 0.0;
  auto h = calibrated(0.0, 0.0, t);
  t += 1.0;  // max_dt(0.25) 초과
  h.update(t, 0.0, 1.0);
  EXPECT_NEAR(h.yawGyroIntegrated(), 0.0, 1e-12);
}
