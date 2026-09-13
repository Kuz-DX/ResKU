#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace reduced_odom
{
inline double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

// Observable state: [x, y, yaw, body-vx, body-wz]. Roll/pitch are AHRS
// measurements and deliberately do not enter this covariance matrix.
class ReducedEkf
{
public:
  using V5 = Eigen::Matrix<double, 5, 1>;
  using M5 = Eigen::Matrix<double, 5, 5>;

  ReducedEkf() {x_.setZero(); P_.setIdentity(); P_ *= 1e-3;}

  void initializeYaw(double yaw, double variance)
  {x_(2) = wrap(yaw); P_(2, 2) = variance;}

  void predict(double dt, const V5 & q)
  {
    const double yaw = x_(2), vx = x_(3), wz = x_(4);
    x_(0) += vx * std::cos(yaw) * dt;
    x_(1) += vx * std::sin(yaw) * dt;
    x_(2) = wrap(yaw + wz * dt);
    M5 F = M5::Identity();
    F(0, 2) = -vx * std::sin(yaw) * dt; F(0, 3) = std::cos(yaw) * dt;
    F(1, 2) =  vx * std::cos(yaw) * dt; F(1, 3) = std::sin(yaw) * dt;
    F(2, 4) = dt;
    P_ = F * P_ * F.transpose();
    P_.diagonal() += q * dt;
    P_ = 0.5 * (P_ + P_.transpose());
  }

  // out_innovation/out_S: 진단/R-gate 튜닝 전용 관측값(section 7 -- reduced_odom
  // validation 작업). accept/reject 판정이나 게이트 로직에는 전혀 관여하지
  // 않는 순수 out-parameter라 기존 호출부(nullptr 기본값)는 동작이 그대로다.
  // gate에 걸려 reject되는 샘플도 그 innovation/S 값 자체는 튜닝에 유용해서
  // 조기 return 전에 채운다.
  bool correct(
    size_t index, double z, double variance, double gate_sigma, bool angle = false,
    double * out_innovation = nullptr, double * out_S = nullptr)
  {
    double innovation = z - x_(index);
    if (angle) {innovation = wrap(innovation);}
    const double s = P_(index, index) + variance;
    if (out_innovation) {*out_innovation = innovation;}
    if (out_S) {*out_S = s;}
    if (!(s > 0.0) || !std::isfinite(s) ||
      (gate_sigma > 0.0 && innovation * innovation > gate_sigma * gate_sigma * s))
    {return false;}
    const Eigen::Matrix<double, 5, 1> K = P_.col(index) / s;  // scalar SPD solve
    x_ += K * innovation;
    x_(2) = wrap(x_(2));
    M5 I_KH = M5::Identity();
    I_KH.col(index) -= K;
    // Joseph form preserves PSD under finite precision.
    P_ = I_KH * P_ * I_KH.transpose() + variance * K * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    return x_.allFinite() && P_.allFinite();
  }

  const V5 & state() const {return x_;}
  const M5 & covariance() const {return P_;}

private:
  V5 x_;
  M5 P_;
};
}  // namespace reduced_odom
