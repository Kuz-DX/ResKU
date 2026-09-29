#pragma once

#include <cmath>
#include <cstddef>
#include <deque>

#include "reduced_odom/reduced_ekf.hpp"

namespace reduced_odom
{
// 0/360(=-pi/pi) 경계에서도 올바른 각도 평균. 359,0,1deg -> 0deg (산술평균 120deg 아님).
template<typename Container>
double circularMean(const Container & angles)
{
  double s = 0.0, c = 0.0;
  for (double a : angles) {s += std::sin(a); c += std::cos(a);}
  return std::atan2(s, c);
}

// [2026-09-29] myAHRS+ gyro + AHRS yaw 기반 heading 추적. 모든 각도는 rad,
// base_link 기준 ROS 관례(위에서 봤을 때 CCW = +).
//
//   yaw_raw             : 센서 AHRS가 준 현재 절대 yaw (base_link 기준으로 변환된 값)
//   yaw_zero            : 시작(또는 reset) 시 정지 구간 circular mean
//   yaw_relative        : wrap(yaw_raw - yaw_zero), [-pi, pi]
//   yaw_unwrapped       : sum(wrap(yaw_raw[k] - yaw_raw[k-1])), 여러 바퀴 누적
//   yaw_gyro_integrated : sum((wz_raw - bias) * dt), 사다리꼴 적분
//
// gyro bias와 yaw_zero는 별개다: resetZero()는 bias를 건드리지 않는다.
class ImuHeading
{
public:
  struct Config
  {
    size_t zero_samples{50};           // 시작 정지 구간 샘플 수 (yaw_zero + 초기 bias)
    double stationary_gyro_max{0.05};  // [rad/s] 정지 판정 시 창 평균 대비 허용 편차
    double stationary_yaw_max{0.035};  // [rad] 정지 판정 시 창 평균 대비 허용 yaw 편차(~2deg)
    double bias_alpha{0.01};           // ZUPT bias EMA 계수 (20Hz에서 시상수 ~5s)
    double max_dt{0.25};               // [s] 이보다 긴 gap은 적분하지 않음
    size_t reset_window{10};           // resetZero()가 평균낼 최근 yaw 샘플 수
  };

  enum class CalibStatus {kCollecting, kRestarted, kDone};

  ImuHeading() = default;
  explicit ImuHeading(const Config & cfg) : cfg_(cfg) {}

  // 정지 상태라고 판단된 샘플만 넣는다. 창 안에서 움직임이 감지되면
  // 창을 이 샘플부터 다시 시작한다(kRestarted).
  CalibStatus addCalibrationSample(double t, double yaw_abs, double wz_raw)
  {
    if (!calib_yaw_.empty()) {
      const double mean_wz = calib_wz_sum_ / static_cast<double>(calib_yaw_.size());
      if (std::fabs(wz_raw - mean_wz) > cfg_.stationary_gyro_max ||
        std::fabs(wrap(yaw_abs - circularMean(calib_yaw_))) > cfg_.stationary_yaw_max)
      {
        restartCalibration();
        pushCalib(yaw_abs, wz_raw);
        return CalibStatus::kRestarted;
      }
    }
    pushCalib(yaw_abs, wz_raw);
    if (calib_yaw_.size() < cfg_.zero_samples) {return CalibStatus::kCollecting;}

    bias_ = calib_wz_sum_ / static_cast<double>(calib_yaw_.size());
    yaw_zero_ = circularMean(calib_yaw_);
    calibrated_ = true;
    startTracking(t, yaw_abs, wz_raw);
    calib_yaw_.clear(); calib_wz_sum_ = 0.0;
    return CalibStatus::kDone;
  }

  void restartCalibration() {calib_yaw_.clear(); calib_wz_sum_ = 0.0;}
  size_t calibrationCount() const {return calib_yaw_.size();}
  bool calibrated() const {return calibrated_;}

  // 보정 완료 후 매 IMU 샘플마다 호출. t는 단조 증가해야 한다.
  void update(double t, double yaw_abs, double wz_raw)
  {
    const double wz_corr = wz_raw - bias_;
    const double dt = t - prev_t_;
    if (dt > 0.0) {
      if (dt <= cfg_.max_dt) {yaw_gyro_ += 0.5 * (wz_corr + prev_wz_corr_) * dt;}
      prev_t_ = t;
    }
    // AHRS yaw 누적은 시간과 무관하게 샘플 간 최소 각도 차이로.
    yaw_unwrapped_ += wrap(yaw_abs - yaw_raw_);
    yaw_raw_ = yaw_abs;
    wz_raw_ = wz_raw;
    wz_corr_ = wz_corr;
    prev_wz_corr_ = wz_corr;
    pushRecent(yaw_abs);
  }

  // ZUPT: 호출자가 "확실히 정지"라고 판단했을 때만 호출. 큰 편차는 무시.
  bool updateBias(double wz_raw)
  {
    if (std::fabs(wz_raw - bias_) > cfg_.stationary_gyro_max) {return false;}
    bias_ += cfg_.bias_alpha * (wz_raw - bias_);
    return true;
  }

  // 현재(최근 reset_window 샘플 circular mean) 방향을 새 yaw_zero로. bias 유지.
  void resetZero()
  {
    if (!calibrated_) {return;}
    yaw_zero_ = recent_.empty() ? yaw_raw_ : circularMean(recent_);
    yaw_unwrapped_ = 0.0;
    yaw_gyro_ = 0.0;
  }

  double yawRaw() const {return yaw_raw_;}
  double yawZero() const {return yaw_zero_;}
  double yawRelative() const {return wrap(yaw_raw_ - yaw_zero_);}
  double yawUnwrapped() const {return yaw_unwrapped_;}
  double yawGyroIntegrated() const {return yaw_gyro_;}
  double wzRaw() const {return wz_raw_;}
  double wzCorrected() const {return wz_corr_;}
  double biasZ() const {return bias_;}

private:
  void pushCalib(double yaw_abs, double wz_raw)
  {calib_yaw_.push_back(yaw_abs); calib_wz_sum_ += wz_raw;}

  void pushRecent(double yaw_abs)
  {
    recent_.push_back(yaw_abs);
    while (recent_.size() > cfg_.reset_window) {recent_.pop_front();}
  }

  void startTracking(double t, double yaw_abs, double wz_raw)
  {
    prev_t_ = t;
    yaw_raw_ = yaw_abs;
    wz_raw_ = wz_raw;
    wz_corr_ = prev_wz_corr_ = wz_raw - bias_;
    yaw_unwrapped_ = 0.0;
    yaw_gyro_ = 0.0;
    recent_.clear();
    pushRecent(yaw_abs);
  }

  Config cfg_;
  std::deque<double> calib_yaw_, recent_;
  double calib_wz_sum_{0.0};
  bool calibrated_{false};
  double bias_{0.0}, yaw_zero_{0.0}, yaw_raw_{0.0};
  double yaw_unwrapped_{0.0}, yaw_gyro_{0.0};
  double wz_raw_{0.0}, wz_corr_{0.0}, prev_wz_corr_{0.0}, prev_t_{0.0};
};
}  // namespace reduced_odom
