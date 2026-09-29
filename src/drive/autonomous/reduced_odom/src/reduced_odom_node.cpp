#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "reduced_odom/imu_heading.hpp"
#include "reduced_odom/reduced_ekf.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

class ReducedOdomNode : public rclcpp::Node
{
public:
  ReducedOdomNode()
  : Node("reduced_odom_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    wheel_topic_ = declare_parameter("wheel_topic", std::string("/wheel/odom"));
    imu_topic_ = declare_parameter("imu_topic", std::string("/imu"));
    output_topic_ = declare_parameter("output_topic", std::string("/odometry/filtered"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    timeout_ = declare_parameter("sensor_timeout", 0.3);
    max_dt_ = declare_parameter("max_dt", 0.25);
    q_ << declare_parameter("q_x", 0.01), declare_parameter("q_y", 0.02),
      declare_parameter("q_yaw", 0.01), declare_parameter("q_vx", 0.10),
      declare_parameter("q_wz", 0.20);
    r_vx_ = declare_parameter("r_wheel_vx", 0.02);
    r_wz_ = declare_parameter("r_wheel_wz", 0.10);
    r_yaw_ = declare_parameter("r_imu_yaw", 0.01);
    wheel_gate_ = declare_parameter("wheel_gate_sigma", 5.0);
    yaw_gate_ = declare_parameter("yaw_gate_sigma", 4.0);
    // false: ignore the IMU's absolute yaw entirely and integrate yaw from
    // the wheel yaw rate only (roll/pitch still come from the IMU when it is
    // present). Used by the manual+return mission, where the AHRS yaw was
    // measured to keep sliding at ~2 deg/s for minutes after the motors ran.
    // Default true keeps the original wheel+IMU behaviour for every other user.
    use_imu_yaw_ = declare_parameter("use_imu_yaw", true);
    // [2026-09-29] true: myAHRS+ gyro(angular_velocity, myahrs_driver
    // ascii_format:=RPYIMU 필요)를 wz 측정으로 융합하고, 시작 정지 구간에서
    // gyro bias와 yaw_zero를 잡아 odom yaw를 0에서 시작한다. 이 모드에서
    // use_imu_yaw=true면 AHRS yaw_relative(= yaw_raw - yaw_zero)를 장기 yaw
    // 보정으로 추가 융합한다. false(기본)면 아래 기존 경로가 그대로 동작한다.
    use_imu_gyro_ = declare_parameter("use_imu_gyro", false);
    r_gyro_wz_ = declare_parameter("r_gyro_wz", 0.001);
    gyro_gate_ = declare_parameter("gyro_gate_sigma", 5.0);
    r_yaw_rel_ = declare_parameter("r_imu_yaw_relative", 0.05);
    calib_requires_wheel_ = declare_parameter("calibration_requires_wheel", true);
    stationary_vx_ = declare_parameter("stationary_vx", 0.005);
    stationary_wz_ = declare_parameter("stationary_wz", 0.01);
    zupt_min_time_ = declare_parameter("zupt_min_time", 1.0);
    {
      reduced_odom::ImuHeading::Config c;
      c.zero_samples = static_cast<size_t>(
        std::max<int64_t>(1, declare_parameter("yaw_zero_samples", 50)));
      c.stationary_gyro_max = declare_parameter("stationary_gyro_max", 0.05);
      c.bias_alpha = declare_parameter("gyro_bias_alpha", 0.01);
      c.max_dt = max_dt_;
      heading_ = reduced_odom::ImuHeading(c);
    }

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(20);
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic_, sensor_qos, std::bind(&ReducedOdomNode::wheelCb, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos, std::bind(&ReducedOdomNode::imuCb, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    diag_pub_ = create_publisher<DiagnosticArray>("/odometry/diagnostics", 10);
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    diag_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ReducedOdomNode::diagnostics, this));
    if (use_imu_gyro_) {
      // 디버그 토픽(모두 std_msgs/Float64, rad 또는 rad/s, base_link 기준 CCW=+).
      for (const char * name : {"yaw_raw", "yaw_zero", "yaw_relative", "yaw_unwrapped",
          "yaw_gyro_integrated", "gyro/z_raw", "gyro/z_corrected", "gyro/bias_z"})
      {
        debug_pubs_[name] = create_publisher<std_msgs::msg::Float64>(std::string("/imu/") + name, 10);
      }
      reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "/reset_yaw_zero", std::bind(&ReducedOdomNode::resetYawZero, this,
        std::placeholders::_1, std::placeholders::_2));
      RCLCPP_INFO(get_logger(),
        "use_imu_gyro=true: waiting for %ld stationary IMU samples to estimate gyro bias and yaw_zero",
        get_parameter("yaw_zero_samples").as_int());
    } else if (!use_imu_yaw_) {
      // No IMU is needed to start publishing in this mode; yaw starts at 0
      // (callers work in a mission frame anchored at their own T0).
      ekf_.initializeYaw(0.0, r_yaw_);
      have_attitude_ = true;
      RCLCPP_WARN(get_logger(), "use_imu_yaw=false: yaw is integrated from wheel odometry only");
    }
    RCLCPP_INFO(get_logger(), "5-state estimator: %s + %s -> %s and %s->%s TF",
      wheel_topic_.c_str(), imu_topic_.c_str(), output_topic_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  static double stampSec(const builtin_interfaces::msg::Time & t)
  {return static_cast<double>(t.sec) + 1e-9 * t.nanosec;}

  bool imuToBase(
    const sensor_msgs::msg::Imu & msg, double & roll, double & pitch, double & yaw,
    tf2::Quaternion * q_base_imu_out = nullptr)
  {
    tf2::Quaternion q_world_imu;
    tf2::fromMsg(msg.orientation, q_world_imu);
    if (!std::isfinite(q_world_imu.length2()) || q_world_imu.length2() < 1e-6) {return false;}
    q_world_imu.normalize();
    try {
      const auto t = tf_buffer_.lookupTransform(base_frame_, msg.header.frame_id, tf2::TimePointZero);
      tf2::Quaternion q_base_imu;
      tf2::fromMsg(t.transform.rotation, q_base_imu);
      q_base_imu.normalize();
      const tf2::Quaternion q_world_base = q_world_imu * q_base_imu.inverse();
      tf2::Matrix3x3(q_world_base).getRPY(roll, pitch, yaw);
      if (q_base_imu_out) {*q_base_imu_out = q_base_imu;}
      return std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU mount TF unavailable: %s", e.what());
      return false;
    }
  }

  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    ++imu_count_;
    const double t = stampSec(msg->header.stamp);
    if (last_imu_stamp_ > 0.0 && t <= last_imu_stamp_) {++timestamp_rejects_; return;}
    double roll, pitch, yaw;
    tf2::Quaternion q_base_imu;
    if (!imuToBase(*msg, roll, pitch, yaw, &q_base_imu)) {++imu_rejects_; return;}
    last_imu_stamp_ = t;
    if (use_imu_gyro_) {
      roll_ = roll; pitch_ = pitch;
      imuGyroUpdate(*msg, t, yaw, q_base_imu);
      return;
    }
    if (!use_imu_yaw_) {
      roll_ = roll; pitch_ = pitch;
      return;
    }
    if (!have_attitude_) {
      // AHRS heading has an arbitrary valid initial angle. It is not an
      // innovation relative to zero, so initialize rather than gate it.
      ekf_.initializeYaw(yaw, r_yaw_);
      last_yaw_innov_ = 0.0; last_yaw_s_ = r_yaw_;
    } else if (!ekf_.correct(2, yaw, r_yaw_, yaw_gate_, true, &last_yaw_innov_, &last_yaw_s_)) {
      ++yaw_gate_rejects_; return;
    }
    roll_ = roll; pitch_ = pitch; have_attitude_ = true;
  }

  void wheelCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    ++wheel_count_;
    const double t = stampSec(msg->header.stamp);
    if (last_wheel_stamp_ <= 0.0) {last_wheel_stamp_ = t; return;}
    const double dt = t - last_wheel_stamp_;
    last_wheel_stamp_ = t;
    if (!(dt > 0.0) || dt > max_dt_) {++timestamp_rejects_; return;}
    const double vx = msg->twist.twist.linear.x;
    const double wz = msg->twist.twist.angular.z;
    if (!std::isfinite(vx) || !std::isfinite(wz)) {++wheel_rejects_; return;}
    if (std::fabs(vx) < stationary_vx_ && std::fabs(wz) < stationary_wz_) {
      if (wheel_still_since_ < 0.0) {wheel_still_since_ = t;}
    } else {
      wheel_still_since_ = -1.0;
    }
    ekf_.predict(dt, q_);
    if (!ekf_.correct(3, vx, r_vx_, wheel_gate_, false, &last_vx_innov_, &last_vx_s_)) {
      ++wheel_rejects_; ++wheel_vx_rejects_;
    }
    // [2026-09-30] gyro가 살아 있으면 휠 wz는 아예 넣지 않는다(스키드 슬립 차단).
    // gyro가 sensor_timeout 이상 끊기면 휠 wz로 자동 전환 -- EKF wz 상태는
    // 측정이 없으면 마지막 값을 유지하므로, 전환 없이 두면 끊기기 직전 회전
    // 속도로 yaw가 계속 돌아간다.
    const bool gyro_ok = gyroHealthy(t);
    updateWzSource(gyro_ok);
    if (!gyro_ok &&
      !ekf_.correct(4, wz, r_wz_, wheel_gate_, false, &last_wz_innov_, &last_wz_s_))
    {
      ++wheel_rejects_; ++wheel_wz_rejects_;
    }
    if (have_attitude_) {publish(msg->header.stamp);}
  }

  bool gyroHealthy(double t) const
  {
    return use_imu_gyro_ && heading_.calibrated() && last_gyro_stamp_ > 0.0 &&
           t - last_gyro_stamp_ <= timeout_;
  }

  // 보정 완료 이후의 gyro<->wheel 전환만 기록한다(보정 전 정지 구간의 휠 사용은 정상).
  void updateWzSource(bool gyro_ok)
  {
    if (!use_imu_gyro_ || !heading_.calibrated()) {return;}
    if (!gyro_ok && !gyro_fallback_) {
      gyro_fallback_ = true;
      ++gyro_fallbacks_;
      // 끊기기 직전 gyro wz에 과신하지 않도록 -> 다음 휠 측정이 wz를 바로 잡는다.
      ekf_.inflateVariance(4, 1.0);
      RCLCPP_WARN(get_logger(),
        "gyro stale (>%.2fs): yaw rate falls back to WHEEL wz (skid slip applies)", timeout_);
    } else if (gyro_ok && gyro_fallback_) {
      gyro_fallback_ = false;
      RCLCPP_INFO(get_logger(), "gyro recovered: yaw rate back to GYRO only");
    }
  }

  void publish(const builtin_interfaces::msg::Time & stamp)
  {
    const auto & x = ekf_.state();
    tf2::Quaternion q; q.setRPY(roll_, pitch_, x(2)); q.normalize();
    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp; out.header.frame_id = odom_frame_; out.child_frame_id = base_frame_;
    out.pose.pose.position.x = x(0); out.pose.pose.position.y = x(1);
    out.pose.pose.orientation = tf2::toMsg(q);
    // Odometry twist is expressed in child_frame_id (base_link), not odom.
    out.twist.twist.linear.x = x(3); out.twist.twist.angular.z = x(4);
    const auto & P = ekf_.covariance();
    out.pose.covariance[0] = P(0, 0); out.pose.covariance[1] = P(0, 1);
    out.pose.covariance[6] = P(1, 0); out.pose.covariance[7] = P(1, 1);
    out.pose.covariance[35] = P(2, 2);
    out.twist.covariance[0] = P(3, 3); out.twist.covariance[35] = P(4, 4);
    odom_pub_->publish(out);
    geometry_msgs::msg::TransformStamped tf;
    tf.header = out.header; tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x(0); tf.transform.translation.y = x(1);
    tf.transform.rotation = out.pose.pose.orientation; tf_pub_->sendTransform(tf);
  }

  // [2026-09-29] use_imu_gyro 경로. yaw_abs는 imuToBase가 준 base_link 기준 절대 yaw.
  void imuGyroUpdate(
    const sensor_msgs::msg::Imu & msg, double t, double yaw_abs, const tf2::Quaternion & q_base_imu)
  {
    if (msg.angular_velocity_covariance[0] < 0.0) {
      ++gyro_missing_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "use_imu_gyro=true but /imu has no angular_velocity (covariance[0]=-1). "
        "Start myahrs_driver with ascii_format:=RPYIMU.");
      return;
    }
    // imu_link 각속도 벡터를 마운트 회전으로 base_link에 옮긴 뒤 z 성분.
    const tf2::Vector3 w_imu(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    const double wz_raw = tf2::quatRotate(q_base_imu, w_imu).z();
    if (!std::isfinite(wz_raw)) {++gyro_missing_; return;}

    if (!heading_.calibrated()) {
      if (!stationaryForCalibration(t)) {
        heading_.restartCalibration();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "gyro/yaw_zero calibration waiting: robot must be stationary (wheel %s)",
          wheelFresh(t) ? "moving" : "odom missing");
        return;
      }
      const auto st = heading_.addCalibrationSample(t, yaw_abs, wz_raw);
      if (st == reduced_odom::ImuHeading::CalibStatus::kRestarted) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "gyro/yaw_zero calibration restarted: IMU motion detected");
      }
      if (st != reduced_odom::ImuHeading::CalibStatus::kDone) {return;}
      // odom은 시작 방향을 yaw=0으로.
      ekf_.initializeYaw(0.0, r_yaw_rel_);
      have_attitude_ = true;
      last_gyro_stamp_ = t;
      RCLCPP_INFO(get_logger(),
        "gyro calibrated: bias_z=%.5f rad/s (%.3f deg/s), yaw_zero=%.3f deg -> odom yaw starts at 0",
        heading_.biasZ(), heading_.biasZ() * 180.0 / M_PI, heading_.yawZero() * 180.0 / M_PI);
      publishHeadingDebug();
      return;
    }

    heading_.update(t, yaw_abs, wz_raw);
    last_gyro_stamp_ = t;
    if (wheelFresh(t) && wheel_still_since_ >= 0.0 && t - wheel_still_since_ >= zupt_min_time_ &&
      heading_.updateBias(wz_raw))
    {
      ++zupt_updates_;
    }
    if (!ekf_.correct(4, heading_.wzCorrected(), r_gyro_wz_, gyro_gate_, false,
        &last_gyro_innov_, &last_gyro_s_))
    {
      ++gyro_rejects_;
    }
    if (use_imu_yaw_ &&
      !ekf_.correct(2, heading_.yawRelative(), r_yaw_rel_, yaw_gate_, true,
        &last_yaw_innov_, &last_yaw_s_))
    {
      ++yaw_gate_rejects_;
    }
    publishHeadingDebug();
  }

  bool wheelFresh(double t) const
  {return last_wheel_stamp_ > 0.0 && std::fabs(t - last_wheel_stamp_) <= timeout_;}

  bool stationaryForCalibration(double t) const
  {
    if (wheelFresh(t)) {return wheel_still_since_ >= 0.0;}
    return !calib_requires_wheel_;
  }

  void publishHeadingDebug()
  {
    const auto pub = [this](const char * name, double v) {
        std_msgs::msg::Float64 m; m.data = v; debug_pubs_.at(name)->publish(m);
      };
    pub("yaw_raw", heading_.yawRaw()); pub("yaw_zero", heading_.yawZero());
    pub("yaw_relative", heading_.yawRelative()); pub("yaw_unwrapped", heading_.yawUnwrapped());
    pub("yaw_gyro_integrated", heading_.yawGyroIntegrated());
    pub("gyro/z_raw", heading_.wzRaw()); pub("gyro/z_corrected", heading_.wzCorrected());
    pub("gyro/bias_z", heading_.biasZ());
  }

  // 현재 방향을 새 yaw=0으로. gyro bias와 odom x/y는 유지.
  void resetYawZero(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!heading_.calibrated()) {
      res->success = false; res->message = "gyro/yaw_zero calibration not finished yet";
      return;
    }
    heading_.resetZero();
    ekf_.initializeYaw(0.0, r_yaw_rel_);
    publishHeadingDebug();
    res->success = true;
    res->message = "yaw_zero=" + std::to_string(heading_.yawZero() * 180.0 / M_PI) +
      " deg; yaw_relative/unwrapped/gyro_integrated and odom yaw reset to 0 (bias and x/y kept)";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  void diagnostics()
  {
    DiagnosticArray a; a.header.stamp = now(); DiagnosticStatus s;
    s.name = "reduced_odom/estimator"; s.hardware_id = "wheel+myAHRS";
    const double now_s = get_clock()->now().seconds();
    const bool wheel_stale = last_wheel_stamp_ <= 0.0 || now_s - last_wheel_stamp_ > timeout_;
    const bool imu_stale = last_imu_stamp_ <= 0.0 || now_s - last_imu_stamp_ > timeout_;
    const bool imu_missing = (use_imu_yaw_ || use_imu_gyro_) && imu_stale;
    s.level = (wheel_stale || imu_missing) ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
    s.message = wheel_stale ? "wheel stale" : (imu_missing ? "IMU stale" : "OK");
    const auto add = [&s](const std::string & k, auto v) {
        KeyValue item; item.key = k; item.value = std::to_string(v); s.values.push_back(item);
      };
    add("wheel_messages", wheel_count_); add("imu_messages", imu_count_);
    add("wheel_rejected", wheel_rejects_); add("imu_rejected", imu_rejects_);
    add("wheel_vx_rejected", wheel_vx_rejects_); add("wheel_wz_rejected", wheel_wz_rejects_);
    add("yaw_innovation_rejected", yaw_gate_rejects_); add("timestamp_anomalies", timestamp_rejects_);
    add("wheel_stale", wheel_stale ? 1 : 0); add("imu_stale", imu_stale ? 1 : 0);
    add("vx", ekf_.state()(3)); add("wz", ekf_.state()(4)); add("yaw", ekf_.state()(2));
    // [2026-08-24 추가, reduced_odom validation] Q/R/gate 판정 로직은 그대로
    // 두고, 이미 계산돼 있는 값만 노출한다 -- estimator 동작 변경 없음.
    const auto & P = ekf_.covariance();
    add("Pxx", P(0, 0)); add("Pyy", P(1, 1)); add("Pyaw", P(2, 2));
    add("Pvx", P(3, 3)); add("Pwz", P(4, 4));
    add("yaw_innovation", last_yaw_innov_); add("yaw_innovation_S", last_yaw_s_);
    add("wheel_vx_innovation", last_vx_innov_); add("wheel_vx_innovation_S", last_vx_s_);
    add("wheel_wz_innovation", last_wz_innov_); add("wheel_wz_innovation_S", last_wz_s_);
    if (use_imu_gyro_) {
      add("gyro_calibrated", heading_.calibrated() ? 1 : 0);
      add("gyro_calibration_samples", heading_.calibrationCount());
      add("gyro_bias_z", heading_.biasZ()); add("yaw_zero", heading_.yawZero());
      add("gyro_wz_rejected", gyro_rejects_); add("gyro_missing", gyro_missing_);
      add("gyro_wz_innovation", last_gyro_innov_); add("gyro_wz_innovation_S", last_gyro_s_);
      add("zupt_updates", zupt_updates_);
      // wz_source: 1 = gyro only, 0 = wheel (보정 전 또는 gyro 끊김)
      add("wz_source_gyro", gyroHealthy(now_s) ? 1 : 0);
      add("gyro_fallbacks", gyro_fallbacks_);
      if (gyro_fallback_ && s.level == DiagnosticStatus::OK) {
        s.level = DiagnosticStatus::WARN; s.message = "gyro stale: wheel wz fallback";
      }
    }
    a.status.push_back(s); diag_pub_->publish(a);
  }

  reduced_odom::ReducedEkf ekf_;
  reduced_odom::ReducedEkf::V5 q_;
  std::string wheel_topic_, imu_topic_, output_topic_, odom_frame_, base_frame_;
  double timeout_, max_dt_, r_vx_, r_wz_, r_yaw_, wheel_gate_, yaw_gate_;
  double last_wheel_stamp_{-1.0}, last_imu_stamp_{-1.0}, roll_{0.0}, pitch_{0.0};
  bool have_attitude_{false};
  bool use_imu_yaw_{true};
  bool use_imu_gyro_{false}, calib_requires_wheel_{true};
  double r_gyro_wz_{0.001}, gyro_gate_{5.0}, r_yaw_rel_{0.05};
  double stationary_vx_{0.005}, stationary_wz_{0.01}, zupt_min_time_{1.0};
  double wheel_still_since_{-1.0}, last_gyro_stamp_{-1.0};
  bool gyro_fallback_{false};
  uint64_t gyro_fallbacks_{0};
  double last_gyro_innov_{0.0}, last_gyro_s_{0.0};
  uint64_t gyro_rejects_{0}, gyro_missing_{0}, zupt_updates_{0};
  reduced_odom::ImuHeading heading_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> debug_pubs_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  uint64_t wheel_count_{0}, imu_count_{0}, wheel_rejects_{0}, imu_rejects_{0};
  uint64_t yaw_gate_rejects_{0}, timestamp_rejects_{0};
  uint64_t wheel_vx_rejects_{0}, wheel_wz_rejects_{0};
  // [2026-08-24 추가, reduced_odom validation] 진단 전용 -- correct()의
  // out-parameter로 채워지며 accept/reject 판정에는 영향 없음.
  double last_yaw_innov_{0.0}, last_yaw_s_{0.0};
  double last_vx_innov_{0.0}, last_vx_s_{0.0};
  double last_wz_innov_{0.0}, last_wz_s_{0.0};
  tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<ReducedOdomNode>());
  rclcpp::shutdown(); return 0;
}
