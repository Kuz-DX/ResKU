#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
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

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(20);
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic_, sensor_qos, std::bind(&ReducedOdomNode::wheelCb, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos, std::bind(&ReducedOdomNode::imuCb, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    diag_pub_ = create_publisher<DiagnosticArray>("/odometry/diagnostics", 10);
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    diag_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ReducedOdomNode::diagnostics, this));
    RCLCPP_INFO(get_logger(), "5-state estimator: %s + %s -> %s and %s->%s TF",
      wheel_topic_.c_str(), imu_topic_.c_str(), output_topic_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  static double stampSec(const builtin_interfaces::msg::Time & t)
  {return static_cast<double>(t.sec) + 1e-9 * t.nanosec;}

  bool imuToBase(const sensor_msgs::msg::Imu & msg, double & roll, double & pitch, double & yaw)
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
    if (!imuToBase(*msg, roll, pitch, yaw)) {++imu_rejects_; return;}
    last_imu_stamp_ = t;
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
    ekf_.predict(dt, q_);
    if (!ekf_.correct(3, vx, r_vx_, wheel_gate_, false, &last_vx_innov_, &last_vx_s_)) {
      ++wheel_rejects_; ++wheel_vx_rejects_;
    }
    if (!ekf_.correct(4, wz, r_wz_, wheel_gate_, false, &last_wz_innov_, &last_wz_s_)) {
      ++wheel_rejects_; ++wheel_wz_rejects_;
    }
    if (have_attitude_) {publish(msg->header.stamp);}
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

  void diagnostics()
  {
    DiagnosticArray a; a.header.stamp = now(); DiagnosticStatus s;
    s.name = "reduced_odom/estimator"; s.hardware_id = "wheel+myAHRS";
    const double now_s = get_clock()->now().seconds();
    const bool wheel_stale = last_wheel_stamp_ <= 0.0 || now_s - last_wheel_stamp_ > timeout_;
    const bool imu_stale = last_imu_stamp_ <= 0.0 || now_s - last_imu_stamp_ > timeout_;
    s.level = (wheel_stale || imu_stale) ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
    s.message = wheel_stale ? "wheel stale" : (imu_stale ? "IMU stale" : "OK");
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
    a.status.push_back(s); diag_pub_->publish(a);
  }

  reduced_odom::ReducedEkf ekf_;
  reduced_odom::ReducedEkf::V5 q_;
  std::string wheel_topic_, imu_topic_, output_topic_, odom_frame_, base_frame_;
  double timeout_, max_dt_, r_vx_, r_wz_, r_yaw_, wheel_gate_, yaw_gate_;
  double last_wheel_stamp_{-1.0}, last_imu_stamp_{-1.0}, roll_{0.0}, pitch_{0.0};
  bool have_attitude_{false};
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
