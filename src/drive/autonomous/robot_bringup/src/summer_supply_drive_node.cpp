#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mission_manager_interfaces/msg/mission_result.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

// 정지 상태에서 파지를 기다린다. picking_command 이후 경로를 추종하며,
// 신호등 정지 요청은 항상 우선한다. 파지 전 차체 접근 동작은 없다.
class SummerSupplyDriveNode : public rclcpp::Node
{
public:
  SummerSupplyDriveNode()
  : Node("summer_supply_drive_node")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    picking_command_topic_ = declare_parameter<std::string>(
      "picking_command_topic", "/arm/picking_command");
    traffic_result_topic_ = declare_parameter<std::string>(
      "traffic_result_topic", "/mission/summer_traffic/result");
    cmd_vel_output_topic_ = declare_parameter<std::string>(
      "cmd_vel_output_topic", "/cmd_vel_auto");
    path_topic_ = declare_parameter<std::string>("path_topic", "/path");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    lookahead_distance_m_ = declare_parameter<double>("lookahead_distance_m", 0.5);
    path_timeout_sec_ = declare_parameter<double>("path_timeout_sec", 1.0);
    hold_last_path_sec_ = declare_parameter<double>("hold_last_path_sec", 5.0);
    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.1);
    max_angular_speed_rad_s_ =
      declare_parameter<double>("max_angular_speed_rad_s", 0.9);
    recovery_turn_threshold_rad_ =
      declare_parameter<double>("recovery_turn_threshold_rad", 0.7);
    recovery_turn_gain_ = declare_parameter<double>("recovery_turn_gain", 1.5);

    cruise_vx_ = declare_parameter<double>("cruise_vx", 0.3927);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_output_topic_, 10);

    picking_cmd_sub_ = create_subscription<std_msgs::msg::Empty>(
      picking_command_topic_, 10,
      std::bind(&SummerSupplyDriveNode::pickingCommandCallback, this, std::placeholders::_1));
    traffic_sub_ = create_subscription<mission_manager_interfaces::msg::MissionResult>(
      traffic_result_topic_, 10,
      std::bind(&SummerSupplyDriveNode::trafficResultCallback, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1),
      std::bind(&SummerSupplyDriveNode::pathCallback, this, std::placeholders::_1));

    state_entered_time_ = get_clock()->now();

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SummerSupplyDriveNode::controlTimerCallback, this));

    RCLCPP_INFO(get_logger(),
      "summer_supply_drive_node started; WAIT_PICK until picking_command (%s)",
      picking_command_topic_.c_str());
  }

private:
  enum class State { DRIVING, WAIT_PICK, STOPPED_TRAFFIC };

  static const char * stateName(State s)
  {
    switch (s) {
      case State::DRIVING: return "driving";
      case State::WAIT_PICK: return "wait_pick";
      case State::STOPPED_TRAFFIC: return "stopped_traffic";
    }
    return "unknown";
  }

  void pickingCommandCallback(const std_msgs::msg::Empty::SharedPtr)
  {
    has_picked_ = true;
  }

  void trafficResultCallback(const mission_manager_interfaces::msg::MissionResult::SharedPtr msg)
  {
    if (!msg->valid) {
      return;  // 무효 판정은 무시 -- 파일 상단 docstring 참고
    }
    if (msg->state == "stop") {
      traffic_stop_requested_ = true;
    } else if (msg->state == "go") {
      traffic_stop_requested_ = false;
    }
    // 'unknown'이나 다른 문자열은 무시(현재 상태 유지).
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty() || msg->header.frame_id.empty()) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        fixed_frame_, msg->header.frame_id, msg->header.stamp,
        tf2::durationFromSec(tf_lookup_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "summer pure pursuit TF %s <- %s failed: %s; holding last path",
        fixed_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }

    std::vector<std::pair<double, double>> transformed_path;
    transformed_path.reserve(msg->poses.size());
    for (const auto & pose_in : msg->poses) {
      geometry_msgs::msg::PoseStamped pose_out;
      tf2::doTransform(pose_in, pose_out, transform);
      transformed_path.emplace_back(pose_out.pose.position.x, pose_out.pose.position.y);
    }
    path_fixed_xy_ = std::move(transformed_path);
    last_path_time_ = get_clock()->now();
    has_path_ = true;
  }

  struct PursuitCommand
  {
    double curvature;
    double bearing_rad;
    bool holding_last;
  };

  std::optional<PursuitCommand> computePurePursuitCommand()
  {
    if (!has_path_ || path_fixed_xy_.empty()) {
      return std::nullopt;
    }
    const double path_age = (get_clock()->now() - last_path_time_).seconds();
    if (path_age > hold_last_path_sec_) {
      return std::nullopt;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      // 저장된 마지막 경로는 odom에 고정돼 있다. 매 제어 tick 현재 자세로
      // base_link에 재투영해야 자갈에서 차체가 돌아도 원래 경로 방향이 보인다.
      transform = tf_buffer_->lookupTransform(
        base_frame_, fixed_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_lookup_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "summer recovery TF %s <- %s failed: %s",
        base_frame_.c_str(), fixed_frame_.c_str(), ex.what());
      return std::nullopt;
    }

    std::vector<std::pair<double, double>> path_base_xy;
    path_base_xy.reserve(path_fixed_xy_.size());
    for (const auto & point : path_fixed_xy_) {
      geometry_msgs::msg::PoseStamped in;
      geometry_msgs::msg::PoseStamped out;
      in.header.frame_id = fixed_frame_;
      in.pose.position.x = point.first;
      in.pose.position.y = point.second;
      in.pose.orientation.w = 1.0;
      tf2::doTransform(in, out, transform);
      path_base_xy.emplace_back(out.pose.position.x, out.pose.position.y);
    }

    // 마지막 경로 위에서 현재 로봇에 가장 가까운 점부터 경로 진행 방향으로
    // lookahead를 잡는다. 오래된 경로의 이미 지나온 첫 점을 다시 목표로 삼아
    // 뒤로 돌아가는 현상을 막는다.
    size_t nearest_idx = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_base_xy.size(); ++i) {
      const double distance = std::hypot(path_base_xy[i].first, path_base_xy[i].second);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_idx = i;
      }
    }
    size_t target_idx = path_base_xy.size() - 1;
    double along_path = 0.0;
    for (size_t i = nearest_idx + 1; i < path_base_xy.size(); ++i) {
      along_path += std::hypot(
        path_base_xy[i].first - path_base_xy[i - 1].first,
        path_base_xy[i].second - path_base_xy[i - 1].second);
      if (along_path >= lookahead_distance_m_) {
        target_idx = i;
        break;
      }
    }
    const auto * target = &path_base_xy[target_idx];

    const double distance = std::hypot(target->first, target->second);
    if (distance < 1e-3) {
      return std::nullopt;
    }
    return PursuitCommand{
      2.0 * target->second / (distance * distance),
      std::atan2(target->second, target->first),
      path_age > path_timeout_sec_};
  }

  void updateState()
  {
    const State prev = state_;

    if (traffic_stop_requested_) {
      // 신호등 정지가 항상 우선한다.
      state_ = State::STOPPED_TRAFFIC;
    } else {
      state_ = has_picked_ ? State::DRIVING : State::WAIT_PICK;
    }

    if (state_ != prev) {
      state_entered_time_ = get_clock()->now();
      RCLCPP_INFO(
        get_logger(), "state %s -> %s", stateName(prev), stateName(state_));
    }
  }

  void controlTimerCallback()
  {
    updateState();

    geometry_msgs::msg::Twist cmd;
    if (state_ == State::DRIVING) {
      double target_vx = cruise_vx_;
      const auto pursuit = computePurePursuitCommand();
      if (pursuit.has_value()) {
        if (std::abs(pursuit->bearing_rad) >= recovery_turn_threshold_rad_) {
          // 자갈에서 크게 돌아 경로가 옆/뒤로 보이면 전진하지 않고 마지막
          // 경로 쪽으로 먼저 제자리 회전한다.
          cmd.linear.x = 0.0;
          cmd.angular.z = std::clamp(
            recovery_turn_gain_ * pursuit->bearing_rad,
            -max_angular_speed_rad_s_, max_angular_speed_rad_s_);
        } else {
          cmd.linear.x = target_vx;
          cmd.angular.z = std::clamp(
            cmd.linear.x * pursuit->curvature,
            -max_angular_speed_rad_s_, max_angular_speed_rad_s_);
        }
        if (pursuit->holding_last) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Fresh /path lost -- steering toward last path in odom frame.");
        }
      } else {
        // path가 없어도(또는 hold-last 타임아웃을 넘겨도) 정지하지 않고
        // 보정 없이 target_vx로 직진(open-loop)한다 -- path는 있으면
        // 쓰는 보정 수단일 뿐 진행 조건이 아니다.
        cmd.linear.x = target_vx;
        cmd.angular.z = 0.0;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "No usable path (including hold-last timeout) -- driving straight open-loop.");
      }
    } else {
      cmd.linear.x = 0.0;  // WAIT_PICK / STOPPED_TRAFFIC
      cmd.angular.z = 0.0;
    }
    cmd_pub_->publish(cmd);
  }

  std::string picking_command_topic_;
  std::string traffic_result_topic_;
  std::string cmd_vel_output_topic_;
  std::string path_topic_;
  std::string base_frame_;
  std::string fixed_frame_;

  double cruise_vx_;
  double control_rate_hz_;
  double lookahead_distance_m_;
  double path_timeout_sec_;
  double hold_last_path_sec_;
  double tf_lookup_timeout_sec_;
  double max_angular_speed_rad_s_;
  double recovery_turn_threshold_rad_;
  double recovery_turn_gain_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<std::pair<double, double>> path_fixed_xy_;
  rclcpp::Time last_path_time_;
  bool has_path_{false};

  State state_{State::WAIT_PICK};
  rclcpp::Time state_entered_time_;
  bool has_picked_{false};
  bool traffic_stop_requested_{false};

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr picking_cmd_sub_;
  rclcpp::Subscription<mission_manager_interfaces::msg::MissionResult>::SharedPtr traffic_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SummerSupplyDriveNode>());
  rclcpp::shutdown();
  return 0;
}
