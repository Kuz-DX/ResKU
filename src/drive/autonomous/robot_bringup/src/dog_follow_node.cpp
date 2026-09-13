#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>

using namespace std::chrono_literals;

// dog_follow_node: 로봇개 추종 미션 전용 주행 제어 노드. MPPI/perception
// 경로(=/path)를 아예 안 쓴다 -- 이 미션은 "매 순간 로봇개의 상대위치를 보고
// 거리/방향을 유지하며 쫓아간다"는 반응형(reactive) 제어라, path 기반이
// 아니라 target-following 제어로 짰다.
//
// [2026-08-29 신규] 시나리오(사용자 설명 그대로):
//   1) 로봇개가 1m/s로 이동, 로봇은 1.5~2.5m 거리를 유지하며 추종
//   2) 10m 직선 구간 -- 시야 차단 없이 정상 추종
//   3) 5m 반원 구간 -- 역시 시야 차단 없이 정상 추종
//   4) 반원 직후 5m 가림막 구간 -- 로봇개가 시야에서 사라짐, 로봇은 멈춤
//   5) 가림막을 지나 로봇개를 다시 포착하면(이때 로봇개는 멈춰있던 동안
//      계속 이동해서 멀어져 있음) 가속도를 붙여 따라잡고, 다시
//      1.5~2.5m 거리로 재추종(5m 직선 구간)
//
// [설계 방침] 위 구간 길이(10m/5m/5m/5m)는 이 노드가 직접 알 필요가 없다
// -- 트랙 지도를 하드코딩하는 대신, "로봇개가 지금 보이는지"와 "지금
// 거리가 목표 범위 안인지"라는 두 가지 관측 가능한 신호만으로 상태를
// 결정한다. 그래야 실제 트랙 길이가 설계와 조금 달라도(가림막이 정확히
// 5m가 아니어도) 그대로 동작한다.
//
// 상태머신:
//   IDLE -> TRACKING <-> STOPPED_OCCLUDED -> CATCHUP -> TRACKING
//
//   - IDLE(초기 상태): 정지. dog_pose_topic_에서 로봇개 위치가 처음
//     들어오면(dog_pose_timeout_sec_ 이내 신선한 데이터) TRACKING으로 진입.
//   - TRACKING: 로봇개가 보이는 동안의 정상 추종. vx는
//     cruise_speed_mps_(로봇개 순항 속도 추정치, 기본 1.0m/s)를
//     feedforward로 깔고, 거리 오차(distance - follow_distance_target_m_,
//     기본 목표 2.0m = 1.5~2.5m 중앙)에 비례한 보정(kp_distance_)을 더해서
//     계산한다 -- 거리가 일정하게 유지되는 정상상태에서는 결국 로봇개
//     속도와 같아지므로 별도의 속도 추정 없이도 "leader-follower" 방식으로
//     자연히 속도가 맞춰진다. wz는 로봇개 방향(bearing, atan2(y,x))에
//     비례한 P제어(kp_bearing_)로 로봇개를 정면에 두도록 조향한다.
//     max_vx_tracking_/max_wz_로 각각 클램프. 로봇개 데이터가
//     dog_pose_timeout_sec_ 이상 안 들어오면(=가림막 진입, 위 4번)
//     STOPPED_OCCLUDED로 전환.
//   - STOPPED_OCCLUDED: vx=wz=0 완전 정지. 로봇개 데이터가 다시 들어오면
//     (=가림막 통과, 위 5번) 얼마나 오래 안 보였든 상관없이 CATCHUP으로
//     전환한다. [2026-08-29] 처음엔 다른 노드들(slope_traverse_node 등)과
//     같은 안전 철학으로 "일정 시간(30초) 넘게 못 찾으면 래치(재시작
//     전까지 재개 안 함)"를 넣었었는데, 여기서는 "로봇개를 오래 놓침"이
//     하드웨어 고장 같은 진짜 결함이 아니라 그냥 일시적으로 시야에서
//     사라진 것뿐이라 성격이 다르다고 판단해 제거했다(사용자 확인) --
//     안 보이는 동안은 이미 vx=wz=0으로 정지해 있으니 래치 없이도 위험한
//     상황은 안 생기고, 오히려 예상보다 살짝 더 오래 걸린 뒤 다시 나타난
//     정상적인 경우까지 미션을 통째로 날리는 게 더 나쁜 실패 모드였다.
//   - CATCHUP: TRACKING과 같은 제어식이지만 max_vx_catchup_(기본 2.5m/s,
//     max_vx_tracking_보다 큼)로 상한을 높여서 "가속도를 붙여 따라잡는다"는
//     요구사항을 반영 -- 가림막 동안 멀어진 거리를 더 빠르게 좁힌다. 현재
//     거리가 follow_distance_max_m_(2.5m) 이하로 들어오면(=다시 정상 추종
//     범위 진입, 위 5번 마지막) TRACKING으로 복귀해서 max_vx_tracking_로
//     되돌아간다. 이 상태에서도 데이터가 끊기면 STOPPED_OCCLUDED로.
//
// dog_pose_topic_ 인터페이스는 PLACEHOLDER다 -- 인지팀과 토픽명/메시지
// 타입이 아직 확정되지 않음. 현재는 geometry_msgs/PointStamped(로봇 상대
// 좌표, x=전방/y=좌측 -- /path와 동일한 body 좌표 규약)로 가정했다.
// 실제로는 bounding box나 별도 커스텀 메시지로 올 수도 있으니, 확정되면
// dogPoseCallback()과 구독 타입만 바꾸면 되도록 다른 로직과 분리해뒀다.
//
// 출력은 /cmd_vel_auto(cmd_vel_output_topic_) -- summer_supply_drive_node와
// 동일하게, 이 미션에서는 nav2/controller_server를 아예 안 띄우므로 이
// 노드가 그 토픽의 유일한 publisher가 되는 걸 전제로 한다(mission_dog_follow.launch.py
// 참고). stability_monitor_node(통신두절/하드웨어 에러 감지)는 이 체인과
// 무관하게 /cmd_vel_safety로 항상 동작한다.
class DogFollowNode : public rclcpp::Node
{
public:
  DogFollowNode()
  : Node("dog_follow_node")
  {
    // --- 인지팀 인터페이스 (PLACEHOLDER, 파일 상단 참고) -------------------
    dog_pose_topic_ = declare_parameter<std::string>(
      "dog_pose_topic", "/perception/dog_pose");
    dog_pose_timeout_sec_ = declare_parameter<double>("dog_pose_timeout_sec", 0.5);

    // --- 추종 거리/속도 ----------------------------------------------------
    follow_distance_min_m_ = declare_parameter<double>("follow_distance_min_m", 1.5);
    follow_distance_max_m_ = declare_parameter<double>("follow_distance_max_m", 2.5);
    follow_distance_target_m_ = declare_parameter<double>("follow_distance_target_m", 2.0);
    cruise_speed_mps_ = declare_parameter<double>("cruise_speed_mps", 1.0);
    kp_distance_ = declare_parameter<double>("kp_distance", 0.5);
    kp_bearing_ = declare_parameter<double>("kp_bearing", 1.0);
    max_vx_tracking_ = declare_parameter<double>("max_vx_tracking", 1.5);
    // [가림막 통과 후 따라잡기] TRACKING보다 높은 상한 -- "가속도를 붙여
    // 따라잡는다"는 요구사항.
    max_vx_catchup_ = declare_parameter<double>("max_vx_catchup", 2.5);
    max_wz_ = declare_parameter<double>("max_wz", 1.0);

    // --- 출력/제어 주기 ------------------------------------------------------
    cmd_vel_output_topic_ = declare_parameter<std::string>(
      "cmd_vel_output_topic", "/cmd_vel_auto");
    state_topic_ = declare_parameter<std::string>("state_topic", "/drive/dog_follow_state");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/dog_follow/debug");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_output_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(1).transient_local());
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(debug_topic_, 10);

    dog_pose_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      dog_pose_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&DogFollowNode::dogPoseCallback, this, std::placeholders::_1));

    state_entered_time_ = get_clock()->now();

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DogFollowNode::controlTimerCallback, this));

    // 실차 튜닝 중 재시작 없이 ros2 param set으로 값을 바로 반영하기 위한
    // 동적 콜백 (slope_traverse_node.cpp와 동일 패턴).
    tunable_params_ = {
      {"dog_pose_timeout_sec", &dog_pose_timeout_sec_},
      {"follow_distance_min_m", &follow_distance_min_m_},
      {"follow_distance_max_m", &follow_distance_max_m_},
      {"follow_distance_target_m", &follow_distance_target_m_},
      {"cruise_speed_mps", &cruise_speed_mps_},
      {"kp_distance", &kp_distance_},
      {"kp_bearing", &kp_bearing_},
      {"max_vx_tracking", &max_vx_tracking_},
      {"max_vx_catchup", &max_vx_catchup_},
      {"max_wz", &max_wz_},
    };
    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&DogFollowNode::onParamUpdate, this, std::placeholders::_1));

    publishState();

    RCLCPP_INFO(
      get_logger(),
      "dog_follow_node started: dog_pose_topic=%s follow_distance=%.1f~%.1fm "
      "cruise_speed=%.2fm/s max_vx_tracking=%.2f max_vx_catchup=%.2f -> initial state IDLE",
      dog_pose_topic_.c_str(), follow_distance_min_m_, follow_distance_max_m_,
      cruise_speed_mps_, max_vx_tracking_, max_vx_catchup_);
  }

private:
  enum class State { IDLE, TRACKING, STOPPED_OCCLUDED, CATCHUP };

  static const char * stateName(State s)
  {
    switch (s) {
      case State::IDLE: return "idle";
      case State::TRACKING: return "tracking";
      case State::STOPPED_OCCLUDED: return "stopped_occluded";
      case State::CATCHUP: return "catchup";
    }
    return "unknown";
  }

  // tunable_params_에 등록된 double 파라미터가 ros2 param set으로 바뀌면
  // 해당 멤버 변수에 즉시 반영 -- 재시작 없이 실차 튜닝하기 위함.
  rcl_interfaces::msg::SetParametersResult onParamUpdate(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      auto it = tunable_params_.find(p.get_name());
      if (it != tunable_params_.end() &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        *(it->second) = p.as_double();
        RCLCPP_INFO(
          get_logger(), "파라미터 실시간 반영: %s = %f", p.get_name().c_str(), p.as_double());
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // 로봇 상대좌표(x=전방, y=좌측) -- /path와 동일한 body 좌표 규약(파일
  // 상단 docstring 참고).
  void dogPoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    last_dog_x_ = msg->point.x;
    last_dog_y_ = msg->point.y;
    has_dog_data_ = true;
    last_dog_time_ = get_clock()->now();
  }

  bool dogDataFresh()
  {
    return has_dog_data_ &&
      (get_clock()->now() - last_dog_time_).seconds() <= dog_pose_timeout_sec_;
  }

  void updateState()
  {
    const rclcpp::Time now = get_clock()->now();
    const State prev = state_;
    const bool fresh = dogDataFresh();
    const double distance = std::hypot(last_dog_x_, last_dog_y_);

    switch (state_) {
      case State::IDLE: {
        if (fresh) {
          state_ = State::TRACKING;
        }
        break;
      }
      case State::TRACKING: {
        if (!fresh) {
          state_ = State::STOPPED_OCCLUDED;
        }
        break;
      }
      case State::STOPPED_OCCLUDED: {
        if (fresh) {
          // 가림막을 통과해 로봇개를 다시 포착 -- 얼마나 오래 안 보였든
          // 상관없이 CATCHUP으로 진입한다(가림막 동안 멀어져 있을
          // 가능성이 높으므로 일단 CATCHUP의 더 높은 상한으로 시작하되,
          // 거리가 이미 follow_distance_max_m_ 이내라면 다음 tick에 바로
          // TRACKING으로 넘어간다). 래치 없음 -- 파일 상단 docstring 참고.
          state_ = State::CATCHUP;
        }
        break;
      }
      case State::CATCHUP: {
        if (!fresh) {
          state_ = State::STOPPED_OCCLUDED;
        } else if (distance <= follow_distance_max_m_) {
          // 목표 추종 거리 범위로 다시 들어옴 -- 정상 추종 재개.
          state_ = State::TRACKING;
        }
        break;
      }
    }

    if (state_ != prev) {
      state_entered_time_ = now;
      publishState();
      RCLCPP_INFO(
        get_logger(), "state %s -> %s (distance=%.2fm)", stateName(prev), stateName(state_),
        distance);
    }
  }

  void controlTimerCallback()
  {
    updateState();

    geometry_msgs::msg::Twist cmd;
    double distance = 0.0;
    double bearing_deg = 0.0;

    if (state_ == State::TRACKING || state_ == State::CATCHUP) {
      distance = std::hypot(last_dog_x_, last_dog_y_);
      const double bearing_rad = std::atan2(last_dog_y_, last_dog_x_);
      bearing_deg = bearing_rad * 180.0 / M_PI;

      const double max_vx = (state_ == State::CATCHUP) ? max_vx_catchup_ : max_vx_tracking_;
      double vx = cruise_speed_mps_ + kp_distance_ * (distance - follow_distance_target_m_);
      vx = std::clamp(vx, 0.0, max_vx);

      double wz = kp_bearing_ * bearing_rad;
      wz = std::clamp(wz, -max_wz_, max_wz_);

      cmd.linear.x = vx;
      cmd.angular.z = wz;
    } else {
      // IDLE / STOPPED_OCCLUDED -- 완전 정지.
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);
    publishDebug(distance, bearing_deg, cmd.linear.x, cmd.angular.z);
  }

  void publishDebug(double distance, double bearing_deg, double vx_cmd, double wz_cmd)
  {
    std_msgs::msg::Float64MultiArray out;
    out.data = {
      static_cast<double>(static_cast<int>(state_)),
      distance,
      bearing_deg,
      vx_cmd,
      wz_cmd,
      dogDataFresh() ? 1.0 : 0.0,
      (get_clock()->now() - state_entered_time_).seconds(),
    };
    debug_pub_->publish(out);
  }

  void publishState()
  {
    std_msgs::msg::String out;
    out.data = stateName(state_);
    state_pub_->publish(out);
  }

  // 인지팀 인터페이스 (PLACEHOLDER)
  std::string dog_pose_topic_;
  double dog_pose_timeout_sec_;

  // 추종 거리/속도
  double follow_distance_min_m_;
  double follow_distance_max_m_;
  double follow_distance_target_m_;
  double cruise_speed_mps_;
  double kp_distance_;
  double kp_bearing_;
  double max_vx_tracking_;
  double max_vx_catchup_;
  double max_wz_;

  // 출력/제어 주기
  std::string cmd_vel_output_topic_;
  std::string state_topic_;
  std::string debug_topic_;
  double control_rate_hz_;

  State state_{State::IDLE};
  rclcpp::Time state_entered_time_;

  double last_dog_x_{0.0};
  double last_dog_y_{0.0};
  bool has_dog_data_{false};
  rclcpp::Time last_dog_time_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr dog_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 실차 튜닝용 동적 파라미터 콜백(onParamUpdate() 참고)
  std::unordered_map<std::string, double *> tunable_params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DogFollowNode>());
  rclcpp::shutdown();
  return 0;
}
