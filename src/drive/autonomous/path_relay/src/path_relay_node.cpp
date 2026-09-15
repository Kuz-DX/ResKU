// path_relay_node: relays a perception-provided track-centerline Path into
// controller_server's FollowPath action, bypassing planner_server/bt_navigator
// entirely (doldrive_ws, 2026-08-10 -- perception-team integration).
//
// ============================================================================
// CONFIRMED WITH PERCEPTION TEAM (as of 2026-08-10):
//   - They publish a track-centerline nav_msgs/Path every frame, 15 Hz.
//   - Path.header.frame_id = "camera_link" (robot-relative, NOT a fixed/global
//     frame -- must be transformed before handing to FollowPath).
//   - Both slopes and flat ground are represented in this single Path topic;
//     there is no separate "slope mode" signal from this topic.
//
// STILL UNCONFIRMED (do not hardcode assumptions -- see parameters below):
//   - Exact topic name.
//   - Number of points per Path / whether poses' orientation fields are
//     filled in meaningfully or left as default identity.
//   - What perception sends when the track is lost (empty Path? topic just
//     stops publishing? some other signal?) -- see TODO near path_timeout_.
//
// OUT OF SCOPE FOR THIS NODE (explicitly deferred, do not implement here):
//   - /terrain/side_slope_angle_deg is NOT read or used anywhere in this
//     file. How it should factor into control (predictive slope handling,
//     relationship to the existing IMU-based SlopeCritic, etc.) is a design
//     question still being discussed separately -- see HANDOFF.md.
//
// STATIC TF: base_link->camera_link and base_link->imu_link are now published
// from measured CAD offsets in robot_bringup/launch/reduced_odom_bringup.launch.py ([하림 수정],
// see that file for the actual numbers/rationale) -- no longer 0,0,0
// placeholders. This node's logic doesn't assume any particular values -- it
// just does a standard tf2 lookupTransform() -- but the frame_id this node
// actually receives on the Path topic still needs to be verified against
// real hardware: realsense2_camera's camera_namespace/camera_name launch
// args can prefix its published frame names, so whether the incoming Path's
// frame_id is literally "camera_link" (vs. e.g. "drive/camera_link") is NOT
// confirmed yet. Verify with `ros2 run tf2_tools view_frames` or
// `ros2 topic echo <path topic> --field header.frame_id` once the camera is
// running with its real launch args, and update target_frame/this comment
// (or add a frame_id override param) accordingly.
// ============================================================================

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class PathRelayNode : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  PathRelayNode()
  : Node("path_relay_node")
  {
    // --- Parameters -----------------------------------------------------
    // Topic name is UNCONFIRMED with perception -- parameterized so it can
    // be changed via launch/yaml without a code change or rebuild.
    // 외부 인지 시스템의 기본 경로 토픽은 '/path'.
    input_path_topic_ = declare_parameter<std::string>(
      "input_path_topic", "/path");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    follow_path_action_name_ = declare_parameter<std::string>(
      "follow_path_action_name", "follow_path");
    // Empty string lets controller_server auto-resolve to its single
    // configured plugin (see ControllerServer::findControllerId /
    // findGoalCheckerId in nav2_controller -- confirmed by reading the
    // actual Humble source: an empty id resolves to the only registered
    // plugin when exactly one is configured, which matches our setup:
    // controller_plugins: ["FollowPath"], and goal_checker_plugins is
    // unset so it defaults to nav2_controller's built-in SimpleGoalChecker).
    controller_id_ = declare_parameter<std::string>("controller_id", "");
    goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", "");

    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.1);

    // What to do when a tf2 lookup genuinely throws (transform tree broken,
    // e.g. a static_transform_publisher not running, or the path's stamp is
    // outside the buffer's time window) -- distinct from the transforms
    // simply being geometrically wrong because they're still placeholder
    // values (that's a silent accuracy problem this node can't detect at
    // all, not a lookup failure). Default "stop": if we can't resolve
    // camera_link->odom at all, we don't know where the track is relative
    // to the robot, so continuing to blindly execute a stale/last-known
    // path risks driving off the actual track. "hold_last" is available for
    // sites where momentary TF hiccups are common and a full stop on every
    // blip is too disruptive -- but that's a tradeoff decision, not a
    // default we should assume; flagged for the user to confirm.
    tf_failure_behavior_ = declare_parameter<std::string>("tf_failure_behavior", "stop");
    if (tf_failure_behavior_ != "stop" && tf_failure_behavior_ != "hold_last") {
      RCLCPP_WARN(
        get_logger(), "Unknown tf_failure_behavior '%s', defaulting to 'stop'",
        tf_failure_behavior_.c_str());
      tf_failure_behavior_ = "stop";
    }

    // TODO(perception-team): what perception actually sends when the track
    // is lost (empty Path? topic stops publishing entirely? some other
    // signal?) is NOT YET CONFIRMED. This timeout is only a basic safety
    // net for the "topic stops publishing" case -- revisit once that's
    // decided.
    path_timeout_sec_ = declare_parameter<double>("path_timeout_sec", 1.0);
    path_timeout_check_period_sec_ = declare_parameter<double>(
      "path_timeout_check_period_sec", 0.2);

    qos_depth_ = declare_parameter<int>("qos_depth", 1);

    // 기본값 0.0: 인지팀이 발행한 원본 경로를 그대로 사용하고 경로 끝을
    // 가상으로 연장하지 않는다. 필요할 때만 양수 값으로 명시적으로 활성화.
    path_extension_m_ = declare_parameter<double>("path_extension_m", 0.0);

    // perception -> MPPI path pipeline 진단용. true일 때만 /debug/path_raw,
    // /debug/path_relay_transformed를 publish한다 -- 실주행에서는 false로 둬서
    // 오버헤드(추가 publish 2회/cycle)가 전혀 없게 한다. 기존 TF 변환/
    // FollowPath 전송 로직은 이 플래그와 무관하게 그대로 동작한다.
    debug_path_pipeline_ = declare_parameter<bool>("debug_path_pipeline", false);

    RCLCPP_INFO(
      get_logger(),
      "path_relay_node starting: input_path_topic='%s' target_frame='%s' "
      "follow_path_action_name='%s' tf_failure_behavior='%s' path_timeout_sec=%.2f",
      input_path_topic_.c_str(), target_frame_.c_str(),
      follow_path_action_name_.c_str(), tf_failure_behavior_.c_str(), path_timeout_sec_);

    // --- TF ---------------------------------------------------------------
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- Action client ------------------------------------------------
    follow_path_client_ = rclcpp_action::create_client<FollowPath>(
      this, follow_path_action_name_);

    // --- Path pipeline debug publishers (opt-in, see debug_path_pipeline_) --
    if (debug_path_pipeline_) {
      debug_path_raw_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/debug/path_raw", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
      debug_path_relay_transformed_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/debug/path_relay_transformed", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    }

    // --- Path subscription ----------------------------------------------
    // Reliable QoS, keep-last(1): we only ever care about the freshest
    // path. NOTE: perception's actual publisher QoS is unconfirmed -- if
    // this node receives zero messages once the real topic is wired up,
    // check for a QoS mismatch (e.g. if they publish best-effort/sensor
    // data QoS, this subscription's default "reliable" won't connect).
    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth_)));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      input_path_topic_, qos,
      std::bind(&PathRelayNode::pathCallback, this, std::placeholders::_1));

    // --- Path-timeout safety net -----------------------------------------
    last_path_receive_time_ = get_clock()->now();
    timeout_timer_ = create_wall_timer(
      std::chrono::duration<double>(path_timeout_check_period_sec_),
      std::bind(&PathRelayNode::timeoutCheckCallback, this));
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    last_path_receive_time_ = get_clock()->now();
    stopped_due_to_timeout_ = false;

    if (msg->poses.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received an empty Path on '%s' -- not sending a FollowPath goal "
        "(perception-team invalid-track signaling is not yet finalized, "
        "see TODO in HANDOFF.md)", input_path_topic_.c_str());
      return;
    }

    if (debug_path_pipeline_) {
      // step 1: perception이 이 노드에 실제로 넣어준 원본 /path 그대로
      // (재계산 없이) republish -- 별도 변환/필터링 없이 msg 그 자체.
      debug_path_raw_pub_->publish(*msg);
    }

    // [하림 수정] 진단용 경고만 추가 -- 기존 TF 변환 동작(target_frame_/
    // tf_failure_behavior_)은 그대로 둔다. 이 frame_id는 경로 생성부
    // 외부 인지 시스템이 body(=camera_link) 좌표계로
    // 명시하도록 고쳐졌어야 하므로, 정상이라면 여기 안 걸려야 한다. 그래도
    // *_optical_frame이 들어오면 좌표(x=전방/y=좌측 body 규약)와 frame_id
    // (optical: x=오른쪽/y=아래/z=전방)가 안 맞을 가능성이 높다는 신호라
    // 경고만 남긴다 -- 이 노드가 임의로 frame_id를 바꾸거나 무시하지는
    // 않는다 (다른 외부 Path가 실제 optical 좌표로 들어오는 경우까지
    // 잘못 처리하게 될 수 있어서).
    static const std::string kOpticalFrameSuffix = "_optical_frame";
    const std::string & frame_id = msg->header.frame_id;
    if (frame_id.size() >= kOpticalFrameSuffix.size() &&
      frame_id.compare(
        frame_id.size() - kOpticalFrameSuffix.size(),
        kOpticalFrameSuffix.size(), kOpticalFrameSuffix) == 0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Path on '%s' has frame_id '%s' (looks like an optical frame). "
        "Path coordinates are expected in body convention (x=forward, "
        "y=left) -- an optical-frame header likely means the wrong frame "
        "was attached upstream. Not overriding it here; fix at the source.",
        input_path_topic_.c_str(), frame_id.c_str());
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      // Look up the transform AT the path's own timestamp (not "latest
      // available") so tf2 interpolates using its time history -- this is
      // what lets a 15 Hz, robot-relative path track a moving robot
      // accurately instead of being off by however stale "latest" TF is.
      transform = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, msg->header.stamp,
        tf2::durationFromSec(tf_lookup_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF lookup %s -> %s failed: %s",
        target_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      handleTfFailure();
      return;
    }

    nav_msgs::msg::Path transformed;
    transformed.header.frame_id = target_frame_;
    transformed.header.stamp = msg->header.stamp;
    transformed.poses.reserve(msg->poses.size());
    for (const auto & pose_in : msg->poses) {
      geometry_msgs::msg::PoseStamped pose_out;
      tf2::doTransform(pose_in, pose_out, transform);
      pose_out.header.frame_id = target_frame_;
      pose_out.header.stamp = msg->header.stamp;
      transformed.poses.push_back(pose_out);
    }

    extendPathTail(transformed);

    if (debug_path_pipeline_) {
      // step 2: FollowPath goal(goal_msg.path = path)에 실제로 들어가는 것과
      // 동일한 Path 객체 -- extendPathTail까지 반영된 이후, sendFollowPathGoal
      // 호출 직전 시점 그대로. 별도로 재계산하지 않는다.
      debug_path_relay_transformed_pub_->publish(transformed);
    }

    sendFollowPathGoal(transformed);
  }

  // 경로 끝을 마지막 세그먼트 방향으로 path_extension_m_만큼 직선 연장.
  // 실제 인지 데이터가 아니라 추정치이므로, 이 연장 구간에 PathAlignCritic
  // 비용이 실릴 정도로 로봇이 실제 진입하기 전에 항상 더 최신 경로로
  // 덮어써진다는 전제 (15Hz로 갱신됨) -- 그 전제가 깨지면(경로 발행이
  // 끊기면) path_timeout_sec 타임아웃이 별도로 로봇을 정지시킴.
  void extendPathTail(nav_msgs::msg::Path & path)
  {
    if (path_extension_m_ <= 0.0 || path.poses.size() < 2) {
      return;
    }

    const auto & p_last = path.poses.back().pose.position;
    const auto & p_prev = path.poses[path.poses.size() - 2].pose.position;
    double dx = p_last.x - p_prev.x;
    double dy = p_last.y - p_prev.y;
    const double seg_len = std::hypot(dx, dy);
    if (seg_len < 1e-6) {
      return;  // 마지막 세그먼트에 방향이 없음 (같은 점 중복 등)
    }
    dx /= seg_len;
    dy /= seg_len;

    const double yaw = std::atan2(dy, dx);
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);

    const double step = std::max(0.05, seg_len);  // 원래 점 간격과 비슷하게
    for (double d = step; d <= path_extension_m_; d += step) {
      geometry_msgs::msg::PoseStamped pose_out;
      pose_out.header = path.header;
      pose_out.pose.position.x = p_last.x + dx * d;
      pose_out.pose.position.y = p_last.y + dy * d;
      pose_out.pose.position.z = p_last.z;
      pose_out.pose.orientation = q;
      path.poses.push_back(pose_out);
    }
  }

  void handleTfFailure()
  {
    if (tf_failure_behavior_ == "hold_last") {
      // Simply don't send a new goal -- controller_server keeps executing
      // whatever the last successfully-transformed path was.
      return;
    }
    // "stop": cancel whatever FollowPath goal is currently active so
    // controller_server halts the robot (publishes zero velocity on
    // cancel -- confirmed in nav2_controller's computeControl()).
    cancelActiveGoal("TF lookup failure");
  }

  void sendFollowPathGoal(const nav_msgs::msg::Path & path)
  {
    if (!follow_path_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "FollowPath action server ('%s') not available yet -- is "
        "controller_server active?", follow_path_action_name_.c_str());
      return;
    }

    auto goal_msg = FollowPath::Goal();
    goal_msg.path = path;
    goal_msg.controller_id = controller_id_;
    goal_msg.goal_checker_id = goal_checker_id_;

    // Deliberately no explicit cancel before sending. Confirmed by reading
    // nav2_controller's actual source (ControllerServer::updateGlobalPath(),
    // nav2_util::SimpleActionServer::handle_accepted()/accept_pending_goal()):
    // sending a new goal while one is executing is accepted as a "pending"
    // goal and swapped in transparently by the control loop on its next
    // iteration (at controller_frequency_, 20 Hz here) via
    // is_preempt_requested()/accept_pending_goal() -- no cancel round-trip,
    // no gap in control output, no MPPI state reset. If a second new goal
    // arrives before the first pending one was even consumed, the server
    // itself aborts the stale pending goal and replaces it (see
    // SimpleActionServer::handle_accepted()) -- so sending at 15 Hz cannot
    // build an unbounded queue; it always coalesces to the freshest path.
    // See the "액션 재전송 전략 조사 결과" report for the full writeup.
    rclcpp_action::Client<FollowPath>::SendGoalOptions send_goal_options;
    send_goal_options.goal_response_callback =
      [this](const GoalHandleFollowPath::SharedPtr & handle) {
        if (handle) {
          current_goal_handle_ = handle;
        }
      };
    send_goal_options.result_callback =
      [this](const GoalHandleFollowPath::WrappedResult & result) {
        // A superseded (preempted) goal is aborted server-side by design
        // (see SimpleActionServer::terminate()) -- that's the expected,
        // normal outcome at 15 Hz and not worth logging above DEBUG.
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED &&
          result.code != rclcpp_action::ResultCode::ABORTED)
        {
          RCLCPP_WARN(
            get_logger(), "FollowPath goal ended with unexpected result code %d",
            static_cast<int>(result.code));
        } else {
          RCLCPP_DEBUG(
            get_logger(), "FollowPath goal ended with result code %d",
            static_cast<int>(result.code));
        }
      };

    follow_path_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void timeoutCheckCallback()
  {
    const auto elapsed = (get_clock()->now() - last_path_receive_time_).seconds();
    if (elapsed > path_timeout_sec_ && !stopped_due_to_timeout_) {
      RCLCPP_WARN(
        get_logger(),
        "No Path received on '%s' for %.2fs (timeout %.2fs) -- stopping.",
        input_path_topic_.c_str(), elapsed, path_timeout_sec_);
      cancelActiveGoal("path timeout");
      stopped_due_to_timeout_ = true;
    }
  }

  void cancelActiveGoal(const std::string & reason)
  {
    if (!current_goal_handle_) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Cancelling active FollowPath goal (%s).", reason.c_str());
    follow_path_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_.reset();
  }

  // Parameters
  std::string input_path_topic_;
  std::string target_frame_;
  std::string follow_path_action_name_;
  std::string controller_id_;
  std::string goal_checker_id_;
  double tf_lookup_timeout_sec_{0.1};
  std::string tf_failure_behavior_;
  double path_timeout_sec_{1.0};
  double path_timeout_check_period_sec_{0.2};
  int qos_depth_{1};
  double path_extension_m_{0.0};
  bool debug_path_pipeline_{false};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_raw_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_relay_transformed_pub_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // State
  rclcpp::Time last_path_receive_time_;
  bool stopped_due_to_timeout_{false};
  GoalHandleFollowPath::SharedPtr current_goal_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathRelayNode>());
  rclcpp::shutdown();
  return 0;
}
