// Standalone reproduction harness for the costs_ heap-corruption bug
// (doldrive_ws investigation, 2026-08-09).
//
// Drives mppi::Optimizer directly -- no controller_server process, no
// bt_navigator/planner_server, no DDS traffic with other nodes, no TF, no
// EKF/CAN/IMU chain. Two threads exercise exactly the race condition
// documented in HANDOFF.md section 4:
//   - "control" thread: repeatedly calls Optimizer::evalControl() with a
//     synthetic pose/plan, replicating MPPIController::computeVelocityCommands()'s
//     param_lock + periodic reset_period_ reset pattern.
//   - "param spam" thread: repeatedly calls node->set_parameters() in-process
//     (no `ros2 param set` CLI, no DDS RPC -- this fires the exact same
//     add_on_set_parameters_callback path that dynamicParamsCallback uses).
//
// Rationale for bypassing the full stack: every ASan attempt against the
// real controller_server process has failed at pluginlib's dlopen() time
// with "Shadow memory range interleaves with an existing memory mapping" --
// an environment/ASLR issue, unrelated to the bug under investigation. This
// harness reaches the same dlopen call (CriticManager::loadCritics() is
// still pluginlib-based) but as close to process start as possible, with
// far less prior heap/mmap fragmentation than a full Nav2 stack, in hopes
// ASan's shadow memory setup succeeds far more often.
//
// Usage:
//   repro_harness --ros-args --params-file <our_mppi_params.yaml> \
//     -p use_sim_time:=false
//
// Debug-only. Never run this (or its ASan build) against real hardware.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{
std::atomic<bool> g_running{true};

void onSignal(int) {g_running.store(false);}

nav_msgs::msg::Path makeFakePlan(const rclcpp::Time & stamp, double goal_x)
{
  // 60 poses in a straight line, ~5cm apart -- matches the point counts
  // seen in earlier PathAlignCritic/PathFollowCritic diagnostic logging
  // (path.x.shape(0) == 60), so offset_from_furthest (20) can actually be
  // exceeded and that gated code path runs, unlike the narrower repro used
  // in earlier sessions.
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "odom";
  plan.header.stamp = stamp;

  constexpr int kNumPoses = 60;
  for (int i = 0; i < kNumPoses; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = goal_x * (static_cast<double>(i) / (kNumPoses - 1));
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }
  return plan;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  rclcpp::init(argc, argv);

  // Node name must be "controller_server" to match our_mppi_params.yaml's
  // top-level "controller_server: ros__parameters: FollowPath: ..." nesting.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server");
  // The real controller_server (nav2_controller::ControllerServer) declares
  // this itself with a sensible default; our bare LifecycleNode doesn't, so
  // Optimizer::setOffset()'s controller_period==model_dt check needs it
  // supplied explicitly. 20 Hz matches our model_dt of 0.05s.
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));

  // Minimal costmap: zero layer plugins so we avoid an *additional*
  // pluginlib dlopen (e.g. InflationLayer) beyond the one we're actually
  // investigating. None of our critics (ConstraintCritic, PathAlignCritic,
  // PathFollowCritic, PathAngleCritic, PreferForwardCritic, SkidCritic,
  // SlopeCritic) read costmap occupancy data -- only Optimizer::initialize()
  // needs costmap_ros_->getCostmap() to return a valid pointer, and
  // getBaseFrameID()/getGlobalFrameID() to return frame strings.
  // Costmap2DROS's own constructor already declares these with defaults --
  // just override the values, don't re-declare.
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
  costmap_ros->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
  costmap_ros->set_parameter(rclcpp::Parameter("global_frame", std::string("odom")));
  costmap_ros->set_parameter(rclcpp::Parameter("robot_base_frame", std::string("base_link")));

  // Spin both nodes on a background executor thread -- Costmap2DROS needs
  // its own node spinning for TF/timers even with zero layers, and
  // node->set_parameters() calls from the param-spam thread need the
  // executor alive to route through the registered callback synchronously
  // (it's a direct in-process call, not a service, but keeping the node
  // "alive" in the conventional sense avoids surprises).
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(costmap_ros->get_node_base_interface());
  std::thread executor_thread([&executor]() {executor.spin();});

  // Costmap2DROS's activation-time transform check (base_link -> odom) has
  // no publisher in this harness at all, so it was retrying/blocking for a
  // highly variable 0.5-20+ seconds before giving up and proceeding anyway
  // -- pure noise for repeated-run timing, unrelated to the bug under
  // investigation. Publish a static identity transform so it succeeds
  // immediately every time.
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster(node);
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = node->get_clock()->now();
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.rotation.w = 1.0;
    static_tf_broadcaster.sendTransform(tf_msg);
  }

  costmap_ros->on_configure(rclcpp_lifecycle::State());
  costmap_ros->on_activate(rclcpp_lifecycle::State());

  // Control experiment: REPRO_HARNESS_SUPPRESS_VERBOSE_TRIGGER=1 pre-declares
  // "verbose" before ParametersHandler::start() registers its callback, so
  // the natural "controller_server.verbose not found -> declared -> fires
  // dynamicParamsCallback once -> needs_reset_ set -> a SECOND reset() right
  // after the first one from initialize()" sequence (documented in
  // HANDOFF.md section 5) does NOT happen. Isolates whether costs_'s
  // misalignment requires reset() to run twice back-to-back at the same
  // shape, vs. happening on a single reset() already.
  if (std::getenv("REPRO_HARNESS_SUPPRESS_VERBOSE_TRIGGER") != nullptr) {
    // ParametersHandler::start() actually declares "controller_server.verbose"
    // (node_name_ + "." + "verbose"), not bare "verbose" -- match that exactly
    // or the pre-declare is a silent no-op (as the first attempt was).
    node->declare_parameter("controller_server.verbose", rclcpp::ParameterValue(false));
    fprintf(stderr, "[repro_harness] pre-declared 'controller_server.verbose' (control experiment)\n");
  }

  auto parameters_handler = std::make_unique<mppi::ParametersHandler>(node);

  mppi::Optimizer optimizer;
  // This is the pluginlib dlopen we're trying to give the best possible
  // chance of surviving ASan's shadow memory setup: as early in the
  // process's life as possible, before any threads/param spam start.
  fprintf(stderr, "[repro_harness] initializing Optimizer (dlopen of critics happens here)...\n");
  optimizer.initialize(node, "FollowPath", costmap_ros, parameters_handler.get());
  parameters_handler->start();
  fprintf(stderr, "[repro_harness] Optimizer initialized OK. Starting control + param-spam threads.\n");

  double reset_period = 1.0;
  node->get_parameter_or("FollowPath.reset_period", reset_period, 1.0);

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.orientation.w = 1.0;

  geometry_msgs::msg::Twist robot_speed;
  robot_speed.linear.x = 0.05;

  std::atomic<uint64_t> control_iterations{0};
  std::atomic<uint64_t> param_changes{0};

  // "Control loop" thread: mirrors MPPIController::computeVelocityCommands()
  // exactly -- param_lock acquired first, periodic reset_period_-triggered
  // reset() under that same lock, then evalControl(). See controller.cpp
  // (doldrive_ws fix, 2026-08-08) for why the lock has to come first.
  std::thread control_thread([&]() {
      auto clock = node->get_clock();
      rclcpp::Time last_time_called = clock->now();
      double goal_x = 2.0;
      rclcpp::Time last_goal_change = clock->now();

      while (g_running.load()) {
        {
          std::lock_guard<std::mutex> param_lock(*parameters_handler->getLock());

          if (clock->now() - last_time_called > rclcpp::Duration::from_seconds(reset_period)) {
            optimizer.reset();
          }
          last_time_called = clock->now();

          // Simulate a new navigate_to_pose goal periodically (not required
          // for reset() timing, which is purely time-based, but exercises
          // PathAlignCritic/PathFollowCritic's plan-shape-dependent code
          // with a plan of realistic length, per the D) code-review note
          // that offset_from_furthest_ gating may behave differently here
          // than in the earlier, narrower reproduction).
          if (clock->now() - last_goal_change > rclcpp::Duration::from_seconds(3.0)) {
            goal_x = (goal_x > 1.5) ? 1.0 : 2.5;
            last_goal_change = clock->now();
          }

          auto plan = makeFakePlan(clock->now(), goal_x);
          robot_pose.header.stamp = clock->now();

          try {
            optimizer.evalControl(robot_pose, robot_speed, plan, nullptr);
          } catch (const std::exception & e) {
            fprintf(stderr, "[repro_harness] evalControl threw: %s\n", e.what());
          }
        }
        control_iterations.fetch_add(1, std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // ~20 Hz
      }
    });

  // "Param spam" thread: same mechanism `ros2 param set /controller_server
  // FollowPath.gamma ...` uses under the hood (node->set_parameters()
  // triggers the same on_set_parameters_callback -> dynamicParamsCallback
  // path), but in-process -- no DDS RPC, no `ros2` CLI subprocess, no
  // daemon flakiness. This is the exact repro condition from HANDOFF.md
  // section 4 ("파라미터 스팸 + 목표 동시 전송").
  // Control experiment: REPRO_HARNESS_NO_PARAM_SPAM=1 disables this thread
  // entirely, to check whether the control thread alone (no concurrent
  // parameter changes at all) can still crash -- if so, the bug isn't the
  // param-spam race and something in this harness's own setup differs from
  // the real controller_server in a way that's worth finding before trusting
  // any further ASan results from this harness.
  bool spam_enabled = (std::getenv("REPRO_HARNESS_NO_PARAM_SPAM") == nullptr);
  fprintf(
    stderr, "[repro_harness] param-spam thread %s\n",
    spam_enabled ? "ENABLED" : "DISABLED (control experiment)");
  std::thread param_spam_thread([&]() {
      bool toggle = false;
      while (g_running.load() && spam_enabled) {
        toggle = !toggle;
        node->set_parameters(
        {rclcpp::Parameter("FollowPath.gamma", toggle ? 0.015 : 0.016)});
        param_changes.fetch_add(1, std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

  // Status ticker so a human watching the log can see it's alive and how
  // far it got before any crash.
  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    fprintf(
      stderr, "[repro_harness] alive: control_iterations=%lu param_changes=%lu\n",
      control_iterations.load(), param_changes.load());
  }

  control_thread.join();
  param_spam_thread.join();
  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}
