// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <chrono>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{

void MPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  last_time_called_ = clock_->now();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);
  getParam(reset_period_, "reset_period", 1.0);
  getParam(debug_path_pipeline_, "debug_path_pipeline", false);

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  path_handler_.setDebugPathPipeline(debug_path_pipeline_);
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  // perception -> MPPI path pipeline 진단용, debug_path_pipeline_일 때만
  // 생성(controller.hpp 멤버 주석 참고) -- 기본(false)이면 publisher 자체가
  // null이라 setPlan()에서 오버헤드가 전혀 없다.
  if (debug_path_pipeline_) {
    setplan_debug_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      "/debug/mppi_setplan_path", 1);
  }

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  setplan_debug_pub_.reset();
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  trajectory_visualizer_.on_activate();
  if (setplan_debug_pub_) {
    setplan_debug_pub_->on_activate();
  }
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  trajectory_visualizer_.on_deactivate();
  if (setplan_debug_pub_) {
    setplan_debug_pub_->on_deactivate();
  }
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  optimizer_.reset();
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif

  // doldrive_ws fix (2026-08-08): param_lock used to be acquired AFTER the
  // periodic reset() call below, leaving that reset() (and the array
  // reallocation inside optimizer_.reset()) completely unprotected against
  // ParametersHandler::dynamicParamsCallback running concurrently on the
  // executor thread. Acquire the lock first so every reset() path -- this
  // periodic one and the parameter-triggered one (see Optimizer::needs_reset_)
  // -- is consistently serialized against parameter changes.
  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());

  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
    reset();
  }
  last_time_called_ = clock_->now();
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker);

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
#endif

  if (visualize_) {
    visualize(std::move(transformed_plan));
  }

  return cmd;
}

void MPPIController::visualize(nav_msgs::msg::Path transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  if (debug_path_pipeline_) {
    // step 3: FollowPath/controller_server를 통해 실제 setPlan()에 들어온
    // path -- path_handler_.setPath() 호출 전, 재계산 없이 그대로.
    ++setplan_call_count_;
    const rclcpp::Time now = clock_->now();
    double dt_sec = 0.0;
    if (setplan_call_count_ > 1) {
      dt_sec = (now - last_setplan_time_).seconds();
    }
    last_setplan_time_ = now;

    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "[debug_path_pipeline] setPlan() call #%zu, dt_since_prev=%.3fs (~%.1fHz), "
      "frame_id='%s' poses=%zu",
      setplan_call_count_, dt_sec, dt_sec > 1e-6 ? 1.0 / dt_sec : 0.0,
      path.header.frame_id.c_str(), path.poses.size());

    if (setplan_debug_pub_ && setplan_debug_pub_->get_subscription_count() > 0) {
      setplan_debug_pub_->publish(path);
    }
  }

  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
