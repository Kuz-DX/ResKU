// Copyright (c) 2023 Open Navigation LLC
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

#include "nav2_mppi_controller/critics/path_align_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void PathAlignCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0);

  getParam(max_path_occupancy_ratio_, "max_path_occupancy_ratio", 0.07);
  getParam(offset_from_furthest_, "offset_from_furthest", 20);
  getParam(trajectory_point_step_, "trajectory_point_step", 4);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5);
  getParam(use_path_orientations_, "use_path_orientations", false);

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
    power_, weight_);
}

void PathAlignCritic::score(CriticData & data)
{
  // Don't apply close to goal, let the goal critics take over
  if (!enabled_ ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  // Don't apply when first getting bearing w.r.t. the path
  utils::setPathFurthestPointIfNotSet(data);
  const size_t path_segments_count = *data.furthest_reached_path_point;  // up to furthest only

  // [계측, 2026-08-25] 회전 구간에서 critic이 회전을 아예 못 보는 게
  // offset_from_furthest(켜짐/꺼짐 문턱) 때문인지, 아니면 후보 궤적이
  // 물리적으로 회전 시작점까지 못 뻗어서(path_segments_count 자체가
  // 짧아서) 비교 대상이 회전 전 직선 구간에 묶여있는 건지 구분하기 위한
  // 진단용 로그. 함수 로직/비용 계산에는 관여하지 않음(순수 read-only) --
  // ~1초(20Hz 기준 20회)마다 한 번만 찍어서 스팸 방지.
  {
    static long score_calls = 0;
    if (++score_calls % 20 == 0) {
      const auto P_x_dbg = xt::view(data.path.x, xt::range(_, -1));
      const auto P_y_dbg = xt::view(data.path.y, xt::range(_, -1));
      float reach_m = 0.0f;
      for (size_t i = 1; i < path_segments_count && i < P_x_dbg.size(); ++i) {
        const float dxr = P_x_dbg(i) - P_x_dbg(i - 1);
        const float dyr = P_y_dbg(i) - P_y_dbg(i - 1);
        reach_m += sqrtf(dxr * dxr + dyr * dyr);
      }
      RCLCPP_INFO(
        logger_,
        "[PATH_ALIGN_REACH] path_segments_count=%zu offset_from_furthest=%zu "
        "reach_m=%.3f gated_off=%s",
        path_segments_count, offset_from_furthest_, reach_m,
        (path_segments_count < offset_from_furthest_) ? "true" : "false");
    }
  }

  if (path_segments_count < offset_from_furthest_) {
    return;
  }

  // Don't apply when dynamic obstacles are blocking significant proportions of the local path
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  const size_t closest_initial_path_point = utils::findPathTrajectoryInitialPoint(data);
  unsigned int invalid_ctr = 0;
  const float range = *data.furthest_reached_path_point - closest_initial_path_point;
  for (size_t i = closest_initial_path_point; i < *data.furthest_reached_path_point; i++) {
    if (!(*data.path_pts_valid)[i]) {invalid_ctr++;}
    if (static_cast<float>(invalid_ctr) / range > max_path_occupancy_ratio_ && invalid_ctr > 2) {
      return;
    }
  }

  const auto P_x = xt::view(data.path.x, xt::range(_, -1));  // path points
  const auto P_y = xt::view(data.path.y, xt::range(_, -1));  // path points
  const auto P_yaw = xt::view(data.path.yaws, xt::range(_, -1));  // path points

  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);
  auto && cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});

  // Find integrated distance in the path
  std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
  float dx = 0.0f, dy = 0.0f;
  for (unsigned int i = 1; i != path_segments_count; i++) {
    dx = P_x(i) - P_x(i - 1);
    dy = P_y(i) - P_y(i - 1);
    float curr_dist = sqrtf(dx * dx + dy * dy);
    path_integrated_distances[i] = path_integrated_distances[i - 1] + curr_dist;
  }

  float traj_integrated_distance = 0.0f;
  float summed_path_dist = 0.0f, dyaw = 0.0f;
  float num_samples = 0.0f;
  float Tx = 0.0f, Ty = 0.0f;
  size_t path_pt = 0;
  for (size_t t = 0; t < batch_size; ++t) {
    traj_integrated_distance = 0.0f;
    summed_path_dist = 0.0f;
    num_samples = 0.0f;
    path_pt = 0u;
    const auto T_x = xt::view(data.trajectories.x, t, xt::all());
    const auto T_y = xt::view(data.trajectories.y, t, xt::all());
    for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
      Tx = T_x(p);
      Ty = T_y(p);
      dx = Tx - T_x(p - trajectory_point_step_);
      dy = Ty - T_y(p - trajectory_point_step_);
      traj_integrated_distance += sqrtf(dx * dx + dy * dy);
      path_pt = utils::findClosestPathPt(
        path_integrated_distances, traj_integrated_distance, path_pt);

      // The nearest path point to align to needs to be not in collision, else
      // let the obstacle critic take over in this region due to dynamic obstacles
      if ((*data.path_pts_valid)[path_pt]) {
        dx = P_x(path_pt) - Tx;
        dy = P_y(path_pt) - Ty;
        num_samples += 1.0f;
        if (use_path_orientations_) {
          const auto T_yaw = xt::view(data.trajectories.yaws, t, xt::all());
          dyaw = angles::shortest_angular_distance(P_yaw(path_pt), T_yaw(p));
          summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
        } else {
          summed_path_dist += sqrtf(dx * dx + dy * dy);
        }
      }
    }
    if (num_samples > 0) {
      cost[t] = summed_path_dist / num_samples;
    } else {
      cost[t] = 0.0f;
    }
  }

  data.costs += xt::pow(std::move(cost) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)
