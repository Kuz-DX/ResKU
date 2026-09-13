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

#include "nav2_mppi_controller/critics/prefer_forward_critic.hpp"

#include <limits>

namespace mppi::critics
{

void PreferForwardCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5);
  // [2026-08-25] 기본값 0.0 = 기존 동작(후진만 벌점)과 완전히 동일.
  getParam(min_desired_vx_, "min_desired_vx", 0.0);
  getParam(
    max_path_deviation_to_penalize_m_,
    "max_path_deviation_to_penalize_m", 3.0);

  RCLCPP_INFO(
    logger_,
    "PreferForwardCritic instantiated with %d power, %f weight, "
    "min_desired_vx=%f, max_path_deviation_to_penalize_m=%f.",
    power_, weight_, min_desired_vx_, max_path_deviation_to_penalize_m_);
}

void PreferForwardCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_ ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  // [2026-08-25] "정지 방지" 벌점을 적용하기 전에, 로봇이 경로에서 너무
  // 많이 벗어난 비정상 상황(예: 슬립/충돌로 경로를 크게 이탈)인지 먼저
  // 확인한다. 그런 상황에서는 "일단 서서 상황 보정"이 정답일 수 있으므로,
  // 이 벌점이 그 판단을 방해하지 않도록 min_desired_vx_ 대신 0.0(=기존
  // 후진-only 동작)으로 폴백한다. 거리 계산은 PathAlignCritic과 달리
  // furthest_reached_path_point 없이, 로봇 현재 위치 기준 경로 전체에서
  // 가장 가까운 점까지의 단순 최근접 거리로 계산(이 critic 목적엔 충분).
  float effective_min_vx = min_desired_vx_;
  if (min_desired_vx_ > 0.0f) {
    const auto & path_x = data.path.x;
    const auto & path_y = data.path.y;
    const float robot_x = static_cast<float>(data.state.pose.pose.position.x);
    const float robot_y = static_cast<float>(data.state.pose.pose.position.y);
    float min_dist_sq = std::numeric_limits<float>::max();
    for (size_t i = 0; i < path_x.shape(0); ++i) {
      const float dx = path_x(i) - robot_x;
      const float dy = path_y(i) - robot_y;
      const float dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
      }
    }
    const float max_dev_sq =
      max_path_deviation_to_penalize_m_ * max_path_deviation_to_penalize_m_;
    if (min_dist_sq > max_dev_sq) {
      effective_min_vx = 0.0f;
    }
  }

  auto backward_motion = xt::maximum(effective_min_vx - data.state.vx, 0);
  data.costs += xt::pow(
    xt::sum(
      std::move(
        backward_motion) * data.model_dt, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PreferForwardCritic,
  mppi::critics::CriticFunction)
