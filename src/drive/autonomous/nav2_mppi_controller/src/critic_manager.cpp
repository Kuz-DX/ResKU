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

#include "nav2_mppi_controller/critic_manager.hpp"

namespace mppi
{

void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  getParams();
  loadCritics();
}

void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
}

void CriticManager::loadCritics()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "nav2_mppi_controller", "mppi::critics::CriticFunction");
  }

  critics_.clear();
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data) const
{
  // TEMP DEBUG (2026-08-17): 좌회전 경로를 줘도 계속 우회전 명령이 나오는
  // 원인을 찾기 위해, critic이 하나씩 돌 때마다 "순 좌회전 후보군"과
  // "순 우회전 후보군"의 평균 누적비용을 비교해서 찍는다. yaws의 마지막-처음
  // 차이(=그 후보 궤적의 순 회전량)로 좌/우를 나눔. 어느 critic이 실행된
  // 직후에 우측 평균이 좌측보다 확 낮아지는지(=그 critic이 우회전을
  // 선호하게 만드는지) 보고 원인 찾으면 이 블록은 지울 것.
  auto node = parent_.lock();
  auto log_split = [&](const char * label) {
      const auto & yaws = data.trajectories.yaws;
      const size_t batch = yaws.shape(0);
      const size_t steps = yaws.shape(1);
      double left_cost_sum = 0.0, right_cost_sum = 0.0;
      size_t left_n = 0, right_n = 0;
      for (size_t t = 0; t < batch; t++) {
        const float net_turn = yaws(t, steps - 1) - yaws(t, 0);
        const float c = data.costs(t);
        if (net_turn > 0.01f) {
          left_cost_sum += c;
          left_n++;
        } else if (net_turn < -0.01f) {
          right_cost_sum += c;
          right_n++;
        }
      }
      RCLCPP_INFO_THROTTLE(
        logger_, *node->get_clock(), 2000,
        "[TURN_BIAS_DEBUG after %s] left_n=%zu left_avg=%.4f  right_n=%zu right_avg=%.4f",
        label, left_n, left_n ? left_cost_sum / left_n : 0.0,
        right_n, right_n ? right_cost_sum / right_n : 0.0);
    };

  for (size_t q = 0; q < critics_.size(); q++) {
    if (data.fail_flag) {
      break;
    }
    critics_[q]->score(data);
    log_split(critic_names_[q].c_str());
  }
}

}  // namespace mppi
