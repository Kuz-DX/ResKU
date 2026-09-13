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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for preferring forward motion
 */
class PreferForwardCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float threshold_to_consider_{0};

  // [2026-08-25 확장] "정지 방지" -- 기존엔 후진(vx<0)만 벌점을 줘서
  // vx=0(완전 정지)엔 벌점이 없었음. min_desired_vx_(기본값 0.0 -> 기존
  // 동작과 완전히 동일)를 넘겨서 이 문턱보다 느리면(정지 포함) 느릴수록
  // 벌점을 주도록 일반화. 단, 경로에서 너무 많이 벗어난 비정상 상황에서까지
  // "멈추지 마"를 강요하면 안 되므로, 로봇 현재 위치가 경로에서
  // max_path_deviation_to_penalize_m_(기본값, 넘으면 이 벌점 자체를 끔)보다
  // 멀면 min_desired_vx_ 대신 0.0(=기존 후진-only 동작)으로 자동 폴백.
  float min_desired_vx_{0.0};
  float max_path_deviation_to_penalize_m_{3.0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_
