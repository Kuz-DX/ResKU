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

#include "nav2_mppi_controller/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <random>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic,
  const std::string & name, ParametersHandler * param_handler)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;

  // [2026-08-17] xt::random::get_default_random_engine()은 시드 없이
  // 기본생성된 std::mt19937이라, C++ 표준상 항상 고정 시드(5489)에서
  // 시작한다 -- 이 패키지 어디에도 xt::random::seed()를 부르는 코드가
  // 없어서, controller_server를 새로 켤 때마다 매번 "무작위" 노이즈가
  // 토씨 하나 안 틀리고 똑같이 재현됐음. batch_size(1000)짜리 유한 표본은
  // 평균이 정확히 0이 아닐 수 있는데, 시드가 고정이면 그 우연한 치우침이
  // 매 실행마다 재현되어 마치 진짜 시스템 편향인 것처럼 보임 -- 실기
  // 디버깅(2026-08-17)에서 완전 직진 경로를 줘도 angular.z가 재시작
  // 직후부터 곧바로 같은 방향으로 쏠리는 걸 반복 확인, critic/커널매틱스
  // 코드는 전부 대칭인데도 편향이 있었던 이유가 이거였을 가능성이 높음.
  // 실제 엔트로피로 재시드해서 실행마다 다른 시퀀스를 쓰게 한다.
  std::random_device rd;
  xt::random::seed(rd());

  auto getParam = param_handler->getParamGetter(name);
  getParam(regenerate_noises_, "regenerate_noises", false);

  if (regenerate_noises_) {
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::shutdown()
{
  active_ = false;
  ready_ = true;
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);

  xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;
  xt::noalias(state.cvy) = control_sequence.vy + noises_vy_;
  xt::noalias(state.cwz) = control_sequence.wz + noises_wz_;
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  // doldrive_ws fix (2026-08-08): settings_ = settings (and is_holonomic_)
  // used to happen HERE, outside noise_lock_, while noiseThread() reads
  // settings_ (via generateNoisedControls()) UNDER noise_lock_ on its own
  // dedicated background thread. That's an unprotected struct-tearing race:
  // settings_ has multiple fields (batch_size, time_steps, ...) and a
  // concurrent read during a partial write can see a mix of old/new values
  // -- e.g. new batch_size with old time_steps -- producing a
  // wrongly-shaped noises_vx_/noises_wz_/noises_vy_ that then propagates
  // into state_.cvx/cwz via setNoisedControls(), causing a shape mismatch
  // against costs_ in Optimizer::updateControlSequence() (reproducibly a
  // SIGSEGV in an AVX store, confirmed via coredump backtrace -- unrelated
  // to the separate parameter-callback race fixed elsewhere). Move both
  // assignments inside the noise_lock_-protected block so every read of
  // settings_ (this function and generateNoisedControls()) is consistently
  // serialized against every write to it.
  std::unique_lock<std::mutex> guard(noise_lock_);
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  xt::noalias(noises_vx_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
  xt::noalias(noises_vy_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
  xt::noalias(noises_wz_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
  ready_ = true;

  if (regenerate_noises_) {
    guard.unlock();
    noise_cond_.notify_all();
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::noiseThread()
{
  do {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this]() {return ready_;});
    ready_ = false;
    generateNoisedControls();
  } while (active_);
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;

  xt::noalias(noises_vx_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.vx);
  xt::noalias(noises_wz_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.wz);
  if (is_holonomic_) {
    xt::noalias(noises_vy_) = xt::random::randn<float>(
      {s.batch_size, s.time_steps}, 0.0f,
      s.sampling_std.vy);
  }
}

}  // namespace mppi
