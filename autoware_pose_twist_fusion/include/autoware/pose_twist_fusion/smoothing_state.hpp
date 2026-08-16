// Copyright 2026 The Autoware Contributors
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

#ifndef AUTOWARE__POSE_TWIST_FUSION__SMOOTHING_STATE_HPP_
#define AUTOWARE__POSE_TWIST_FUSION__SMOOTHING_STATE_HPP_

#include <gtsam/geometry/Pose3.h>

#include <cstdint>

namespace autoware::pose_twist_fusion::detail
{

/// Private member type of OutputStage (D37 / D43).
///
/// It is owned exclusively by the output callback group; neither the Graph2 worker
/// nor any sensor callback may touch it, which is what removes the lost-update
/// hazard that a shared smoothing state would have.
struct SmoothingState
{
  enum class Mode : std::uint8_t { kIdle, kSmoothing };

  Mode mode{Mode::kIdle};
  /// Correction offset: X_published = X_internal . Expmap(c). GTSAM order [omega; nu].
  gtsam::Vector6 c{gtsam::Vector6::Zero()};
  /// Latched correction rate; u = -c0 / T_apply, therefore always parallel to c
  /// which makes `c <- c + u dt` exact on the Lie group (D32).
  gtsam::Vector6 u{gtsam::Vector6::Zero()};
  /// Remaining absorption time [s].
  double t_left{0.0};
  /// |c| at the moment the current smoothing was latched (relatch decision).
  double latched_norm{0.0};

  void clear()
  {
    mode = Mode::kIdle;
    c.setZero();
    u.setZero();
    t_left = 0.0;
    latched_norm = 0.0;
  }
};

}  // namespace autoware::pose_twist_fusion::detail

#endif  // AUTOWARE__POSE_TWIST_FUSION__SMOOTHING_STATE_HPP_
