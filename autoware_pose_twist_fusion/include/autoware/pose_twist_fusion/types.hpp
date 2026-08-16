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

#ifndef AUTOWARE__POSE_TWIST_FUSION__TYPES_HPP_
#define AUTOWARE__POSE_TWIST_FUSION__TYPES_HPP_

#include <gtsam/geometry/Pose3.h>

#include <cstdint>

namespace autoware::pose_twist_fusion
{

/// Generation counter of the internal estimate. It is incremented every time the
/// estimator installs a new anchor (design_last.md §5.3.7 (6) / D37). `OutputStage`
/// detects anchor updates by comparing it, and never by looking at poses.
using EstimateGeneration = std::uint64_t;

/// Estimate as produced by a fusion plugin. **No smoothing is applied** (D43).
///
/// This is the type that gates / NIS / NEES / Graph2 initial values must use (D27).
/// It is deliberately a different type from PublishedEstimate and there is no
/// implicit conversion between the two (INV-4 / P3).
struct InternalEstimate
{
  /// ROS time in seconds the estimate refers to.
  double stamp{0.0};
  bool valid{false};

  /// map <- base_link.
  gtsam::Pose3 pose;
  /// Pose that still contains the estimated yaw bias (ekf mode). Equal to `pose`
  /// for the 6-DoF modes, which do not model a yaw bias (design §3.2).
  gtsam::Pose3 biased_pose;
  /// map frame translational velocity of base_link.
  gtsam::Vector3 velocity_world{gtsam::Vector3::Zero()};
  /// base_link frame angular velocity (gyro minus bias, rotated to base_link).
  gtsam::Vector3 angular_velocity_body{gtsam::Vector3::Zero()};

  /// Pose covariance in the GTSAM body tangent of `pose`, order [omega; nu] (§5.3.5).
  gtsam::Matrix6 pose_cov{gtsam::Matrix6::Identity()};
  /// Twist covariance in the GTSAM body tangent order [omega; nu], base_link frame.
  gtsam::Matrix6 twist_cov{gtsam::Matrix6::Identity()};

  /// Estimated yaw bias [rad] (0 for the 6-DoF modes, design §3.2).
  double yaw_bias{0.0};

  /// z / roll / pitch replacement values for `output.flatten_to_2d` (design §3.3).
  bool has_flat_reference{false};
  double flat_z{0.0};
  double flat_roll{0.0};
  double flat_pitch{0.0};
};

/// Estimate that is actually published (all topics and TF, design §3.1).
///
/// It can only be created by OutputStage out of an InternalEstimate and the
/// smoothing offset `c`; it deliberately has no public constructor and no
/// independently settable pose, which is how INV-4 is enforced by the type system.
class PublishedEstimate
{
public:
  PublishedEstimate() = default;

  [[nodiscard]] bool valid() const { return valid_; }
  [[nodiscard]] double stamp() const { return stamp_; }
  [[nodiscard]] const gtsam::Pose3 & pose() const { return pose_; }
  [[nodiscard]] const gtsam::Pose3 & biased_pose() const { return biased_pose_; }
  /// base_link frame linear velocity.
  [[nodiscard]] const gtsam::Vector3 & linear_velocity_body() const { return linear_body_; }
  [[nodiscard]] const gtsam::Vector3 & angular_velocity_body() const { return angular_body_; }
  /// MSE matrix (covariance + bias*bias^T) in GTSAM body tangent order [omega; nu].
  [[nodiscard]] const gtsam::Matrix6 & pose_mse() const { return pose_mse_; }
  [[nodiscard]] const gtsam::Matrix6 & twist_mse() const { return twist_mse_; }
  [[nodiscard]] double yaw_bias() const { return yaw_bias_; }
  [[nodiscard]] bool smoothing_active() const { return smoothing_active_; }
  /// Remaining smoothing offset |c| (translation / rotation) for diagnostics.
  [[nodiscard]] double smoothing_offset_lin() const { return smoothing_offset_lin_; }
  [[nodiscard]] double smoothing_offset_rot() const { return smoothing_offset_rot_; }
  [[nodiscard]] bool has_flat_reference() const { return has_flat_reference_; }
  [[nodiscard]] double flat_z() const { return flat_z_; }
  [[nodiscard]] double flat_roll() const { return flat_roll_; }
  [[nodiscard]] double flat_pitch() const { return flat_pitch_; }

private:
  friend class OutputStage;

  bool valid_{false};
  double stamp_{0.0};
  gtsam::Pose3 pose_;
  gtsam::Pose3 biased_pose_;
  gtsam::Vector3 linear_body_{gtsam::Vector3::Zero()};
  gtsam::Vector3 angular_body_{gtsam::Vector3::Zero()};
  gtsam::Matrix6 pose_mse_{gtsam::Matrix6::Identity()};
  gtsam::Matrix6 twist_mse_{gtsam::Matrix6::Identity()};
  double yaw_bias_{0.0};
  bool smoothing_active_{false};
  double smoothing_offset_lin_{0.0};
  double smoothing_offset_rot_{0.0};
  bool has_flat_reference_{false};
  double flat_z_{0.0};
  double flat_roll_{0.0};
  double flat_pitch_{0.0};
};

}  // namespace autoware::pose_twist_fusion

#endif  // AUTOWARE__POSE_TWIST_FUSION__TYPES_HPP_
