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

#include "plugins/factor_graph_fusion/factors/wheel_speed_factor.hpp"

#include <gtsam/base/Matrix.h>

namespace autoware::pose_twist_fusion::fg
{

gtsam::Vector WheelSpeedFactor::evaluateError(
  const gtsam::Pose3 & pose, const gtsam::Vector3 & velocity,
  boost::optional<gtsam::Matrix &> h_pose, boost::optional<gtsam::Matrix &> h_velocity) const
{
  const gtsam::Matrix3 rotation = pose.rotation().matrix();
  const gtsam::Vector3 body_velocity = rotation.transpose() * velocity;

  if (h_pose) {
    gtsam::Matrix jacobian = gtsam::Matrix::Zero(1, 6);
    jacobian.block<1, 3>(0, 0) = gtsam::skewSymmetric(body_velocity).row(0);
    *h_pose = jacobian;
  }
  if (h_velocity) {
    gtsam::Matrix jacobian = gtsam::Matrix::Zero(1, 3);
    jacobian.block<1, 3>(0, 0) = rotation.transpose().row(0);
    *h_velocity = jacobian;
  }
  return (gtsam::Vector(1) << body_velocity.x() - measured_vx_).finished();
}

}  // namespace autoware::pose_twist_fusion::fg
