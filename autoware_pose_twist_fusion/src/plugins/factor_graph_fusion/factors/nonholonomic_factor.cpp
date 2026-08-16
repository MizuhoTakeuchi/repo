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

#include "plugins/factor_graph_fusion/factors/nonholonomic_factor.hpp"

#include <gtsam/base/Matrix.h>

namespace autoware::pose_twist_fusion::fg
{

gtsam::Vector NonHolonomicFactor::evaluateError(
  const gtsam::Pose3 & pose, const gtsam::Vector3 & velocity,
  boost::optional<gtsam::Matrix &> h_pose, boost::optional<gtsam::Matrix &> h_velocity) const
{
  const gtsam::Matrix3 rotation = pose.rotation().matrix();
  const gtsam::Vector3 body_velocity = rotation.transpose() * velocity;

  if (h_pose) {
    gtsam::Matrix jacobian = gtsam::Matrix::Zero(2, 6);
    jacobian.block<2, 3>(0, 0) = gtsam::skewSymmetric(body_velocity).bottomRows<2>();
    *h_pose = jacobian;
  }
  if (h_velocity) {
    gtsam::Matrix jacobian = gtsam::Matrix::Zero(2, 3);
    jacobian.block<2, 3>(0, 0) = rotation.transpose().bottomRows<2>();
    *h_velocity = jacobian;
  }
  return (gtsam::Vector(2) << body_velocity.y(), body_velocity.z()).finished();
}

}  // namespace autoware::pose_twist_fusion::fg
