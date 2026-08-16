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

#include "plugins/factor_graph_fusion/factors/gnss_lever_arm_factor.hpp"

#include <gtsam/base/Matrix.h>

namespace autoware::pose_twist_fusion::fg
{

gtsam::Matrix gnss_lever_arm_jacobian(const gtsam::Pose3 & pose, const gtsam::Point3 & lever_arm)
{
  const gtsam::Matrix3 rotation = pose.rotation().matrix();
  gtsam::Matrix jacobian = gtsam::Matrix::Zero(3, 6);
  jacobian.block<3, 3>(0, 0) = -rotation * gtsam::skewSymmetric(lever_arm);
  jacobian.block<3, 3>(0, 3) = rotation;
  return jacobian;
}

gtsam::Vector GnssLeverArmFactor::evaluateError(
  const gtsam::Pose3 & pose, boost::optional<gtsam::Matrix &> h_pose) const
{
  const gtsam::Matrix3 rotation = pose.rotation().matrix();
  const gtsam::Point3 predicted = pose.translation() + rotation * lever_arm_;
  if (h_pose) {
    *h_pose = gnss_lever_arm_jacobian(pose, lever_arm_);
  }
  return predicted - measured_;
}

}  // namespace autoware::pose_twist_fusion::fg
