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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__NONHOLONOMIC_FACTOR_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__NONHOLONOMIC_FACTOR_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace autoware::pose_twist_fusion::fg
{

/// Non-holonomic constraint of design §5.3.3a: the lateral and vertical components
/// of the body velocity are approximately zero (2 dimensional residual).
/// It is not added below `nonholonomic_min_speed_mps` to avoid over-constraining a
/// standing vehicle.
class NonHolonomicFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>
{
public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>;

  NonHolonomicFactor(
    gtsam::Key pose_key, gtsam::Key velocity_key, const gtsam::SharedNoiseModel & model)
  : Base(model, pose_key, velocity_key)
  {
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose, const gtsam::Vector3 & velocity,
    boost::optional<gtsam::Matrix &> h_pose = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none) const override;

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new NonHolonomicFactor(*this)));
  }
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__NONHOLONOMIC_FACTOR_HPP_
