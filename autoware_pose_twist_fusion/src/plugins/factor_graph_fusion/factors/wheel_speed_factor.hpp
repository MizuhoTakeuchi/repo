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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__WHEEL_SPEED_FACTOR_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__WHEEL_SPEED_FACTOR_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace autoware::pose_twist_fusion::fg
{

/// Wheel speed measurement model of design §5.3.3a (D11).
///
///   prediction  v_hat_x = e_x^T . R(X)^T . V
///   residual    r = v_hat_x - z_v                       (1 dimensional)
///   jacobians   dr/dX = [ e_x^T skew(R^T V) , 0_{1x3} ] (GTSAM tangent [omega; nu])
///               dr/dV = e_x^T R^T
class WheelSpeedFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>
{
public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>;

  WheelSpeedFactor(
    gtsam::Key pose_key, gtsam::Key velocity_key, double measured_vx,
    const gtsam::SharedNoiseModel & model)
  : Base(model, pose_key, velocity_key), measured_vx_(measured_vx)
  {
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose, const gtsam::Vector3 & velocity,
    boost::optional<gtsam::Matrix &> h_pose = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none) const override;

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new WheelSpeedFactor(*this)));
  }

  [[nodiscard]] double measured() const { return measured_vx_; }

private:
  double measured_vx_;
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__WHEEL_SPEED_FACTOR_HPP_
