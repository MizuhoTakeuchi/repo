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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__GNSS_LEVER_ARM_FACTOR_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__GNSS_LEVER_ARM_FACTOR_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace autoware::pose_twist_fusion::fg
{

/// GNSS position factor with lever arm (design §5.3.3b / D25).
///
///   prediction  h(X) = p + R(X) . r_bg          (antenna position in map)
///   residual    r = h(X) - z                    (3 dimensional, map frame)
///   jacobian    dr/dX = [ -R . skew(r_bg) , R ] (GTSAM tangent [omega; nu])
///
/// Pre-rotating the measurement into base_link (`z - R_hat r_bg`) is forbidden: it
/// freezes the current attitude estimate into the measurement and removes the path
/// through which GNSS constrains attitude.
///
/// With `r_bg = 0` the Jacobian degenerates to [0, R], i.e. exactly gtsam::GPSFactor,
/// so no case distinction is needed (verified by 試験 14).
class GnssLeverArmFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  GnssLeverArmFactor(
    gtsam::Key pose_key, const gtsam::Point3 & measured_position,
    const gtsam::Point3 & lever_arm, const gtsam::SharedNoiseModel & model)
  : Base(model, pose_key), measured_(measured_position), lever_arm_(lever_arm)
  {
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> h_pose = boost::none) const override;

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new GnssLeverArmFactor(*this)));
  }

  [[nodiscard]] const gtsam::Point3 & measured() const { return measured_; }
  [[nodiscard]] const gtsam::Point3 & lever_arm() const { return lever_arm_; }

private:
  gtsam::Point3 measured_;
  gtsam::Point3 lever_arm_;
};

/// Jacobian of the same measurement model, used by the layer A/B covariance
/// correction so that gate and factor share one model (design §5.3.3b).
gtsam::Matrix gnss_lever_arm_jacobian(const gtsam::Pose3 & pose, const gtsam::Point3 & lever_arm);

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__GNSS_LEVER_ARM_FACTOR_HPP_
