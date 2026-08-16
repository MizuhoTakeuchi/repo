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

#ifndef PLUGINS__LIO_SAM__LIO_SAM_COVARIANCE_HPP_
#define PLUGINS__LIO_SAM__LIO_SAM_COVARIANCE_HPP_

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>

namespace autoware::pose_twist_fusion::lio_sam
{

using Matrix15 = Eigen::Matrix<double, 15, 15>;

/// Result of the joint 15x15 forward propagation of design §5.2.1 / D39.
struct PropagatedCovariance
{
  /// [omega; nu] pose covariance (GTSAM body tangent).
  gtsam::Matrix6 pose{gtsam::Matrix6::Identity()};
  /// world frame velocity covariance.
  gtsam::Matrix3 velocity{gtsam::Matrix3::Identity()};
  /// [accel bias; gyro bias].
  gtsam::Matrix6 bias{gtsam::Matrix6::Identity()};
  /// Full 9x9 NavState block, kept for the Monte-Carlo cross check of 試験 9.
  gtsam::Matrix9 nav{gtsam::Matrix9::Identity()};
};

/// Propagates the anchor's joint covariance through the preintegration.
///
///   Sigma_nav(t) = H . Sigma_anchor . H^T + pim.preintMeasCov(),  H = [H_state H_bias]
///
/// X, V and B are propagated **jointly**: propagating them separately drops the
/// cross terms 2 dt P_pv and -dt P_vb, which under-estimates the position variance
/// in the common positively-correlated case (D33 / D39). That is why no blockwise
/// option exists.
PropagatedCovariance propagate_joint(
  const gtsam::PreintegratedImuMeasurements & pim, const gtsam::NavState & anchor_state,
  const gtsam::imuBias::ConstantBias & anchor_bias, const Matrix15 & anchor_covariance,
  double dt, double accel_bias_rw_stddev, double gyro_bias_rw_stddev);

}  // namespace autoware::pose_twist_fusion::lio_sam

#endif  // PLUGINS__LIO_SAM__LIO_SAM_COVARIANCE_HPP_
