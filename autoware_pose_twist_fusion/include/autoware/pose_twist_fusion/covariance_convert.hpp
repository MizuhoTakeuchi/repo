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

#ifndef AUTOWARE__POSE_TWIST_FUSION__COVARIANCE_CONVERT_HPP_
#define AUTOWARE__POSE_TWIST_FUSION__COVARIANCE_CONVERT_HPP_

#include <gtsam/geometry/Pose3.h>

#include <array>

namespace autoware::pose_twist_fusion::covariance_convert
{

using RosCovariance = std::array<double, 36>;

/// Permutation Pi that maps a ROS ordered tangent vector [x,y,z,rx,ry,rz]
/// to the GTSAM order [omega; nu] (design §5.3.5).
gtsam::Matrix6 permutation();

/// ROS pose covariance -> GTSAM body tangent covariance of a pose whose rotation is `R`.
///   P_gtsam = blkdiag(R^T,R^T) . Pi . P_ros . Pi^T . blkdiag(R,R)
gtsam::Matrix6 ros_pose_to_gtsam(const RosCovariance & ros_cov, const gtsam::Rot3 & rotation);

/// Inverse of ros_pose_to_gtsam.
///   P_ros = Pi^T . blkdiag(R,R) . P_gtsam . blkdiag(R^T,R^T) . Pi
RosCovariance gtsam_pose_to_ros(const gtsam::Matrix6 & gtsam_cov, const gtsam::Rot3 & rotation);

/// Twist covariance is already expressed in base_link, so only the ordering
/// changes: P_twist,ros = Pi^T . M . Pi   ([omega;nu] -> [nu;omega], design §5.3.5).
RosCovariance gtsam_twist_to_ros(const gtsam::Matrix6 & gtsam_cov);

/// Inverse of gtsam_twist_to_ros: M = Pi . P_ros . Pi^T.
gtsam::Matrix6 ros_twist_to_gtsam(const RosCovariance & ros_cov);

/// (P + P^T) / 2.
gtsam::Matrix6 symmetrize(const gtsam::Matrix6 & p);

/// Eigen decomposition with negative eigenvalues clipped to `eps` (design §5.3.9 item 6).
/// `clipped` (optional) is set when at least one eigenvalue had to be raised.
gtsam::Matrix6 make_psd(const gtsam::Matrix6 & p, double eps = 1.0e-12, bool * clipped = nullptr);

/// Symmetrize + PSD clip + NaN/Inf replacement, applied right before publishing so
/// that design §3.3 ("never publish zero / NaN covariance") holds for every mode.
gtsam::Matrix6 sanitize(
  const gtsam::Matrix6 & p, double undetermined_variance = 1.0e4, bool * repaired = nullptr);

bool is_finite(const gtsam::Matrix6 & p);
bool is_all_zero(const RosCovariance & ros_cov);
bool is_finite(const RosCovariance & ros_cov);

/// Inverse right Jacobian J_r^{-1}(xi) of SE(3) (D40). Used for transporting the
/// internal pose covariance to the published reference: J = J_r^{-1}(-c).
gtsam::Matrix6 right_jacobian_inverse(const gtsam::Vector6 & xi);

/// Ad_{C^-1} with C = Expmap(c) (D40, twist transport).
gtsam::Matrix6 adjoint_of_inverse(const gtsam::Vector6 & c);

}  // namespace autoware::pose_twist_fusion::covariance_convert

#endif  // AUTOWARE__POSE_TWIST_FUSION__COVARIANCE_CONVERT_HPP_
