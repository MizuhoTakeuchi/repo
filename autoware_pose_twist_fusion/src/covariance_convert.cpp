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

#include "autoware/pose_twist_fusion/covariance_convert.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace autoware::pose_twist_fusion::covariance_convert
{

namespace
{
gtsam::Matrix6 block_rotation(const gtsam::Rot3 & rotation)
{
  gtsam::Matrix6 j = gtsam::Matrix6::Zero();
  j.topLeftCorner<3, 3>() = rotation.matrix();
  j.bottomRightCorner<3, 3>() = rotation.matrix();
  return j;
}

gtsam::Matrix6 from_ros_array(const RosCovariance & ros_cov)
{
  gtsam::Matrix6 m;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      m(i, j) = ros_cov[static_cast<size_t>(i * 6 + j)];
    }
  }
  return m;
}

RosCovariance to_ros_array(const gtsam::Matrix6 & m)
{
  RosCovariance out{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      out[static_cast<size_t>(i * 6 + j)] = m(i, j);
    }
  }
  return out;
}
}  // namespace

gtsam::Matrix6 permutation()
{
  gtsam::Matrix6 p = gtsam::Matrix6::Zero();
  // v_ros = [t; theta]  ->  Pi . v_ros = [theta; t]
  p.topRightCorner<3, 3>() = gtsam::Matrix3::Identity();
  p.bottomLeftCorner<3, 3>() = gtsam::Matrix3::Identity();
  return p;
}

gtsam::Matrix6 ros_pose_to_gtsam(const RosCovariance & ros_cov, const gtsam::Rot3 & rotation)
{
  const gtsam::Matrix6 pi = permutation();
  const gtsam::Matrix6 j = block_rotation(rotation);
  const gtsam::Matrix6 p_perm = pi * from_ros_array(ros_cov) * pi.transpose();
  return j.transpose() * p_perm * j;
}

RosCovariance gtsam_pose_to_ros(const gtsam::Matrix6 & gtsam_cov, const gtsam::Rot3 & rotation)
{
  const gtsam::Matrix6 pi = permutation();
  const gtsam::Matrix6 j = block_rotation(rotation);
  const gtsam::Matrix6 p_perm = j * gtsam_cov * j.transpose();
  return to_ros_array(pi.transpose() * p_perm * pi);
}

RosCovariance gtsam_twist_to_ros(const gtsam::Matrix6 & gtsam_cov)
{
  const gtsam::Matrix6 pi = permutation();
  return to_ros_array(pi.transpose() * gtsam_cov * pi);
}

gtsam::Matrix6 ros_twist_to_gtsam(const RosCovariance & ros_cov)
{
  const gtsam::Matrix6 pi = permutation();
  return pi * from_ros_array(ros_cov) * pi.transpose();
}

gtsam::Matrix6 symmetrize(const gtsam::Matrix6 & p)
{
  return 0.5 * (p + p.transpose());
}

gtsam::Matrix6 make_psd(const gtsam::Matrix6 & p, double eps, bool * clipped)
{
  const gtsam::Matrix6 sym = symmetrize(p);
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> solver(sym);
  if (solver.info() != Eigen::Success) {
    if (clipped != nullptr) {
      *clipped = true;
    }
    return gtsam::Matrix6::Identity() * eps;
  }
  gtsam::Vector6 values = solver.eigenvalues();
  bool did_clip = false;
  for (int i = 0; i < 6; ++i) {
    if (values(i) < eps) {
      values(i) = eps;
      did_clip = true;
    }
  }
  if (clipped != nullptr) {
    *clipped = did_clip;
  }
  if (!did_clip) {
    return sym;
  }
  const gtsam::Matrix6 vectors = solver.eigenvectors();
  return vectors * values.asDiagonal() * vectors.transpose();
}

bool is_finite(const gtsam::Matrix6 & p)
{
  return p.allFinite();
}

bool is_finite(const RosCovariance & ros_cov)
{
  return std::all_of(
    ros_cov.begin(), ros_cov.end(), [](double v) { return std::isfinite(v); });
}

bool is_all_zero(const RosCovariance & ros_cov)
{
  return std::all_of(ros_cov.begin(), ros_cov.end(), [](double v) { return v == 0.0; });
}

gtsam::Matrix6 sanitize(const gtsam::Matrix6 & p, double undetermined_variance, bool * repaired)
{
  bool touched = false;
  gtsam::Matrix6 out = p;
  if (!out.allFinite()) {
    // Replace the whole matrix rather than individual entries: a NaN anywhere means
    // the estimate that produced it cannot be trusted component-wise either.
    out = gtsam::Matrix6::Identity() * undetermined_variance;
    touched = true;
  }
  bool clipped = false;
  out = make_psd(symmetrize(out), 1.0e-12, &clipped);
  touched = touched || clipped;

  // §3.3: never publish an all-zero covariance. Un-estimated dimensions get
  // `undetermined_variance` instead of 0.
  for (int i = 0; i < 6; ++i) {
    if (out(i, i) <= 0.0) {
      out(i, i) = undetermined_variance;
      touched = true;
    }
  }
  if (repaired != nullptr) {
    *repaired = touched;
  }
  return out;
}

gtsam::Matrix6 right_jacobian_inverse(const gtsam::Vector6 & xi)
{
#if defined(PTF_SMOOTHING_LOGMAP_DERIVATIVE_ARG_POSE)
  return gtsam::Pose3::LogmapDerivative(gtsam::Pose3::Expmap(xi));
#elif defined(PTF_SMOOTHING_LOGMAP_DERIVATIVE_ARG_VECTOR)
  return gtsam::Pose3::LogmapDerivative(xi);
#else
  // Fallback: J_r^{-1} = (J_r)^{-1} with J_r = ExpmapDerivative.
  return gtsam::Pose3::ExpmapDerivative(xi).inverse();
#endif
}

gtsam::Matrix6 adjoint_of_inverse(const gtsam::Vector6 & c)
{
  return gtsam::Pose3::Expmap(c).inverse().AdjointMap();
}

}  // namespace autoware::pose_twist_fusion::covariance_convert
