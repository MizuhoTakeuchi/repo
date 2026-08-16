// Copyright 2022 Autoware Foundation
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
//
// Vendored from autoware_ekf_localizer (src/measurement.hpp) without algorithmic changes.
// Only the namespace (autoware::ekf_localizer -> autoware::pose_twist_fusion::ekf_vendored),
// the include paths and the header guards were adjusted so that both packages can be
// loaded into the same process. See design_last.md §5.1.

#ifndef EKF_VENDORED__MEASUREMENT_HPP_
#define EKF_VENDORED__MEASUREMENT_HPP_

#include <Eigen/Core>

namespace autoware::pose_twist_fusion::ekf_vendored
{

Eigen::Matrix<double, 3, 6> pose_measurement_matrix();
Eigen::Matrix<double, 2, 6> twist_measurement_matrix();
Eigen::Matrix3d pose_measurement_covariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step);
Eigen::Matrix2d twist_measurement_covariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step);

}  // namespace autoware::pose_twist_fusion::ekf_vendored

#endif  // EKF_VENDORED__MEASUREMENT_HPP_
