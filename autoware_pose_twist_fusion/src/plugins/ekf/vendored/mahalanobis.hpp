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
// Vendored from autoware_ekf_localizer (src/mahalanobis.hpp) without algorithmic changes.
// Only the namespace (autoware::ekf_localizer -> autoware::pose_twist_fusion::ekf_vendored),
// the include paths and the header guards were adjusted so that both packages can be
// loaded into the same process. See design_last.md §5.1.

#ifndef EKF_VENDORED__MAHALANOBIS_HPP_
#define EKF_VENDORED__MAHALANOBIS_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace autoware::pose_twist_fusion::ekf_vendored
{

double squared_mahalanobis(
  const Eigen::VectorXd & x, const Eigen::VectorXd & y, const Eigen::MatrixXd & C);

double mahalanobis(const Eigen::VectorXd & x, const Eigen::VectorXd & y, const Eigen::MatrixXd & C);

}  // namespace autoware::pose_twist_fusion::ekf_vendored

#endif  // EKF_VENDORED__MAHALANOBIS_HPP_
