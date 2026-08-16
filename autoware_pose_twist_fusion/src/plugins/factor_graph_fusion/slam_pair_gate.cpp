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

#include "plugins/factor_graph_fusion/slam_pair_gate.hpp"

#include <cmath>
#include <utility>

namespace autoware::pose_twist_fusion::fg
{

SlamPairGate::SlamPairGate(
  const SlamParams & params, std::shared_ptr<CovarianceCorrector> corrector)
: params_(params), corrector_(std::move(corrector))
{
}

void SlamPairGate::reset()
{
  has_previous_ = false;
}

SlamPairGate::PairResult SlamPairGate::evaluate(
  double stamp, const gtsam::Pose3 & odom_pose, const gtsam::Matrix6 & odom_covariance,
  const gtsam::Pose3 & delta_graph1, const gtsam::Matrix6 & sigma_relative)
{
  PairResult result;
  if (!has_previous_) {
    has_previous_ = true;
    previous_stamp_ = stamp;
    previous_pose_ = odom_pose;
    previous_covariance_ = odom_covariance;
    return result;  // the first sample only becomes the start point of a pair
  }

  const double dt = stamp - previous_stamp_;
  const gtsam::Pose3 relative = previous_pose_.inverse() * odom_pose;

  gtsam::Matrix6 sigma_delta;
  if (params_.covariance_is_relative) {
    // Best case: the SLAM front end reports the relative increment covariance.
    sigma_delta = odom_covariance;
  } else {
    // Independence assumption. It is *not* guaranteed to be conservative: it only is
    // when sym(Ad^-1 P_12) >= 0 (D33). The loop-closure case where it fails is
    // handled by the jump detection below.
    const gtsam::Matrix6 adjoint_inverse = relative.inverse().AdjointMap();
    sigma_delta =
      adjoint_inverse * previous_covariance_ * adjoint_inverse.transpose() + odom_covariance;
  }
  sigma_delta *= params_.relative_covariance_scale * params_.covariance_scale;

  const gtsam::Vector6 innovation = gtsam::Pose3::Logmap(delta_graph1.inverse() * relative);

  CovarianceCorrector::Input input;
  input.dof = 6;
  input.innovation = innovation;
  input.measurement_covariance = sigma_delta;
  input.innovation_covariance = sigma_relative + sigma_delta;
  const CovarianceCorrector::Result correction = corrector_->correct(input);

  result.has_pair = true;
  result.previous_stamp = previous_stamp_;
  result.relative = relative;
  result.d2 = correction.d2;
  result.weight = correction.weight;
  result.covariance = correction.corrected_covariance;
  result.accepted = correction.accepted;
  result.rejected_chi2 = !correction.accepted;

  // Backup: a dynamic upper bound catches a SLAM front end that over-reports its
  // own covariance (which would make d^2 small no matter how large the jump is).
  const double translation = relative.translation().norm();
  const double rotation = gtsam::Rot3::Logmap(relative.rotation()).norm();
  const bool translation_exceeded =
    translation > params_.max_speed_mps * std::abs(dt) + params_.jump_margin_m;
  const bool rotation_exceeded =
    rotation > params_.max_yaw_rate_rps * std::abs(dt) + params_.jump_margin_rad;
  if (translation_exceeded || rotation_exceeded) {
    result.accepted = false;
    result.rejected_dynamic = true;
  }

  if (result.rejected_chi2) {
    ++chi2_reject_count_;
  }
  if (result.rejected_dynamic) {
    ++dynamic_reject_count_;
  }

  previous_stamp_ = stamp;
  previous_pose_ = odom_pose;
  previous_covariance_ = odom_covariance;
  return result;
}

}  // namespace autoware::pose_twist_fusion::fg
