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

#include "plugins/factor_graph_fusion/covariance_corrector.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace autoware::pose_twist_fusion::fg
{

CovarianceCorrector::CovarianceCorrector(
  const CovarianceCorrectionParams & params, std::string robust_kernel,
  double robust_kernel_param)
: params_(params), robust_kernel_(std::move(robust_kernel)), robust_kernel_param_(robust_kernel_param)
{
}

double CovarianceCorrector::gate_threshold(int dof, bool recovery) const
{
  double threshold = params_.gate_chi2.dim6;
  if (dof == 1) {
    threshold = params_.gate_chi2.dim1;
  } else if (dof == 3) {
    threshold = params_.gate_chi2.dim3;
  }
  return recovery ? threshold * params_.recovery_gate_scale : threshold;
}

double CovarianceCorrector::weight_of(double d2, int dof) const
{
  const double excess = std::max(0.0, d2 - static_cast<double>(dof));
  double w = 1.0;
  if (params_.weight_function == "inverse_nis") {
    w = static_cast<double>(dof) / std::max(static_cast<double>(dof), d2);
  } else if (params_.weight_function == "inverse_linear") {
    w = 1.0 / (1.0 + excess);
  } else {
    // exp_excess_nis (default): only the "surprise" beyond E[d^2] = dof is punished.
    w = std::exp(-0.5 * excess);
  }
  return std::clamp(w, params_.w_min, 1.0);
}

CovarianceCorrector::Result CovarianceCorrector::correct(const Input & input) const
{
  Result result;
  result.corrected_covariance = input.measurement_covariance;

  const Eigen::MatrixXd s = 0.5 * (input.innovation_covariance + input.innovation_covariance.transpose());
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(s);
  if (ldlt.info() != Eigen::Success) {
    // Layer 0: a non-decomposable innovation covariance means the input cannot be
    // judged at all; drop it rather than pretend it passed.
    result.accepted = false;
    result.gate_rejected = true;
    result.d2 = std::numeric_limits<double>::infinity();
    return result;
  }
  const Eigen::VectorXd solved = ldlt.solve(input.innovation);
  result.d2 = input.innovation.dot(solved);
  if (!std::isfinite(result.d2) || result.d2 < 0.0) {
    result.accepted = false;
    result.gate_rejected = true;
    return result;
  }

  // Layer A: hard chi-squared gate.
  if (params_.enable_gate && result.d2 > gate_threshold(input.dof, input.recovery)) {
    result.accepted = false;
    result.gate_rejected = true;
    result.weight = params_.w_min;
    return result;
  }

  // Layer B: continuous confidence weighting, R_z' = R_z / w.
  if (params_.enable_weighting) {
    result.weight = weight_of(result.d2, input.dof);
    result.corrected_covariance = input.measurement_covariance / result.weight;
  }
  return result;
}

gtsam::SharedNoiseModel CovarianceCorrector::make_noise_model(const gtsam::Matrix & covariance) const
{
  gtsam::SharedNoiseModel base = gtsam::noiseModel::Gaussian::Covariance(covariance);
  if (robust_kernel_ == "none") {
    return base;
  }

  // Layer C (default off): a robust kernel on top of the pre-gate.
  gtsam::noiseModel::mEstimator::Base::shared_ptr estimator;
  if (robust_kernel_ == "huber") {
    estimator = gtsam::noiseModel::mEstimator::Huber::Create(robust_kernel_param_);
  } else if (robust_kernel_ == "cauchy") {
    estimator = gtsam::noiseModel::mEstimator::Cauchy::Create(robust_kernel_param_);
  } else if (robust_kernel_ == "dcs") {
    estimator = gtsam::noiseModel::mEstimator::DCS::Create(robust_kernel_param_);
  } else if (robust_kernel_ == "gm") {
    estimator = gtsam::noiseModel::mEstimator::GemanMcClure::Create(robust_kernel_param_);
  } else {
    return base;
  }
  return gtsam::noiseModel::Robust::Create(
    estimator, boost::static_pointer_cast<gtsam::noiseModel::Gaussian>(base));
}

gtsam::Matrix6 CovarianceCorrector::relax_low_speed(
  const gtsam::Matrix6 & sigma1, double speed) const
{
  if (std::abs(speed) >= params_.low_speed_relax_mps) {
    return sigma1;
  }
  gtsam::Matrix6 relaxed = sigma1;
  const double rot_floor = params_.low_speed_min_rot_stddev * params_.low_speed_min_rot_stddev;
  const double pos_floor = params_.low_speed_min_pos_stddev * params_.low_speed_min_pos_stddev;
  for (int i = 0; i < 3; ++i) {
    relaxed(i, i) = std::max(relaxed(i, i), rot_floor);
    relaxed(i + 3, i + 3) = std::max(relaxed(i + 3, i + 3), pos_floor);
  }
  return relaxed;
}

}  // namespace autoware::pose_twist_fusion::fg
