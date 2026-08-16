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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__COVARIANCE_CORRECTOR_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__COVARIANCE_CORRECTOR_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <string>

namespace autoware::pose_twist_fusion::fg
{

/// Layers 0 / A / B / C of design §5.3.6 - the core of requirement R11.
///
/// Scope (D42): only absolute pose inputs (GNSS / NDT / SLAM) go through layers
/// A/B/C. Wheel speed gets layer A only, and that gate lives inside Graph1.
class CovarianceCorrector
{
public:
  struct Input
  {
    int dof{6};
    /// Innovation nu in the tangent representation of §5.3.5.
    gtsam::Vector innovation;
    /// S = H Sigma_1(t) H^T + R_z (predicted innovation covariance).
    gtsam::Matrix innovation_covariance;
    /// Measurement noise R_z in the same space as the innovation.
    gtsam::Matrix measurement_covariance;
    /// Gate relaxation after a dropout longer than `long_dropout_s`.
    bool recovery{false};
  };

  struct Result
  {
    bool accepted{true};
    /// d^2 = nu^T S^-1 nu, frozen at arrival (INV-3).
    double d2{0.0};
    double weight{1.0};
    /// R_z' = R_z / w.
    gtsam::Matrix corrected_covariance;
    bool gate_rejected{false};
  };

  CovarianceCorrector(
    const CovarianceCorrectionParams & params, std::string robust_kernel,
    double robust_kernel_param);

  [[nodiscard]] Result correct(const Input & input) const;

  /// Wraps the corrected covariance into a GTSAM noise model, applying layer C when
  /// `robust_kernel != none`.
  [[nodiscard]] gtsam::SharedNoiseModel make_noise_model(const gtsam::Matrix & covariance) const;

  /// Layer B weight function (`weight_function` parameter, D22: this is an explicit
  /// heuristic, not a Bayesian derivation).
  [[nodiscard]] double weight_of(double d2, int dof) const;

  /// Low speed relaxation of design §5.3.6: floors the position / rotation diagonal
  /// of Sigma_1 so that a standing vehicle does not reject valid absolute inputs.
  [[nodiscard]] gtsam::Matrix6 relax_low_speed(const gtsam::Matrix6 & sigma1, double speed) const;

  [[nodiscard]] double gate_threshold(int dof, bool recovery) const;

private:
  CovarianceCorrectionParams params_;
  std::string robust_kernel_;
  double robust_kernel_param_;
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__COVARIANCE_CORRECTOR_HPP_
