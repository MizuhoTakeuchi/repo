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
//
// Test 5 (a) of design_last.md §11: the GNSS outlier decision is governed by the
// NIS, not by a distance. Both configured conditions of the design are executed.

#include "plugins/factor_graph_fusion/covariance_corrector.hpp"

#include <gtest/gtest.h>

#include <cmath>

using autoware::pose_twist_fusion::CovarianceCorrectionParams;
using autoware::pose_twist_fusion::fg::CovarianceCorrector;

namespace
{
CovarianceCorrector::Input make_gnss_input(
  double distance, double predicted_stddev_scale, double measurement_stddev_scale)
{
  CovarianceCorrector::Input input;
  input.dof = 3;
  input.innovation = gtsam::Vector3(distance, 0.0, 0.0);

  gtsam::Matrix3 sigma_h = gtsam::Matrix3::Zero();
  sigma_h.diagonal() << std::pow(0.1 * predicted_stddev_scale, 2),
    std::pow(0.1 * predicted_stddev_scale, 2), std::pow(0.2 * predicted_stddev_scale, 2);

  gtsam::Matrix3 r_z = gtsam::Matrix3::Zero();
  r_z.diagonal() << std::pow(0.5 * measurement_stddev_scale, 2),
    std::pow(0.5 * measurement_stddev_scale, 2), std::pow(1.0 * measurement_stddev_scale, 2);

  input.measurement_covariance = r_z;
  input.innovation_covariance = sigma_h + r_z;
  return input;
}
}  // namespace

TEST(CovarianceCorrector, ChiSquaredValueMatchesTheAnalyticSolution)
{
  const CovarianceCorrectionParams params;
  const CovarianceCorrector corrector(params, "none", 1.345);

  // S_x = 0.1^2 + 0.5^2 = 0.26
  const double s_x = 0.26;
  for (const double distance : {5.0, 20.0, 100.0}) {
    const auto result = corrector.correct(make_gnss_input(distance, 1.0, 1.0));
    EXPECT_NEAR(result.d2, distance * distance / s_x, 1e-6) << "distance " << distance;
  }
}

TEST(CovarianceCorrector, FiveMetreOutlierIsRejectedUnderTheDefaultCondition)
{
  const CovarianceCorrectionParams params;  // gate_chi2.dim3 = 16.27
  const CovarianceCorrector corrector(params, "none", 1.345);
  const auto result = corrector.correct(make_gnss_input(5.0, 1.0, 1.0));
  EXPECT_GT(result.d2, params.gate_chi2.dim3);
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.gate_rejected);
}

TEST(CovarianceCorrector, WithTheGateOffTheWeightSaturatesAtWMin)
{
  CovarianceCorrectionParams params;
  params.enable_gate = false;
  const CovarianceCorrector corrector(params, "none", 1.345);
  const auto result = corrector.correct(make_gnss_input(5.0, 1.0, 1.0));
  EXPECT_TRUE(result.accepted);
  // w = clamp(exp(-0.5 (96.15 - 3)), 0.05, 1) = w_min
  EXPECT_NEAR(result.weight, params.w_min, 1e-12);
  // R' = R / w, i.e. the constraint is weakened by 1/w_min = 20.
  EXPECT_NEAR(result.corrected_covariance(0, 0), 0.25 / params.w_min, 1e-9);
}

TEST(CovarianceCorrector, TheSameFiveMetresIsAcceptedWhenTheCovariancesAreLarger)
{
  // Second condition of test 5 (a): with 10x larger standard deviations the very
  // same 5 m innovation is accepted with w = 1. The threshold is the NIS, not the
  // distance - a distance based rule cannot express this.
  const CovarianceCorrectionParams params;
  const CovarianceCorrector corrector(params, "none", 1.345);
  const auto result = corrector.correct(make_gnss_input(5.0, 10.0, 10.0));
  EXPECT_NEAR(result.d2, 25.0 / 26.0, 1e-6);
  EXPECT_LT(result.d2, params.gate_chi2.dim3);
  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(result.weight, 1.0, 1e-12);
  EXPECT_NEAR(result.corrected_covariance(0, 0), 25.0, 1e-9);
}

TEST(CovarianceCorrector, WeightFunctionAlternatives)
{
  // D22: the weight function is an explicit heuristic and must be swappable.
  CovarianceCorrectionParams params;
  params.enable_gate = false;
  const double d2 = 20.0;

  params.weight_function = "exp_excess_nis";
  EXPECT_NEAR(
    CovarianceCorrector(params, "none", 1.0).weight_of(d2, 6),
    std::max(std::exp(-0.5 * (d2 - 6.0)), params.w_min), 1e-12);

  params.weight_function = "inverse_nis";
  EXPECT_NEAR(CovarianceCorrector(params, "none", 1.0).weight_of(d2, 6), 6.0 / 20.0, 1e-12);

  params.weight_function = "inverse_linear";
  EXPECT_NEAR(CovarianceCorrector(params, "none", 1.0).weight_of(d2, 6), 1.0 / 15.0, 1e-12);

  // Consistent measurements (d2 <= dof) are never down-weighted.
  for (const char * function : {"exp_excess_nis", "inverse_nis", "inverse_linear"}) {
    params.weight_function = function;
    EXPECT_NEAR(CovarianceCorrector(params, "none", 1.0).weight_of(3.0, 6), 1.0, 1e-12);
  }
}

TEST(CovarianceCorrector, GateThresholdsPerDegreeOfFreedom)
{
  const CovarianceCorrectionParams params;
  const CovarianceCorrector corrector(params, "none", 1.345);
  EXPECT_NEAR(corrector.gate_threshold(1, false), 10.83, 1e-12);
  EXPECT_NEAR(corrector.gate_threshold(3, false), 16.27, 1e-12);
  EXPECT_NEAR(corrector.gate_threshold(6, false), 22.46, 1e-12);
  // After a long dropout the gate is relaxed so the estimator can re-acquire.
  EXPECT_NEAR(corrector.gate_threshold(3, true), 16.27 * params.recovery_gate_scale, 1e-9);
}

TEST(CovarianceCorrector, LowSpeedRelaxationFloorsTheDiagonal)
{
  const CovarianceCorrectionParams params;
  const CovarianceCorrector corrector(params, "none", 1.345);
  const gtsam::Matrix6 tight = gtsam::Matrix6::Identity() * 1e-8;

  const gtsam::Matrix6 standing = corrector.relax_low_speed(tight, 0.0);
  EXPECT_NEAR(standing(3, 3), std::pow(params.low_speed_min_pos_stddev, 2), 1e-15);
  EXPECT_NEAR(standing(0, 0), std::pow(params.low_speed_min_rot_stddev, 2), 1e-15);

  // Above the threshold the covariance is left untouched.
  EXPECT_TRUE(corrector.relax_low_speed(tight, 5.0).isApprox(tight));
}

TEST(CovarianceCorrector, RobustKernelWrapsTheNoiseModel)
{
  const CovarianceCorrectionParams params;
  const gtsam::Matrix covariance = gtsam::Matrix::Identity(3, 3) * 0.25;

  const auto plain = CovarianceCorrector(params, "none", 1.345).make_noise_model(covariance);
  EXPECT_EQ(boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(plain), nullptr);

  const auto robust = CovarianceCorrector(params, "huber", 1.345).make_noise_model(covariance);
  EXPECT_NE(boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(robust), nullptr);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
