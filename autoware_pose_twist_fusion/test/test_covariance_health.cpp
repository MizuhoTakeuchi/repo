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
// Test 9 of design_last.md §11: covariance health, and the Monte-Carlo check of the
// joint 15x15 forward propagation used by the lio_sam mode (D39).

#include "autoware/pose_twist_fusion/covariance_convert.hpp"
#include "plugins/lio_sam/lio_sam_covariance.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

namespace lio_sam = autoware::pose_twist_fusion::lio_sam;
namespace cc = autoware::pose_twist_fusion::covariance_convert;

namespace
{
constexpr double kGravity = 9.80665;
constexpr double kDt = 0.01;
constexpr int kSteps = 50;  // 0.5 s
constexpr double kAccelNoise = 0.05;  // m/s^2/sqrt(Hz), large enough for a quick MC
constexpr double kGyroNoise = 0.005;  // rad/s/sqrt(Hz)

boost::shared_ptr<gtsam::PreintegrationParams> make_params()
{
  auto params = gtsam::PreintegrationParams::MakeSharedU(kGravity);
  params->accelerometerCovariance = gtsam::Matrix33::Identity() * kAccelNoise * kAccelNoise;
  params->gyroscopeCovariance = gtsam::Matrix33::Identity() * kGyroNoise * kGyroNoise;
  params->integrationCovariance = gtsam::Matrix33::Identity() * 1e-10;
  return params;
}

/// Anchor covariance with a deliberately positive position-velocity correlation,
/// which is the case where dropping the cross terms under-estimates (D33).
lio_sam::Matrix15 anchor_covariance()
{
  lio_sam::Matrix15 covariance = lio_sam::Matrix15::Zero();
  covariance.block<3, 3>(0, 0) = gtsam::Matrix3::Identity() * 1e-4;   // attitude
  covariance.block<3, 3>(3, 3) = gtsam::Matrix3::Identity() * 4e-2;   // position
  covariance.block<3, 3>(6, 6) = gtsam::Matrix3::Identity() * 1e-2;   // velocity
  covariance.block<3, 3>(9, 9) = gtsam::Matrix3::Identity() * 1e-4;   // accel bias
  covariance.block<3, 3>(12, 12) = gtsam::Matrix3::Identity() * 1e-6;  // gyro bias
  // position <-> velocity correlation (rho = 0.5)
  const double cross = 0.5 * std::sqrt(4e-2 * 1e-2);
  covariance.block<3, 3>(3, 6) = gtsam::Matrix3::Identity() * cross;
  covariance.block<3, 3>(6, 3) = gtsam::Matrix3::Identity() * cross;
  return covariance;
}
}  // namespace

TEST(CovarianceHealth, JointPropagationMatchesMonteCarlo)
{
  const auto params = make_params();
  const gtsam::NavState anchor_state(
    gtsam::Pose3(gtsam::Rot3::Yaw(0.3), gtsam::Point3(1.0, 2.0, 0.0)),
    gtsam::Vector3(8.0, 0.0, 0.0));
  const gtsam::imuBias::ConstantBias anchor_bias;
  const lio_sam::Matrix15 sigma_anchor = anchor_covariance();

  // Nominal measurements: level, constant speed, mild yaw rate.
  const gtsam::Vector3 accel(0.0, 1.6, kGravity);
  const gtsam::Vector3 gyro(0.0, 0.0, 0.2);

  gtsam::PreintegratedImuMeasurements nominal(params, anchor_bias);
  for (int i = 0; i < kSteps; ++i) {
    nominal.integrateMeasurement(accel, gyro, kDt);
  }
  const auto propagated = lio_sam::propagate_joint(
    nominal, anchor_state, anchor_bias, sigma_anchor, kSteps * kDt, 1e-4, 1e-5);

  // --- Monte Carlo ---------------------------------------------------------
  // truth : integrate the clean measurements from the perturbed anchor
  // filter: integrate the noisy measurements (which contain the bias error) from
  //         the nominal anchor - exactly the error the covariance describes.
  std::mt19937 generator(12345);
  std::normal_distribution<double> normal(0.0, 1.0);
  const Eigen::LLT<lio_sam::Matrix15> llt(sigma_anchor);
  const lio_sam::Matrix15 sqrt_sigma = llt.matrixL();
  const double accel_sigma = kAccelNoise / std::sqrt(kDt);
  const double gyro_sigma = kGyroNoise / std::sqrt(kDt);

  gtsam::Matrix9 empirical = gtsam::Matrix9::Zero();
  const int samples = 3000;
  for (int m = 0; m < samples; ++m) {
    Eigen::Matrix<double, 15, 1> unit;
    for (int k = 0; k < 15; ++k) {
      unit(k) = normal(generator);
    }
    const Eigen::Matrix<double, 15, 1> delta = sqrt_sigma * unit;

    const gtsam::NavState true_state = anchor_state.retract(delta.head<9>());
    const gtsam::imuBias::ConstantBias true_bias(delta.segment<3>(9), delta.segment<3>(12));

    gtsam::PreintegratedImuMeasurements truth(params, anchor_bias);
    gtsam::PreintegratedImuMeasurements filtered(params, anchor_bias);
    for (int i = 0; i < kSteps; ++i) {
      gtsam::Vector3 accel_noise;
      gtsam::Vector3 gyro_noise;
      for (int k = 0; k < 3; ++k) {
        accel_noise(k) = normal(generator) * accel_sigma;
        gyro_noise(k) = normal(generator) * gyro_sigma;
      }
      truth.integrateMeasurement(accel, gyro, kDt);
      // the estimator sees the true bias plus white noise on top of the signal
      filtered.integrateMeasurement(
        accel + true_bias.accelerometer() + accel_noise,
        gyro + true_bias.gyroscope() + gyro_noise, kDt);
    }
    const gtsam::NavState predicted_truth = truth.predict(true_state, anchor_bias);
    const gtsam::NavState predicted_filter = filtered.predict(anchor_state, anchor_bias);
    const gtsam::Vector9 error = predicted_filter.localCoordinates(predicted_truth);
    empirical += error * error.transpose();
  }
  empirical /= static_cast<double>(samples);

  // Position and velocity blocks must agree with the analytic propagation.
  const double position_relative =
    std::abs(empirical.block<3, 3>(3, 3).trace() - propagated.nav.block<3, 3>(3, 3).trace()) /
    propagated.nav.block<3, 3>(3, 3).trace();
  const double velocity_relative =
    std::abs(empirical.block<3, 3>(6, 6).trace() - propagated.nav.block<3, 3>(6, 6).trace()) /
    propagated.nav.block<3, 3>(6, 6).trace();
  EXPECT_LT(position_relative, 0.05) << "empirical:\n"
                                     << empirical << "\nanalytic:\n"
                                     << propagated.nav;
  EXPECT_LT(velocity_relative, 0.05);
}

TEST(CovarianceHealth, BlockwisePropagationUnderestimatesThePosition)
{
  // D39: propagating X, V and B separately drops the 2 dt P_pv term, which is the
  // regression this test pins down. `blockwise` is deliberately not offered as an
  // option; `fixed` is the only escape hatch.
  const auto params = make_params();
  const gtsam::NavState anchor_state(gtsam::Pose3(), gtsam::Vector3(8.0, 0.0, 0.0));
  const gtsam::imuBias::ConstantBias anchor_bias;
  const lio_sam::Matrix15 sigma_anchor = anchor_covariance();

  gtsam::PreintegratedImuMeasurements nominal(params, anchor_bias);
  for (int i = 0; i < kSteps; ++i) {
    nominal.integrateMeasurement(gtsam::Vector3(0.0, 0.0, kGravity), gtsam::Vector3::Zero(), kDt);
  }
  const auto joint = lio_sam::propagate_joint(
    nominal, anchor_state, anchor_bias, sigma_anchor, kSteps * kDt, 1e-4, 1e-5);

  lio_sam::Matrix15 block_diagonal = sigma_anchor;
  block_diagonal.block<3, 3>(3, 6).setZero();
  block_diagonal.block<3, 3>(6, 3).setZero();
  const auto blockwise = lio_sam::propagate_joint(
    nominal, anchor_state, anchor_bias, block_diagonal, kSteps * kDt, 1e-4, 1e-5);

  const double blockwise_position_variance = blockwise.nav.block<3, 3>(3, 3).trace();
  const double joint_position_variance = joint.nav.block<3, 3>(3, 3).trace();
  EXPECT_LT(blockwise_position_variance, joint_position_variance)
    << "dropping the position-velocity correlation must under-estimate, not over-estimate";
}

TEST(CovarianceHealth, PropagatedCovarianceIsSymmetricPsdAndNonZero)
{
  const auto params = make_params();
  const gtsam::NavState anchor_state;
  const gtsam::imuBias::ConstantBias anchor_bias;

  gtsam::PreintegratedImuMeasurements pim(params, anchor_bias);
  for (int i = 0; i < kSteps; ++i) {
    pim.integrateMeasurement(gtsam::Vector3(0.0, 0.0, kGravity), gtsam::Vector3::Zero(), kDt);
  }
  const auto propagated = lio_sam::propagate_joint(
    pim, anchor_state, anchor_bias, anchor_covariance(), kSteps * kDt, 1e-4, 1e-5);

  // design §3.3 / §5.3.9: symmetric, PSD, finite, never all zero.
  EXPECT_LT((propagated.pose - propagated.pose.transpose()).cwiseAbs().maxCoeff(), 1e-9);
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> solver(propagated.pose);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-12);
  EXPECT_TRUE(propagated.pose.allFinite());
  EXPECT_GT(propagated.pose.norm(), 0.0);
  EXPECT_GT(propagated.velocity.norm(), 0.0);
  // the bias covariance grows with the random walk
  EXPECT_GT(propagated.bias(0, 0), anchor_covariance()(9, 9));
}

TEST(CovarianceHealth, SanitizeGuaranteesAPublishableCovariance)
{
  // Whatever the estimator produced, §3.3 forbids publishing zero / NaN.
  gtsam::Matrix6 zero = gtsam::Matrix6::Zero();
  const gtsam::Matrix6 healthy = cc::sanitize(zero, 1.0e4);
  for (int i = 0; i < 6; ++i) {
    EXPECT_GT(healthy(i, i), 0.0);
  }
  EXPECT_TRUE(healthy.allFinite());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
