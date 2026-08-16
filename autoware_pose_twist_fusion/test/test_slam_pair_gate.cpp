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
// Test 23 of design_last.md §11: SLAM relative constraints (D38).

#include "plugins/factor_graph_fusion/slam_pair_gate.hpp"

#include <gtest/gtest.h>

#include <memory>

using autoware::pose_twist_fusion::CovarianceCorrectionParams;
using autoware::pose_twist_fusion::SlamParams;
using autoware::pose_twist_fusion::fg::CovarianceCorrector;
using autoware::pose_twist_fusion::fg::SlamPairGate;

namespace
{
std::shared_ptr<CovarianceCorrector> make_corrector()
{
  return std::make_shared<CovarianceCorrector>(CovarianceCorrectionParams{}, "none", 1.345);
}

gtsam::Matrix6 small_covariance(double stddev)
{
  return gtsam::Matrix6::Identity() * stddev * stddev;
}
}  // namespace

TEST(SlamPairGate, FirstSampleOnlyStartsAPair)
{
  SlamPairGate gate(SlamParams{}, make_corrector());
  const auto result = gate.evaluate(
    1.0, gtsam::Pose3(), small_covariance(0.01), gtsam::Pose3(), small_covariance(0.01));
  EXPECT_FALSE(result.has_pair);
  EXPECT_DOUBLE_EQ(gate.previous_stamp(), 1.0);
}

TEST(SlamPairGate, NormalDrivingIsAlwaysAccepted)
{
  // (1) 15 m/s with 100 ms SLAM: every increment is 1.5 m. A fixed 1.0 m jump
  // threshold - which the design explicitly forbids - would reject all of them.
  SlamPairGate gate(SlamParams{}, make_corrector());
  const double speed = 15.0;
  const double period = 0.1;

  gtsam::Pose3 pose;
  gate.evaluate(0.0, pose, small_covariance(0.01), gtsam::Pose3(), small_covariance(0.01));

  for (int i = 1; i <= 20; ++i) {
    const gtsam::Pose3 next(gtsam::Rot3(), gtsam::Point3(speed * period * i, 0.0, 0.0));
    const gtsam::Pose3 delta_graph1 = pose.inverse() * next;
    const auto result = gate.evaluate(
      i * period, next, small_covariance(0.01), delta_graph1, small_covariance(0.02));
    ASSERT_TRUE(result.has_pair);
    EXPECT_TRUE(result.accepted) << "step " << i << " d2=" << result.d2;
    EXPECT_GT(result.relative.translation().norm(), 1.0);  // > any fixed 1 m threshold
    pose = next;
  }
  EXPECT_EQ(gate.chi2_reject_count(), 0u);
  EXPECT_EQ(gate.dynamic_reject_count(), 0u);
}

TEST(SlamPairGate, LoopClosureJumpIsRejectedByTheChiSquaredTest)
{
  // (2) A 0.3 m reference jump that dead reckoning cannot explain.
  SlamPairGate gate(SlamParams{}, make_corrector());
  const double period = 0.1;
  gtsam::Pose3 pose;
  gate.evaluate(0.0, pose, small_covariance(0.01), gtsam::Pose3(), small_covariance(0.01));

  const gtsam::Pose3 expected(gtsam::Rot3(), gtsam::Point3(1.5, 0.0, 0.0));
  const gtsam::Pose3 jumped(gtsam::Rot3(), gtsam::Point3(1.8, 0.0, 0.0));  // +0.3 m
  const auto result =
    gate.evaluate(period, jumped, small_covariance(0.01), expected, small_covariance(0.02));

  ASSERT_TRUE(result.has_pair);
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.rejected_chi2);
  EXPECT_GT(result.d2, 22.46);
  EXPECT_EQ(gate.chi2_reject_count(), 1u);
}

TEST(SlamPairGate, LowSpeedLoopClosureIsAlsoDetected)
{
  // (2) at 1 m/s: the chi-squared test does not depend on the driving speed.
  SlamPairGate gate(SlamParams{}, make_corrector());
  gate.evaluate(0.0, gtsam::Pose3(), small_covariance(0.01), gtsam::Pose3(), small_covariance(0.01));

  const gtsam::Pose3 expected(gtsam::Rot3(), gtsam::Point3(0.1, 0.0, 0.0));
  const gtsam::Pose3 jumped(gtsam::Rot3(), gtsam::Point3(0.4, 0.0, 0.0));
  const auto result =
    gate.evaluate(0.1, jumped, small_covariance(0.01), expected, small_covariance(0.02));
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.rejected_chi2);
}

TEST(SlamPairGate, DynamicBoundCatchesAnOverconfidentSlamFrontEnd)
{
  // The backup criterion: a front end that reports a huge covariance would make d^2
  // small no matter how large the jump is, so the kinematic bound must still fire.
  SlamPairGate gate(SlamParams{}, make_corrector());
  const gtsam::Matrix6 huge = gtsam::Matrix6::Identity() * 1e6;
  gate.evaluate(0.0, gtsam::Pose3(), huge, gtsam::Pose3(), small_covariance(0.01));

  const gtsam::Pose3 expected(gtsam::Rot3(), gtsam::Point3(1.5, 0.0, 0.0));
  const gtsam::Pose3 teleported(gtsam::Rot3(), gtsam::Point3(30.0, 0.0, 0.0));
  const auto result = gate.evaluate(0.1, teleported, huge, expected, small_covariance(0.02));

  EXPECT_LT(result.d2, 22.46) << "the chi-squared test alone would accept this";
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.rejected_dynamic);
  EXPECT_EQ(gate.dynamic_reject_count(), 1u);
}

TEST(SlamPairGate, WeightAndCovarianceAreFrozenAtPairFormation)
{
  // (4) INV-3: d^2, w and Sigma_Delta' belong to the pair and never change later.
  SlamPairGate gate(SlamParams{}, make_corrector());
  gate.evaluate(0.0, gtsam::Pose3(), small_covariance(0.05), gtsam::Pose3(), small_covariance(0.01));

  const gtsam::Pose3 expected(gtsam::Rot3(), gtsam::Point3(1.5, 0.0, 0.0));
  // Offset chosen so that dof < d^2 < gate threshold: accepted, but down-weighted.
  const gtsam::Pose3 measured(gtsam::Rot3(), gtsam::Point3(1.85, 0.0, 0.0));
  const auto first =
    gate.evaluate(0.1, measured, small_covariance(0.05), expected, small_covariance(0.02));
  ASSERT_TRUE(first.accepted);
  EXPECT_LT(first.weight, 1.0);  // layer B down-weighted it
  // R' = R / w
  const double raw = first.covariance(3, 3) * first.weight;
  EXPECT_GT(first.covariance(3, 3), raw);

  const auto copy = first;
  EXPECT_DOUBLE_EQ(copy.d2, first.d2);
  EXPECT_DOUBLE_EQ(copy.weight, first.weight);
  EXPECT_TRUE(copy.covariance.isApprox(first.covariance));
}

TEST(SlamPairGate, RelativeCovarianceCanComeFromTheFrontEnd)
{
  // `covariance_is_relative: true` is the most correct source (§5.3.3c), and it
  // must be used directly rather than being rebuilt from two absolute covariances.
  SlamParams params;
  params.covariance_is_relative = true;
  params.covariance_scale = 1.0;
  params.relative_covariance_scale = 1.0;
  SlamPairGate gate(params, make_corrector());

  const gtsam::Matrix6 relative_covariance = small_covariance(0.03);
  gate.evaluate(
    0.0, gtsam::Pose3(), relative_covariance, gtsam::Pose3(), small_covariance(0.01));
  const gtsam::Pose3 expected(gtsam::Rot3(), gtsam::Point3(1.5, 0.0, 0.0));
  const auto result =
    gate.evaluate(0.1, expected, relative_covariance, expected, small_covariance(0.01));
  ASSERT_TRUE(result.accepted);
  EXPECT_NEAR(result.covariance(3, 3), relative_covariance(3, 3), 1e-12);
}

TEST(SlamPairGate, ResetForgetsThePreviousSample)
{
  SlamPairGate gate(SlamParams{}, make_corrector());
  gate.evaluate(0.0, gtsam::Pose3(), small_covariance(0.01), gtsam::Pose3(), small_covariance(0.01));
  gate.reset();
  EXPECT_DOUBLE_EQ(gate.previous_stamp(), 0.0);
  const auto result = gate.evaluate(
    0.1, gtsam::Pose3(), small_covariance(0.01), gtsam::Pose3(), small_covariance(0.01));
  EXPECT_FALSE(result.has_pair);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
