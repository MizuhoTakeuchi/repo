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
// Test 14 of design_last.md §11 (GNSS part): r_bg = 0 must reproduce GPSFactor, and
// a non-zero lever arm must match the numerical derivative.

#include "plugins/factor_graph_fusion/factors/gnss_lever_arm_factor.hpp"

#include <gtest/gtest.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>

#include <vector>

using autoware::pose_twist_fusion::fg::GnssLeverArmFactor;
using gtsam::symbol_shorthand::X;

namespace
{
std::vector<gtsam::Pose3> test_poses()
{
  return {
    gtsam::Pose3(),
    gtsam::Pose3(gtsam::Rot3::Yaw(1.1), gtsam::Point3(100.0, 25.0, 3.0)),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.03, -0.07, -2.0), gtsam::Point3(-5.0, 8.0, 0.4))};
}
}  // namespace

TEST(GnssLeverArmFactor, ZeroLeverArmEqualsGpsFactor)
{
  const gtsam::Point3 measured(101.0, 24.5, 3.2);
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.5);
  GnssLeverArmFactor ours(X(0), measured, gtsam::Point3(0, 0, 0), noise);
  gtsam::GPSFactor reference(X(0), measured, noise);

  for (const auto & pose : test_poses()) {
    gtsam::Matrix h_ours;
    gtsam::Matrix h_reference;
    const gtsam::Vector residual_ours = ours.evaluateError(pose, h_ours);
    const gtsam::Vector residual_reference = reference.evaluateError(pose, h_reference);
    EXPECT_TRUE(gtsam::assert_equal(residual_reference, residual_ours, 1e-9));
    EXPECT_TRUE(gtsam::assert_equal(h_reference, h_ours, 1e-6));
  }
}

TEST(GnssLeverArmFactor, LeverArmResidualAndJacobian)
{
  const gtsam::Point3 lever_arm(1.2, -0.3, 1.6);  // antenna on the roof
  const gtsam::Point3 measured(3.0, 4.0, 5.0);
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.5);
  GnssLeverArmFactor factor(X(0), measured, lever_arm, noise);

  for (const auto & pose : test_poses()) {
    gtsam::Matrix jacobian;
    const gtsam::Vector residual = factor.evaluateError(pose, jacobian);

    // residual = p + R r_bg - z
    const gtsam::Vector3 expected =
      pose.translation() + pose.rotation().matrix() * lever_arm - measured;
    EXPECT_TRUE(gtsam::assert_equal(gtsam::Vector(expected), residual, 1e-12));

    const auto error_fn = [&factor](const gtsam::Pose3 & p) { return factor.evaluateError(p); };
    const gtsam::Matrix numeric =
      gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(error_fn, pose);
    EXPECT_TRUE(gtsam::assert_equal(numeric, jacobian, 1e-6));
  }
}

TEST(GnssLeverArmFactor, AttitudeIsConstrainedThroughTheLeverArm)
{
  // The whole point of D25: with a non-zero lever arm the rotation block of the
  // Jacobian is non-zero, i.e. GNSS position does constrain attitude.
  const gtsam::Point3 lever_arm(1.5, 0.0, 1.0);
  const gtsam::Pose3 pose(gtsam::Rot3::Yaw(0.5), gtsam::Point3(0, 0, 0));
  const gtsam::Matrix jacobian =
    autoware::pose_twist_fusion::fg::gnss_lever_arm_jacobian(pose, lever_arm);
  const double rotation_block_norm = jacobian.leftCols(3).norm();
  EXPECT_GT(rotation_block_norm, 0.5);

  const gtsam::Matrix zero_arm =
    autoware::pose_twist_fusion::fg::gnss_lever_arm_jacobian(pose, gtsam::Point3(0, 0, 0));
  const double zero_rotation_block_norm = zero_arm.leftCols(3).norm();
  EXPECT_NEAR(zero_rotation_block_norm, 0.0, 1e-12);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
