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
// Test 14 of design_last.md §11 (wheel speed / non-holonomic part): residual and
// analytic Jacobians against gtsam::numericalDerivative.

#include "plugins/factor_graph_fusion/factors/nonholonomic_factor.hpp"
#include "plugins/factor_graph_fusion/factors/wheel_speed_factor.hpp"

#include <gtest/gtest.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <vector>

using autoware::pose_twist_fusion::fg::NonHolonomicFactor;
using autoware::pose_twist_fusion::fg::WheelSpeedFactor;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace
{
std::vector<gtsam::Pose3> test_poses()
{
  return {
    gtsam::Pose3(),
    gtsam::Pose3(gtsam::Rot3::Yaw(0.9), gtsam::Point3(10.0, -3.0, 0.5)),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.05, -0.1, 2.4), gtsam::Point3(-7.0, 2.0, 1.5))};
}
}  // namespace

TEST(WheelSpeedFactor, ResidualIsTheForwardBodyVelocityError)
{
  const gtsam::Pose3 pose(gtsam::Rot3::Yaw(M_PI / 2.0), gtsam::Point3(0, 0, 0));
  const gtsam::Vector3 world_velocity(0.0, 12.0, 0.0);  // heading +y at 12 m/s
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.2);
  WheelSpeedFactor factor(X(0), V(0), 12.0, noise);
  const gtsam::Vector residual = factor.evaluateError(pose, world_velocity);
  ASSERT_EQ(residual.size(), 1);
  EXPECT_NEAR(residual(0), 0.0, 1e-12);
}

TEST(WheelSpeedFactor, JacobiansMatchNumericalDerivatives)
{
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.2);
  WheelSpeedFactor factor(X(0), V(0), 8.0, noise);

  for (const auto & pose : test_poses()) {
    const std::vector<gtsam::Vector3> velocities = {
      gtsam::Vector3(8.0, 0.1, -0.05), gtsam::Vector3(-3.0, 0.0, 0.0),
      gtsam::Vector3(0.0, 0.0, 0.0)};
    for (const auto & velocity : velocities) {
      gtsam::Matrix h_pose;
      gtsam::Matrix h_velocity;
      (void)factor.evaluateError(pose, velocity, h_pose, h_velocity);

      const auto error_fn = [&factor](const gtsam::Pose3 & p, const gtsam::Vector3 & v) {
        return factor.evaluateError(p, v);
      };
      const gtsam::Matrix numeric_pose =
        gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Vector3>(
          error_fn, pose, velocity);
      const gtsam::Matrix numeric_velocity =
        gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Vector3>(
          error_fn, pose, velocity);

      EXPECT_TRUE(gtsam::assert_equal(numeric_pose, h_pose, 1e-6));
      EXPECT_TRUE(gtsam::assert_equal(numeric_velocity, h_velocity, 1e-6));
    }
  }
}

TEST(NonHolonomicFactor, ResidualIsTheLateralAndVerticalVelocity)
{
  const gtsam::Pose3 pose(gtsam::Rot3::Yaw(0.3), gtsam::Point3(1, 2, 3));
  const gtsam::Vector3 body_velocity(10.0, 0.4, -0.2);
  const gtsam::Vector3 world_velocity = pose.rotation() * body_velocity;
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.05));
  NonHolonomicFactor factor(X(0), V(0), noise);
  const gtsam::Vector residual = factor.evaluateError(pose, world_velocity);
  ASSERT_EQ(residual.size(), 2);
  EXPECT_NEAR(residual(0), 0.4, 1e-12);
  EXPECT_NEAR(residual(1), -0.2, 1e-12);
}

TEST(NonHolonomicFactor, JacobiansMatchNumericalDerivatives)
{
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.05));
  NonHolonomicFactor factor(X(0), V(0), noise);
  for (const auto & pose : test_poses()) {
    const gtsam::Vector3 velocity(6.0, -0.3, 0.2);
    gtsam::Matrix h_pose;
    gtsam::Matrix h_velocity;
    (void)factor.evaluateError(pose, velocity, h_pose, h_velocity);

    const auto error_fn = [&factor](const gtsam::Pose3 & p, const gtsam::Vector3 & v) {
      return factor.evaluateError(p, v);
    };
    EXPECT_TRUE(gtsam::assert_equal(
      gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Vector3>(
        error_fn, pose, velocity),
      h_pose, 1e-6));
    EXPECT_TRUE(gtsam::assert_equal(
      gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Vector3>(
        error_fn, pose, velocity),
      h_velocity, 1e-6));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
