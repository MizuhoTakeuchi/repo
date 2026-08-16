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
// Tests 15 and 24 of design_last.md §11: the initialization state machine and the
// yaw source for `pose_source: gnss` (D41).

#include "plugins/factor_graph_fusion/initializer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <optional>

using autoware::pose_twist_fusion::FactorGraphParams;
using autoware::pose_twist_fusion::fg::ImuSample;
using autoware::pose_twist_fusion::fg::Initializer;

namespace
{
void feed_static_imu(Initializer & initializer, double gravity, double duration = 1.5)
{
  for (double t = 0.0; t <= duration + 1e-9; t += 0.01) {
    ImuSample sample;
    sample.stamp = t;
    sample.accel = gtsam::Vector3(0.0, 0.0, gravity);
    sample.gyro = gtsam::Vector3::Zero();
    initializer.on_imu(sample);
  }
}
}  // namespace

TEST(Initializer, WaitsForEnoughImuSamples)
{
  FactorGraphParams params;
  Initializer initializer(params);
  initializer.on_initial_pose(0.0, gtsam::Pose3(), gtsam::Matrix6::Identity() * 1e-2);
  EXPECT_FALSE(initializer.try_initialize(0.0).has_value());
  EXPECT_EQ(initializer.state(), Initializer::State::kUninitialized);
  EXPECT_NE(initializer.message().find("IMU"), std::string::npos);
}

TEST(Initializer, InitialPoseSource)
{
  FactorGraphParams params;
  Initializer initializer(params);
  feed_static_imu(initializer, params.imu.gravity);
  EXPECT_FALSE(initializer.try_initialize(1.5).has_value());  // no initial pose yet

  const gtsam::Pose3 pose(gtsam::Rot3::Yaw(0.8), gtsam::Point3(5, 6, 7));
  initializer.on_initial_pose(0.0, pose, gtsam::Matrix6::Identity() * 1e-2);
  const auto anchor = initializer.try_initialize(1.5);
  ASSERT_TRUE(anchor.has_value());
  EXPECT_EQ(initializer.state(), Initializer::State::kRunning);
  EXPECT_LT((anchor->state.pose().translation() - pose.translation()).norm(), 1e-9);
  // The initial covariance is inflated by pose_cov_scale so that absolute inputs
  // can pull the estimate back if the initial pose was wrong.
  EXPECT_NEAR(anchor->covariance(3, 3), 1e-2 * params.initialization.pose_cov_scale, 1e-12);
}

TEST(Initializer, NdtPoseSource)
{
  FactorGraphParams params;
  params.initialization.pose_source = "ndt";
  Initializer initializer(params);
  feed_static_imu(initializer, params.imu.gravity);
  EXPECT_FALSE(initializer.try_initialize(1.5).has_value());

  const gtsam::Pose3 pose(gtsam::Rot3::Yaw(-0.2), gtsam::Point3(1, 2, 3));
  initializer.on_ndt(1.0, pose, gtsam::Matrix6::Identity() * 4e-4);
  ASSERT_TRUE(initializer.try_initialize(1.5).has_value());
}

TEST(Initializer, GnssWithDualAntennaHeading)
{
  // Test 24 (1): a trustworthy heading initializes within one cycle.
  FactorGraphParams params;
  params.initialization.pose_source = "gnss";
  Initializer initializer(params);
  feed_static_imu(initializer, params.imu.gravity);

  const double true_yaw = 1.05;
  initializer.on_gnss(
    1.4, gtsam::Point3(100, 50, 3), gtsam::Matrix3::Identity() * 0.25, true_yaw, 0.01);
  const auto anchor = initializer.try_initialize(1.5);
  ASSERT_TRUE(anchor.has_value());
  EXPECT_EQ(initializer.gnss_yaw_source_used(), "dual_antenna");
  EXPECT_NEAR(anchor->state.pose().rotation().yaw(), true_yaw, 0.1);
}

TEST(Initializer, GnssWithoutHeadingWhileStandingRefusesToStart)
{
  // Test 24 (2): the design refuses rather than starting with a guessed yaw (D41).
  FactorGraphParams params;
  params.initialization.pose_source = "gnss";
  Initializer initializer(params);
  feed_static_imu(initializer, params.imu.gravity);
  initializer.on_gnss(
    1.4, gtsam::Point3(100, 50, 3), gtsam::Matrix3::Identity() * 0.25, std::nullopt, 1.0);

  EXPECT_FALSE(initializer.try_initialize(1.5).has_value());
  EXPECT_EQ(initializer.state(), Initializer::State::kUninitialized);
  EXPECT_EQ(initializer.message(), "gnss init: yaw source unavailable");
}

TEST(Initializer, GnssCourseOverGround)
{
  // Test 24 (3): driving > 2 m/s for > 1 s over > 2 m yields the heading.
  FactorGraphParams params;
  params.initialization.pose_source = "gnss";
  Initializer initializer(params);
  feed_static_imu(initializer, params.imu.gravity, 0.5);

  const double heading = 0.6;
  const double speed = 5.0;
  for (double t = 0.0; t <= 2.0 + 1e-9; t += 0.1) {
    initializer.on_wheel_speed(t, speed);
    const double distance = speed * t;
    const gtsam::Point3 position(
      distance * std::cos(heading), distance * std::sin(heading), 0.0);
    initializer.on_gnss(t, position, gtsam::Matrix3::Identity() * 0.09, std::nullopt, 1.0);
  }

  const auto anchor = initializer.try_initialize(2.0);
  ASSERT_TRUE(anchor.has_value());
  EXPECT_EQ(initializer.gnss_yaw_source_used(), "course");
  EXPECT_NEAR(anchor->state.pose().rotation().yaw(), heading, 0.2);
  // The initial velocity follows the wheel speed along the estimated heading.
  EXPECT_NEAR(anchor->state.v().norm(), speed, 1e-6);
}

TEST(Initializer, GnssZeroYawFallbackIsExplicitOnly)
{
  // Test 24 (4): `zero_with_large_covariance` must be selected explicitly, and the
  // yaw variance must be the configured fallback.
  FactorGraphParams params;
  params.initialization.pose_source = "gnss";
  params.initialization.gnss_yaw_source = "zero_with_large_covariance";
  params.initialization.gnss_yaw_fallback_stddev_rad = 1.0;
  Initializer initializer(params);
  feed_static_imu(initializer, params.imu.gravity);
  initializer.on_gnss(
    1.4, gtsam::Point3(0, 0, 0), gtsam::Matrix3::Identity() * 0.25, std::nullopt, 1.0);

  const auto anchor = initializer.try_initialize(1.5);
  ASSERT_TRUE(anchor.has_value());
  EXPECT_EQ(initializer.gnss_yaw_source_used(), "zero_with_large_covariance");
  EXPECT_NEAR(anchor->covariance(2, 2), 1.0, 1e-9);

  // `auto` never picks this path: standing without a heading it refuses.
  FactorGraphParams automatic = params;
  automatic.initialization.gnss_yaw_source = "auto";
  Initializer strict(automatic);
  feed_static_imu(strict, params.imu.gravity);
  strict.on_gnss(1.4, gtsam::Point3(0, 0, 0), gtsam::Matrix3::Identity() * 0.25, std::nullopt, 1.0);
  EXPECT_FALSE(strict.try_initialize(1.5).has_value());
}

TEST(Initializer, GnssAttitudeComesFromGravityWhenStanding)
{
  // For pose_source = gnss the roll / pitch are derived from gravity and b_a is
  // assumed, because one static IMU cannot separate the two (D26).
  FactorGraphParams params;
  params.initialization.pose_source = "gnss";
  Initializer initializer(params);

  const double roll = 0.1;
  const double pitch = -0.05;
  const gtsam::Rot3 attitude = gtsam::Rot3::RzRyRx(roll, pitch, 0.0);
  const gtsam::Vector3 specific_force =
    attitude.matrix().transpose() * gtsam::Vector3(0.0, 0.0, params.imu.gravity);
  for (double t = 0.0; t <= 1.5 + 1e-9; t += 0.01) {
    ImuSample sample;
    sample.stamp = t;
    sample.accel = specific_force;
    sample.gyro = gtsam::Vector3::Zero();
    initializer.on_imu(sample);
  }
  initializer.on_gnss(
    1.4, gtsam::Point3(0, 0, 0), gtsam::Matrix3::Identity() * 0.25, 0.0, 0.01);

  const auto anchor = initializer.try_initialize(1.5);
  ASSERT_TRUE(anchor.has_value());
  const gtsam::Vector3 rpy = anchor->state.pose().rotation().rpy();
  EXPECT_NEAR(rpy(0), roll, 1e-6);
  EXPECT_NEAR(rpy(1), pitch, 1e-6);
  EXPECT_LT(anchor->bias.accelerometer().norm(), 1e-12);  // b_a assumed, not estimated
}

TEST(Initializer, ResetReturnsToUninitialized)
{
  FactorGraphParams params;
  Initializer initializer(params);
  feed_static_imu(initializer, params.imu.gravity);
  initializer.on_initial_pose(0.0, gtsam::Pose3(), gtsam::Matrix6::Identity() * 1e-2);
  ASSERT_TRUE(initializer.try_initialize(1.5).has_value());

  initializer.reset();
  EXPECT_EQ(initializer.state(), Initializer::State::kUninitialized);
  EXPECT_FALSE(initializer.try_initialize(2.0).has_value());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
