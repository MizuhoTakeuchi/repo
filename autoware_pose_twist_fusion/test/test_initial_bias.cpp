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
// Test 19 of design_last.md §11: initial IMU bias and the gravity sign convention.
//
// b_a = mean(accel) - R_wb^T (0,0,g). Using b_a = mean(accel) instead produces the
// classic 9.8 m/s^2 error, and flipping the gravity sign produces 19.6 m/s^2.

#include "plugins/factor_graph_fusion/initializer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using autoware::pose_twist_fusion::FactorGraphParams;
using autoware::pose_twist_fusion::fg::Anchor;
using autoware::pose_twist_fusion::fg::ImuSample;
using autoware::pose_twist_fusion::fg::Initializer;

namespace
{
struct Attitude
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

/// Static IMU stream for a vehicle at `attitude` with the given true biases.
std::vector<ImuSample> static_stream(
  const Attitude & attitude, const gtsam::Vector3 & accel_bias, const gtsam::Vector3 & gyro_bias,
  double gravity, double duration = 1.5, double period = 0.01)
{
  const gtsam::Rot3 r_wb = gtsam::Rot3::RzRyRx(attitude.roll, attitude.pitch, attitude.yaw);
  // Specific force of a stationary IMU: f = R_wb^T (0,0,g) + b_a (MakeSharedU).
  const gtsam::Vector3 specific_force =
    r_wb.matrix().transpose() * gtsam::Vector3(0.0, 0.0, gravity) + accel_bias;

  std::vector<ImuSample> samples;
  for (double t = 0.0; t <= duration + 1e-9; t += period) {
    ImuSample sample;
    sample.stamp = t;
    sample.accel = specific_force;
    sample.gyro = gyro_bias;
    samples.push_back(sample);
  }
  return samples;
}

Anchor initialize_with(const Attitude & attitude, const gtsam::Vector3 & accel_bias,
                       const gtsam::Vector3 & gyro_bias)
{
  FactorGraphParams params;
  params.initialization.pose_source = "initialpose";
  Initializer initializer(params);

  const gtsam::Pose3 initial_pose(
    gtsam::Rot3::RzRyRx(attitude.roll, attitude.pitch, attitude.yaw), gtsam::Point3(10, 20, 1));
  initializer.on_initial_pose(0.0, initial_pose, gtsam::Matrix6::Identity() * 1e-2);

  const auto samples = static_stream(attitude, accel_bias, gyro_bias, params.imu.gravity);
  for (const auto & sample : samples) {
    initializer.on_imu(sample);
  }
  const auto anchor = initializer.try_initialize(samples.back().stamp);
  EXPECT_EQ(initializer.state(), Initializer::State::kRunning) << initializer.message();
  EXPECT_TRUE(anchor.has_value());
  return anchor.value_or(Anchor{});
}
}  // namespace

TEST(InitialBias, LevelVehicle)
{
  const gtsam::Vector3 accel_bias(0.05, -0.02, 0.10);
  const gtsam::Vector3 gyro_bias(0.001, -0.002, 0.003);
  const Anchor anchor = initialize_with({0.0, 0.0, 0.0}, accel_bias, gyro_bias);

  ASSERT_TRUE(anchor.valid);
  EXPECT_LT((anchor.bias.accelerometer() - accel_bias).norm(), 1e-6);
  EXPECT_LT((anchor.bias.gyroscope() - gyro_bias).norm(), 1e-9);
}

TEST(InitialBias, RollTenDegrees)
{
  const gtsam::Vector3 accel_bias(-0.03, 0.07, 0.01);
  const gtsam::Vector3 gyro_bias(0.0005, 0.0, -0.001);
  const Anchor anchor = initialize_with({10.0 * M_PI / 180.0, 0.0, 0.4}, accel_bias, gyro_bias);

  ASSERT_TRUE(anchor.valid);
  EXPECT_LT((anchor.bias.accelerometer() - accel_bias).norm(), 1e-6);
  EXPECT_LT((anchor.bias.gyroscope() - gyro_bias).norm(), 1e-9);
}

TEST(InitialBias, PitchMinusFiveDegrees)
{
  const gtsam::Vector3 accel_bias(0.0, 0.0, 0.0);
  const gtsam::Vector3 gyro_bias(0.0, 0.0, 0.0);
  const Anchor anchor =
    initialize_with({0.0, -5.0 * M_PI / 180.0, -1.2}, accel_bias, gyro_bias);

  ASSERT_TRUE(anchor.valid);
  // With a zero true bias the estimate must be zero: a sign error in the gravity
  // convention would show up as ~9.8 or ~19.6 m/s^2 here.
  EXPECT_LT(anchor.bias.accelerometer().norm(), 1e-6);
}

TEST(InitialBias, MovingAtStartupKeepsTheConfiguredBiasWithLargerUncertainty)
{
  FactorGraphParams params;
  params.initialization.pose_source = "initialpose";
  params.initialization.bias_init_timeout_s = 0.5;
  Initializer initializer(params);
  initializer.on_initial_pose(0.0, gtsam::Pose3(), gtsam::Matrix6::Identity() * 1e-2);

  // Rotating at start-up: the static detection never triggers.
  for (double t = 0.0; t <= 1.0; t += 0.01) {
    ImuSample sample;
    sample.stamp = t;
    sample.accel = gtsam::Vector3(0.0, 0.0, params.imu.gravity);
    sample.gyro = gtsam::Vector3(0.0, 0.0, 0.5);
    initializer.on_imu(sample);
  }
  const auto first = initializer.try_initialize(0.0);
  EXPECT_FALSE(first.has_value());  // still collecting
  const auto anchor = initializer.try_initialize(2.0);  // past bias_init_timeout_s
  ASSERT_TRUE(anchor.has_value());
  EXPECT_LT(anchor->bias.gyroscope().norm(), 1e-12);  // the configured default
  // The uncertainty is doubled because the bias could not be observed.
  const double gyro_variance = anchor->covariance(12, 12);
  EXPECT_NEAR(
    gyro_variance, std::pow(2.0 * params.initialization.init_gyro_bias_stddev, 2), 1e-12);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
