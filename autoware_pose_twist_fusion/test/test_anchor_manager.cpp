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
// Tests 3 and 20 of design_last.md §11 (integration part): one full Graph2 tick -
// window construction, batch optimization, marginalization, history prune, anchor
// hand-over and Graph1 replay - plus the invariants INV-1 / INV-2 / INV-5.

#include "plugins/factor_graph_fusion/anchor_manager.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using autoware::pose_twist_fusion::FactorGraphParams;
namespace fg = autoware::pose_twist_fusion::fg;

namespace
{
constexpr double kSpeed = 10.0;
constexpr double kImuPeriod = 0.01;
constexpr double kTwistPeriod = 0.02;
constexpr double kNdtPeriod = 0.1;

FactorGraphParams make_params()
{
  FactorGraphParams params;
  params.slam.use = false;  // the required input configuration (RC-3)
  return params;
}

gtsam::Pose3 truth_pose(double t)
{
  return gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(kSpeed * t, 0.0, 0.0));
}

/// Test fixture that owns the whole factor_graph_fusion back end.
struct Fixture
{
  explicit Fixture(const FactorGraphParams & parameters)
  : params(parameters),
    corrector(std::make_shared<fg::CovarianceCorrector>(
      parameters.covariance_correction, parameters.robust_kernel,
      parameters.robust_kernel_param)),
    imu_buffer(parameters.buffer.imu_buffer_ms * 1e-3),
    twist_buffer(parameters.buffer.vehicle_twist_buffer_ms * 1e-3),
    graph1(parameters, corrector),
    manager(
      parameters, graph1, imu_buffer, twist_buffer, pending, gtsam::Point3(0.0, 0.0, 0.0))
  {
  }

  void start(double t0)
  {
    fg::Anchor anchor;
    anchor.stamp = t0;
    anchor.state = gtsam::NavState(truth_pose(t0), gtsam::Vector3(kSpeed, 0.0, 0.0));
    anchor.bias = gtsam::imuBias::ConstantBias();
    anchor.covariance = fg::Matrix15::Identity() * 1e-4;
    anchor.valid = true;
    manager.set_initial_anchor(anchor);
  }

  /// Drives the sensors from `from` to `to` and pushes NDT measurements.
  void drive(double from, double to)
  {
    for (double t = from + kImuPeriod; t <= to + 1e-9; t += kImuPeriod) {
      fg::ImuSample sample;
      sample.stamp = t;
      sample.accel = gtsam::Vector3(0.0, 0.0, params.imu.gravity);
      sample.gyro = gtsam::Vector3::Zero();
      imu_buffer.push(sample);
      graph1.apply_imu(sample);

      if (std::fmod(t + 1e-9, kTwistPeriod) < 1e-6) {
        fg::TwistSample twist;
        twist.stamp = t;
        twist.vx = kSpeed;
        twist.variance = 0.04;
        twist_buffer.push(twist);
        graph1.apply_wheel_speed(twist);
        twist_buffer.apply_gate_results({twist});
      }
      if (std::fmod(t + 1e-9, kNdtPeriod) < 1e-6) {
        push_ndt(t);
      }
    }
  }

  void push_ndt(double stamp)
  {
    fg::AcceptedMeasurement measurement;
    measurement.stamp = stamp;
    measurement.source = fg::MeasurementSource::kNdt;
    measurement.value.pose = truth_pose(stamp);
    measurement.corrected_covariance = Eigen::MatrixXd::Identity(6, 6) * 4e-4;
    measurement.noise = gtsam::noiseModel::Gaussian::Covariance(measurement.corrected_covariance);
    measurement.nis = 0.5;
    measurement.weight = 1.0;
    pending.push(measurement);
  }

  FactorGraphParams params;
  std::shared_ptr<fg::CovarianceCorrector> corrector;
  fg::ImuBuffer imu_buffer;
  fg::VehicleTwistBuffer twist_buffer;
  fg::Graph1Propagator graph1;
  fg::PendingMeasurementQueue pending;
  fg::AnchorManager manager;
};
}  // namespace

TEST(AnchorManager, OneTickProducesAnAnchorAndAdvancesTheWindow)
{
  Fixture fixture(make_params());
  fixture.start(0.0);
  const auto generation_before = fixture.manager.generation();
  fixture.drive(0.0, 1.5);

  fixture.manager.run_tick();
  const auto stats = fixture.manager.stats();

  EXPECT_EQ(stats.ticks, 1u) << stats.last_error;
  EXPECT_EQ(stats.optimizer_failures, 0u);
  EXPECT_GT(stats.node_count, 3u);
  // INV-2: with the forced node at t_next_begin nothing straddles the boundary.
  EXPECT_EQ(stats.straddling_factors, 0u);
  // INV-5: nothing was pruned before it ever became a factor.
  EXPECT_EQ(stats.inv5_blocked, 0u);
  EXPECT_EQ(stats.dropped_not_yet_built, 0u);

  // t_begin moved to t_next_begin = t_end + interval - lag = 1.5 + 0.5 - 1.0
  EXPECT_NEAR(fixture.manager.window_begin(), 1.0, 1e-6);
  EXPECT_GT(fixture.manager.generation(), generation_before);

  // The anchor is the window's right edge and Graph1 was reset onto it.
  const auto anchor = fixture.manager.last_anchor();
  ASSERT_TRUE(anchor.valid);
  EXPECT_NEAR(anchor.stamp, stats.t_end, 1e-9);
  EXPECT_LT((anchor.state.pose().translation() - truth_pose(anchor.stamp).translation()).norm(), 0.2);
  EXPECT_NEAR(anchor.state.v().x(), kSpeed, 0.5);

  // Test 3: the anchor update must not move the estimate by more than the immediate
  // application threshold, since the measurements are consistent here.
  const auto snapshot = fixture.graph1.snapshot();
  EXPECT_LT((snapshot.state.pose().translation() - anchor.state.pose().translation()).norm(), 1.0);
  EXPECT_TRUE(snapshot.covariance.allFinite());
}

TEST(AnchorManager, MeasurementsSurviveUntilTheyAreAbsorbed)
{
  // Test 20 (1): a measurement inside the window is re-injected every tick until the
  // marginal prior absorbs it, and only then is it pruned.
  Fixture fixture(make_params());
  fixture.start(0.0);
  fixture.drive(0.0, 1.5);
  fixture.manager.run_tick();
  const auto after_first = fixture.manager.stats();
  EXPECT_GT(after_first.history_size, 0u);

  fixture.drive(1.5, 2.0);
  fixture.manager.run_tick();
  const auto after_second = fixture.manager.stats();

  EXPECT_EQ(after_second.ticks, 2u) << after_second.last_error;
  EXPECT_EQ(after_second.straddling_factors, 0u);
  EXPECT_EQ(after_second.inv5_blocked, 0u);
  // The window advanced by one optimize interval.
  EXPECT_NEAR(fixture.manager.window_begin(), 1.5, 1e-6);
  // Old measurements were absorbed rather than accumulating without bound.
  EXPECT_LE(after_second.history_size, after_first.history_size + 5);
}

TEST(AnchorManager, TickIsSkippedWhileTheWindowIsTooShort)
{
  // Right after the initialization t_next_begin is still behind t_begin; skipping
  // keeps the boundary well defined instead of building a degenerate window.
  Fixture fixture(make_params());
  fixture.start(0.0);
  fixture.drive(0.0, 0.3);
  fixture.manager.run_tick();
  const auto stats = fixture.manager.stats();
  EXPECT_EQ(stats.ticks, 0u);
  EXPECT_GE(stats.skipped_ticks, 1u);
  EXPECT_NEAR(fixture.manager.window_begin(), 0.0, 1e-9);
}

TEST(AnchorManager, ResetClearsTheWindowState)
{
  Fixture fixture(make_params());
  fixture.start(0.0);
  fixture.drive(0.0, 1.5);
  fixture.manager.run_tick();
  ASSERT_EQ(fixture.manager.stats().ticks, 1u);

  fixture.manager.reset();
  EXPECT_NEAR(fixture.manager.window_begin(), 0.0, 1e-12);
  EXPECT_EQ(fixture.manager.previous_snapshot(), nullptr);
  fixture.manager.run_tick();  // no anchor: must be a no-op, not a crash
  EXPECT_EQ(fixture.manager.stats().ticks, 1u);
}

TEST(AnchorManager, WorkerThreadRunsTicksOffTheCallbackPath)
{
  // D17: the optimization must not run inside a ROS callback. The worker is driven
  // by request_tick(), which never blocks the caller.
  Fixture fixture(make_params());
  fixture.start(0.0);
  fixture.drive(0.0, 1.5);

  fixture.manager.start();
  fixture.manager.request_tick();
  for (int i = 0; i < 200 && fixture.manager.stats().ticks == 0u; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  fixture.manager.stop();
  EXPECT_EQ(fixture.manager.stats().ticks, 1u) << fixture.manager.stats().last_error;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
