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
// Test 18 of design_last.md §11: replay equivalence.
//
// (a) continuous propagation vs (b) an anchor update at t_reset with the *same*
// X*, V*, B* followed by the replay of design §5.3.7 (5). Dropping the wheel speed
// replay must break the equivalence - that is the regression this test protects.

#include "plugins/factor_graph_fusion/graph1_propagator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

using autoware::pose_twist_fusion::FactorGraphParams;
using autoware::pose_twist_fusion::fg::Anchor;
using autoware::pose_twist_fusion::fg::CovarianceCorrector;
using autoware::pose_twist_fusion::fg::Graph1Propagator;
using autoware::pose_twist_fusion::fg::ImuBuffer;
using autoware::pose_twist_fusion::fg::ImuSample;
using autoware::pose_twist_fusion::fg::TwistSample;
using autoware::pose_twist_fusion::fg::VehicleTwistBuffer;

namespace
{
constexpr double kImuPeriod = 0.01;    // 100 Hz
constexpr double kTwistPeriod = 0.02;  // 50 Hz
constexpr double kDuration = 1.0;
constexpr double kResetTime = 0.5;
constexpr double kSpeed = 10.0;
constexpr double kYawRate = 0.2;

std::shared_ptr<CovarianceCorrector> make_corrector(const FactorGraphParams & params)
{
  return std::make_shared<CovarianceCorrector>(
    params.covariance_correction, params.robust_kernel, params.robust_kernel_param);
}

ImuSample make_imu(double t, double gravity)
{
  ImuSample sample;
  sample.stamp = t;
  // Constant speed on a circle: centripetal acceleration plus gravity as specific
  // force in the body frame (the vehicle is level).
  sample.accel = gtsam::Vector3(0.0, kSpeed * kYawRate, gravity);
  sample.gyro = gtsam::Vector3(0.0, 0.0, kYawRate);
  return sample;
}

TwistSample make_twist(double t, double vx = kSpeed)
{
  TwistSample sample;
  sample.stamp = t;
  sample.vx = vx;
  sample.variance = 0.04;
  return sample;
}

Anchor make_initial_anchor()
{
  Anchor anchor;
  anchor.stamp = 0.0;
  anchor.state = gtsam::NavState(gtsam::Pose3(), gtsam::Vector3(kSpeed, 0.0, 0.0));
  anchor.bias = gtsam::imuBias::ConstantBias();
  anchor.covariance = autoware::pose_twist_fusion::fg::Matrix15::Identity() * 1e-4;
  anchor.valid = true;
  return anchor;
}

/// One entry of the merged sensor stream (IMU first on equal stamps, §5.3.7 (5)).
struct Event
{
  bool is_imu{true};
  ImuSample imu;
  TwistSample twist;
};

std::vector<Event> make_stream(double gravity, double outlier_time = -1.0)
{
  std::vector<Event> events;
  for (int i = 1; i * kImuPeriod <= kDuration + 1e-9; ++i) {
    Event event;
    event.is_imu = true;
    event.imu = make_imu(i * kImuPeriod, gravity);
    events.push_back(event);
  }
  for (int i = 1; i * kTwistPeriod <= kDuration + 1e-9; ++i) {
    const double stamp = i * kTwistPeriod;
    const bool outlier = outlier_time > 0.0 && std::abs(stamp - outlier_time) < 1e-9;
    Event event;
    event.is_imu = false;
    event.twist = make_twist(stamp, outlier ? 60.0 : kSpeed);
    events.push_back(event);
  }
  std::stable_sort(events.begin(), events.end(), [](const Event & a, const Event & b) {
    const double stamp_a = a.is_imu ? a.imu.stamp : a.twist.stamp;
    const double stamp_b = b.is_imu ? b.imu.stamp : b.twist.stamp;
    if (stamp_a != stamp_b) {
      return stamp_a < stamp_b;
    }
    return a.is_imu && !b.is_imu;  // IMU before wheel speed on ties
  });
  return events;
}

void apply(
  const Event & event, Graph1Propagator & graph1, ImuBuffer & imu_buffer,
  VehicleTwistBuffer & twist_buffer)
{
  if (event.is_imu) {
    imu_buffer.push(event.imu);
    graph1.apply_imu(event.imu);
    return;
  }
  TwistSample sample = event.twist;
  twist_buffer.push(sample);
  graph1.apply_wheel_speed(sample);
  twist_buffer.apply_gate_results({sample});
}
}  // namespace

TEST(ReplayEquivalence, AnchorUpdateWithReplayReproducesContinuousPropagation)
{
  const FactorGraphParams params;
  const auto events = make_stream(params.imu.gravity);

  // --- (a) continuous propagation, capturing the state at the reset instant ---
  ImuBuffer imu_buffer(3.5);
  VehicleTwistBuffer twist_buffer(3.5);
  Graph1Propagator continuous(params, make_corrector(params));
  continuous.reset_to_anchor(make_initial_anchor());

  Anchor anchor;
  size_t reset_index = 0;
  for (size_t i = 0; i < events.size(); ++i) {
    apply(events[i], continuous, imu_buffer, twist_buffer);
    const double stamp = events[i].is_imu ? events[i].imu.stamp : events[i].twist.stamp;
    if (!anchor.valid && stamp >= kResetTime - 1e-12) {
      // This is exactly what Graph2 hands over: X*, V*, B* and Sigma* at t_end.
      const auto snapshot = continuous.snapshot();
      anchor.stamp = snapshot.stamp;
      anchor.state = snapshot.state;
      anchor.bias = snapshot.bias;
      anchor.covariance = snapshot.covariance;
      anchor.valid = true;
      reset_index = i;
    }
  }
  ASSERT_TRUE(anchor.valid);
  const auto reference = continuous.snapshot();
  const auto reference_history = continuous.state_history_copy();

  // --- (b) the same stream, but with an anchor update + replay at t_reset ----
  ImuBuffer imu_b(3.5);
  VehicleTwistBuffer twist_b(3.5);
  Graph1Propagator replayed(params, make_corrector(params));
  replayed.reset_to_anchor(make_initial_anchor());
  for (size_t i = 0; i < events.size(); ++i) {
    apply(events[i], replayed, imu_b, twist_b);
    if (i == reset_index) {
      // Graph2 finished: reset Graph1 to the anchor and replay everything after it.
      replayed.reset_to_anchor(anchor);
      replayed.replay_from(anchor.stamp, imu_b, twist_b);
    }
  }
  const auto result = replayed.snapshot();

  EXPECT_NEAR(result.stamp, reference.stamp, 1e-12);
  EXPECT_LT(
    (result.state.pose().translation() - reference.state.pose().translation()).norm(), 1e-9);
  EXPECT_LT(
    gtsam::Rot3::Logmap(result.state.pose().rotation().between(reference.state.pose().rotation()))
      .norm(),
    1e-9);
  EXPECT_LT((result.state.v() - reference.state.v()).norm(), 1e-9);
  EXPECT_LT(
    (result.covariance - reference.covariance).norm() / reference.covariance.norm(), 1e-9);

  // Every grid point after the anchor must agree as well.
  const auto replayed_history = replayed.state_history_copy();
  for (const auto & sample : reference_history.samples()) {
    if (sample.stamp < anchor.stamp - 1e-9) {
      continue;
    }
    const auto other = replayed_history.interpolate(sample.stamp);
    ASSERT_TRUE(other.has_value()) << "missing grid point at " << sample.stamp;
    EXPECT_LT((other->pose.translation() - sample.pose.translation()).norm(), 1e-9);
    EXPECT_LT((other->pose_cov - sample.pose_cov).norm() / sample.pose_cov.norm(), 1e-9);
  }
}

TEST(ReplayEquivalence, DroppingTheWheelSpeedReplayBreaksTheEquivalence)
{
  // D15: replaying only the IMU changes the Graph1 algorithm across the anchor
  // update - the wheel speed information of the replayed interval is lost. Both
  // runs start from the *same* anchor here, so the difference is only the replay.
  const FactorGraphParams params;
  const auto events = make_stream(params.imu.gravity);

  ImuBuffer imu_buffer(3.5);
  VehicleTwistBuffer twist_buffer(3.5);
  Graph1Propagator continuous(params, make_corrector(params));
  continuous.reset_to_anchor(make_initial_anchor());
  Anchor anchor;
  for (const auto & event : events) {
    apply(event, continuous, imu_buffer, twist_buffer);
    const double stamp = event.is_imu ? event.imu.stamp : event.twist.stamp;
    if (!anchor.valid && stamp >= kResetTime - 1e-12) {
      const auto snapshot = continuous.snapshot();
      anchor.stamp = snapshot.stamp;
      anchor.state = snapshot.state;
      anchor.bias = snapshot.bias;
      anchor.covariance = snapshot.covariance;
      anchor.valid = true;
    }
  }
  ASSERT_TRUE(anchor.valid);

  Graph1Propagator full_replay(params, make_corrector(params));
  full_replay.reset_to_anchor(anchor);
  full_replay.replay_from(anchor.stamp, imu_buffer, twist_buffer);

  Graph1Propagator imu_only(params, make_corrector(params));
  imu_only.reset_to_anchor(anchor);
  for (const auto & sample : imu_buffer.since(anchor.stamp)) {
    imu_only.apply_imu(sample);  // deliberately no wheel speed
  }

  const double with_wheel = full_replay.snapshot().covariance.block<3, 3>(6, 6).trace();
  const double without_wheel = imu_only.snapshot().covariance.block<3, 3>(6, 6).trace();
  EXPECT_GT(without_wheel, with_wheel * 1.5)
    << "the wheel speed replay must actually constrain the velocity";
}

TEST(ReplayEquivalence, LayerAWheelSpeedRejectionIsReproduced)
{
  // D42 / test 18: the gate decision itself must come out identical in the replay.
  const FactorGraphParams params;
  const double outlier_time = 0.60;
  const auto events = make_stream(params.imu.gravity, outlier_time);

  ImuBuffer imu_buffer(3.5);
  VehicleTwistBuffer twist_buffer(3.5);
  Graph1Propagator continuous(params, make_corrector(params));
  continuous.reset_to_anchor(make_initial_anchor());
  Anchor anchor;
  for (const auto & event : events) {
    apply(event, continuous, imu_buffer, twist_buffer);
    const double stamp = event.is_imu ? event.imu.stamp : event.twist.stamp;
    if (!anchor.valid && stamp >= kResetTime - 1e-12) {
      const auto snapshot = continuous.snapshot();
      anchor.stamp = snapshot.stamp;
      anchor.state = snapshot.state;
      anchor.bias = snapshot.bias;
      anchor.covariance = snapshot.covariance;
      anchor.valid = true;
    }
  }
  EXPECT_EQ(continuous.wheel_speed_reject_count(), 1u);

  bool flagged = false;
  for (const auto & sample : twist_buffer.range(0.0, kDuration)) {
    if (std::abs(sample.stamp - outlier_time) < 1e-9) {
      flagged = sample.rejected;
    }
  }
  EXPECT_TRUE(flagged) << "the rejection flag must be written back to the buffer";

  Graph1Propagator replayed(params, make_corrector(params));
  replayed.reset_to_anchor(anchor);
  replayed.replay_from(anchor.stamp, imu_buffer, twist_buffer);
  EXPECT_EQ(replayed.wheel_speed_reject_count(), 1u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
