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
// Test 4 of design_last.md §11 (unit part): delayed measurements are located by
// SE(3) interpolation in the Graph1 state history.

#include "plugins/factor_graph_fusion/state_history.hpp"

#include <gtest/gtest.h>

using autoware::pose_twist_fusion::fg::StateHistory;
using autoware::pose_twist_fusion::fg::StateSample;

namespace
{
StateSample make_sample(double t)
{
  StateSample sample;
  sample.stamp = t;
  // 10 m/s along x with a 0.5 rad/s yaw rate.
  sample.pose = gtsam::Pose3(gtsam::Rot3::Yaw(0.5 * t), gtsam::Point3(10.0 * t, 0.0, 0.0));
  sample.velocity = gtsam::Vector3(10.0, 0.0, 0.0);
  sample.pose_cov = gtsam::Matrix6::Identity() * (1e-4 * (1.0 + t));
  sample.vel_cov = gtsam::Matrix3::Identity() * 1e-3;
  return sample;
}

StateHistory filled_history(double length_s = 1.5, double step = 0.02, double duration = 1.0)
{
  StateHistory history(length_s);
  for (double t = 0.0; t <= duration + 1e-9; t += step) {
    history.push(make_sample(t));
  }
  return history;
}
}  // namespace

TEST(StateHistory, InterpolatesBetweenGridPoints)
{
  const StateHistory history = filled_history();
  const auto sample = history.interpolate(0.51);
  ASSERT_TRUE(sample.has_value());
  EXPECT_NEAR(sample->stamp, 0.51, 1e-12);
  EXPECT_NEAR(sample->pose.translation().x(), 5.1, 1e-6);
  EXPECT_NEAR(sample->pose.rotation().yaw(), 0.255, 1e-6);
  // Linear covariance interpolation: a first order approximation, PSD preserved.
  EXPECT_GT(sample->pose_cov(0, 0), 1e-4);
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> solver(sample->pose_cov);
  EXPECT_GE(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(StateHistory, RejectsStampsOutsideTheHistory)
{
  const StateHistory history = filled_history();
  EXPECT_FALSE(history.interpolate(-0.5).has_value());
  EXPECT_FALSE(history.interpolate(2.0).has_value());
}

TEST(StateHistory, RetentionDropsOldSamples)
{
  const StateHistory history = filled_history(0.2, 0.02, 1.0);
  EXPECT_GE(history.oldest_stamp(), 0.79);
  EXPECT_NEAR(history.latest_stamp(), 1.0, 1e-9);
}

TEST(StateHistory, EraseAfterKeepsSamplesOlderThanTheAnchor)
{
  // design §5.3.4: the replay only rebuilds samples newer than the anchor, so a
  // delayed measurement arriving right after an anchor update can still be located.
  StateHistory history = filled_history();
  history.erase_after(0.5);
  EXPECT_NEAR(history.latest_stamp(), 0.5, 1e-9);
  EXPECT_TRUE(history.interpolate(0.3).has_value());
  EXPECT_FALSE(history.interpolate(0.7).has_value());
}

TEST(StateHistory, PushOverwritesRewrittenGridPoints)
{
  // The replay pushes the same grid times again; they must replace, not duplicate.
  StateHistory history = filled_history();
  const size_t before = history.size();
  history.erase_after(0.5);
  for (double t = 0.52; t <= 1.0 + 1e-9; t += 0.02) {
    history.push(make_sample(t));
  }
  EXPECT_EQ(history.size(), before);
  const auto sample = history.interpolate(0.9);
  ASSERT_TRUE(sample.has_value());
  EXPECT_NEAR(sample->pose.translation().x(), 9.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
