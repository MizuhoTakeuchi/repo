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
// Test 17 of design_last.md §11 (node timeline part), the wheel speed binding of
// D30 and the boundary merge rule of D34.

#include "plugins/factor_graph_fusion/graph2_window.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

using autoware::pose_twist_fusion::FactorGraphParams;
using autoware::pose_twist_fusion::fg::AcceptedMeasurement;
using autoware::pose_twist_fusion::fg::Graph2Window;
using autoware::pose_twist_fusion::fg::MeasurementSource;
using autoware::pose_twist_fusion::fg::TwistSample;

namespace
{
constexpr double kBegin = 100.0;
constexpr double kNextBegin = 100.5;
constexpr double kEnd = 101.0;

AcceptedMeasurement make_measurement(double stamp)
{
  AcceptedMeasurement measurement;
  measurement.stamp = stamp;
  measurement.source = MeasurementSource::kNdt;
  return measurement;
}

std::vector<AcceptedMeasurement *> pointers(std::vector<AcceptedMeasurement> & storage)
{
  std::vector<AcceptedMeasurement *> out;
  out.reserve(storage.size());
  for (auto & measurement : storage) {
    out.push_back(&measurement);
  }
  return out;
}
}  // namespace

TEST(Graph2Window, TimelineIsSortedAndContainsTheThreeBoundaries)
{
  const FactorGraphParams params;
  const Graph2Window builder(params);
  std::vector<AcceptedMeasurement> measurements = {
    make_measurement(100.30), make_measurement(100.10), make_measurement(100.70),
    make_measurement(100.20)};
  auto refs = pointers(measurements);

  const auto window = builder.build(kBegin, kNextBegin, kEnd, {}, refs, 0, 1);
  ASSERT_TRUE(window.valid);

  double previous = -1.0;
  for (const auto & node : window.nodes) {
    EXPECT_GT(node.stamp, previous);
    previous = node.stamp;
  }
  EXPECT_NEAR(window.nodes.front().stamp, kBegin, 1e-12);
  EXPECT_NEAR(window.nodes.back().stamp, kEnd, 1e-12);

  const auto next_begin = std::find_if(
    window.nodes.begin(), window.nodes.end(), [](const auto & node) { return node.is_next_begin; });
  ASSERT_NE(next_begin, window.nodes.end());
  EXPECT_NEAR(next_begin->stamp, kNextBegin, 1e-12);
  // D29: t_next_begin is always a node, so no chain factor can straddle it.
  EXPECT_EQ(window.next_begin_index, next_begin->index);
}

TEST(Graph2Window, ArrivalOrderDoesNotChangeTheTimeline)
{
  // Test 17 (2)/(4): the rebuilt window must not depend on the arrival order.
  const FactorGraphParams params;
  const Graph2Window builder(params);
  std::vector<AcceptedMeasurement> in_order = {
    make_measurement(100.1), make_measurement(100.2), make_measurement(100.3),
    make_measurement(100.4)};
  std::vector<AcceptedMeasurement> shuffled = {
    make_measurement(100.3), make_measurement(100.1), make_measurement(100.4),
    make_measurement(100.2)};
  auto refs_a = pointers(in_order);
  auto refs_b = pointers(shuffled);

  const auto window_a = builder.build(kBegin, kNextBegin, kEnd, {}, refs_a, 0, 1);
  const auto window_b = builder.build(kBegin, kNextBegin, kEnd, {}, refs_b, 0, 1);
  ASSERT_EQ(window_a.nodes.size(), window_b.nodes.size());
  for (size_t i = 0; i < window_a.nodes.size(); ++i) {
    EXPECT_NEAR(window_a.nodes[i].stamp, window_b.nodes[i].stamp, 1e-12);
  }
}

TEST(Graph2Window, MeasurementNearTheBoundaryGetsItsOwnNode)
{
  // D34: |stamp - t_begin| <= tolerance must NOT be absorbed by t_begin, otherwise
  // no factor is ever built from it and the prune then deletes it as "absorbed".
  const FactorGraphParams params;
  const Graph2Window builder(params);
  std::vector<AcceptedMeasurement> measurements = {make_measurement(kBegin + 0.002)};
  auto refs = pointers(measurements);

  const auto window = builder.build(kBegin, kNextBegin, kEnd, {}, refs, 0, 1);
  ASSERT_TRUE(window.valid);
  EXPECT_EQ(window.boundary_shifted, 1u);
  EXPECT_NE(measurements[0].assigned_node, window.begin_index);
  EXPECT_TRUE(measurements[0].has_assigned_node);
  // The shift is at most the merge tolerance and is compensated by extra noise.
  EXPECT_LE(std::abs(measurements[0].node_time_offset), 0.005 + 1e-9);
  EXPECT_GT(measurements[0].node_time_offset, 0.0);
}

TEST(Graph2Window, ForcedNodesSplitLongIntervals)
{
  const FactorGraphParams params;  // graph2_max_node_interval_ms = 100
  const Graph2Window builder(params);
  std::vector<AcceptedMeasurement> none;
  auto refs = pointers(none);
  const auto window = builder.build(kBegin, kNextBegin, kEnd, {}, refs, 0, 1);
  ASSERT_TRUE(window.valid);
  for (size_t i = 0; i + 1 < window.nodes.size(); ++i) {
    EXPECT_LE(window.nodes[i + 1].stamp - window.nodes[i].stamp, 0.1 + 1e-9);
  }
}

TEST(Graph2Window, BeginNodeReusesThePreviousBoundaryIndex)
{
  // The marginal prior is attached to the boundary keys of the previous tick, so
  // t_begin must keep that index while the new nodes continue the counter.
  const FactorGraphParams params;
  const Graph2Window builder(params);
  std::vector<AcceptedMeasurement> none;
  auto refs = pointers(none);
  const auto window = builder.build(kBegin, kNextBegin, kEnd, {}, refs, 7, 20);
  ASSERT_TRUE(window.valid);
  EXPECT_EQ(window.begin_index, 7u);
  EXPECT_GE(window.next_begin_index, 20u);
  // indices increase with time, which is what the INV-1 / INV-2 comparisons need
  std::uint64_t previous = 0;
  for (size_t i = 0; i < window.nodes.size(); ++i) {
    if (i > 0) {
      EXPECT_GT(window.nodes[i].index, previous);
    }
    previous = window.nodes[i].index;
  }
  EXPECT_GE(window.next_free_index, window.end_index);
}

TEST(Graph2Window, WheelSpeedBindsOneSamplePerNode)
{
  // D30: a sample is used by at most one node and a node takes at most one sample.
  const FactorGraphParams params;
  const Graph2Window builder(params);
  std::vector<AcceptedMeasurement> none;
  auto refs = pointers(none);
  const auto window = builder.build(kBegin, kNextBegin, kEnd, {}, refs, 0, 1);

  std::vector<TwistSample> samples;
  for (double t = kBegin; t <= kEnd; t += 0.02) {  // 50 Hz
    TwistSample sample;
    sample.stamp = t;
    sample.vx = 8.0;
    sample.variance = 0.04;
    samples.push_back(sample);
  }

  const auto bindings = builder.bind_wheel_speed(window, samples);
  std::vector<std::uint64_t> used_nodes;
  for (const auto & binding : bindings) {
    EXPECT_EQ(
      std::count(used_nodes.begin(), used_nodes.end(), binding.node_index), 0);
    used_nodes.push_back(binding.node_index);
    EXPECT_NE(binding.node_index, window.begin_index);  // boundary node gets none
    EXPECT_GE(binding.variance, 0.04);                  // time offset noise added
  }
  EXPECT_FALSE(bindings.empty());
  EXPECT_LE(bindings.size(), window.nodes.size());
}

TEST(Graph2Window, RejectedWheelSpeedSamplesAreNotBound)
{
  // D42: a sample rejected by the layer A gate inside Graph1 must not reach Graph2.
  const FactorGraphParams params;
  const Graph2Window builder(params);
  std::vector<AcceptedMeasurement> none;
  auto refs = pointers(none);
  const auto window = builder.build(kBegin, kNextBegin, kEnd, {}, refs, 0, 1);

  std::vector<TwistSample> samples;
  for (double t = kBegin; t <= kEnd; t += 0.02) {
    TwistSample sample;
    sample.stamp = t;
    sample.vx = 8.0;
    sample.rejected = true;
    sample.gate_evaluated = true;
    samples.push_back(sample);
  }
  EXPECT_TRUE(builder.bind_wheel_speed(window, samples).empty());
}

TEST(Graph2Window, WheelSpeedBindingCanBeDisabled)
{
  FactorGraphParams params;
  params.wheel_speed.graph2_binding = "none";  // the conservative escape hatch
  const Graph2Window builder(params);
  std::vector<AcceptedMeasurement> none;
  auto refs = pointers(none);
  const auto window = builder.build(kBegin, kNextBegin, kEnd, {}, refs, 0, 1);
  std::vector<TwistSample> samples(10);
  double t = kBegin;
  for (auto & sample : samples) {
    sample.stamp = t;
    sample.vx = 5.0;
    t += 0.02;
  }
  EXPECT_TRUE(builder.bind_wheel_speed(window, samples).empty());
}

TEST(Graph2Window, DegenerateWindowIsRejected)
{
  const FactorGraphParams params;
  const Graph2Window builder(params);
  std::vector<AcceptedMeasurement> none;
  auto refs = pointers(none);
  // t_next_begin must lie strictly between t_begin and t_end (§6.2).
  EXPECT_FALSE(builder.build(kBegin, kBegin, kEnd, {}, refs, 0, 1).valid);
  EXPECT_FALSE(builder.build(kBegin, kEnd, kEnd, {}, refs, 0, 1).valid);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
