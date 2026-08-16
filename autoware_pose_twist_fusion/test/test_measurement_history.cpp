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
// Test 20 of design_last.md §11: persistence of the measurement store and the
// no-double-counting invariants INV-1 / INV-3 / INV-5 (plus their exceptions).

#include "plugins/factor_graph_fusion/measurement_history.hpp"

#include <gtest/gtest.h>

using autoware::pose_twist_fusion::fg::AcceptedMeasurement;
using autoware::pose_twist_fusion::fg::Contribution;
using autoware::pose_twist_fusion::fg::Graph2MeasurementHistory;
using autoware::pose_twist_fusion::fg::MeasurementSource;

namespace
{
AcceptedMeasurement make_measurement(
  double stamp, MeasurementSource source = MeasurementSource::kNdt)
{
  AcceptedMeasurement measurement;
  measurement.stamp = stamp;
  measurement.source = source;
  measurement.nis = 1.23;
  measurement.weight = 0.87;
  measurement.corrected_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  return measurement;
}
}  // namespace

TEST(MeasurementHistory, OutOfOrderInsertionIsSorted)
{
  // Test 17: out-of-order arrivals must be resolved by the store, not by the caller.
  Graph2MeasurementHistory history(512);
  history.insert(make_measurement(10.30));
  history.insert(make_measurement(10.10));
  history.insert(make_measurement(10.40));
  history.insert(make_measurement(10.20));

  double previous = 0.0;
  for (const auto & measurement : history.measurements()) {
    EXPECT_GE(measurement.stamp, previous);
    previous = measurement.stamp;
  }
  EXPECT_EQ(history.size(), 4u);
}

TEST(MeasurementHistory, MeasurementsStayInTheWindowAcrossTicks)
{
  // (1) A measurement at 9.8 must still be re-injected in the [9.5, 10.5] window of
  // the next tick; that is what makes the batch reconstruction possible at all.
  Graph2MeasurementHistory history(512);
  history.insert(make_measurement(9.8, MeasurementSource::kGnss));
  history.insert(make_measurement(9.9, MeasurementSource::kNdt));

  auto first_tick = history.in_window(9.0, 10.0);
  ASSERT_EQ(first_tick.size(), 2u);
  for (auto * measurement : first_tick) {
    measurement->assigned_node = 5;
    measurement->has_assigned_node = true;
    measurement->contribution = Contribution::kFactorAdded;
  }

  const auto second_tick = history.in_window(9.5, 10.5);
  EXPECT_EQ(second_tick.size(), 2u);
  // INV-3: the frozen values are untouched by the re-injection.
  EXPECT_DOUBLE_EQ(second_tick.front()->nis, 1.23);
  EXPECT_DOUBLE_EQ(second_tick.front()->weight, 0.87);
}

TEST(MeasurementHistory, PruneUsesTheAssignedNodeNotTheStamp)
{
  // design §5.3.7 (3): a measurement merged into the boundary node can have
  // stamp > t_next_begin and still be absorbed, so the stamp is the wrong criterion.
  Graph2MeasurementHistory history(512);
  history.insert(make_measurement(10.0));
  history.insert(make_measurement(10.6));  // later stamp ...
  auto entries = history.in_window(9.0, 11.0);
  entries[0]->assigned_node = 3;
  entries[0]->has_assigned_node = true;
  entries[0]->contribution = Contribution::kFactorAdded;
  entries[1]->assigned_node = 4;  // ... but assigned to the boundary node
  entries[1]->has_assigned_node = true;
  entries[1]->contribution = Contribution::kFactorAdded;

  const auto report = history.prune_absorbed(4);
  EXPECT_EQ(report.removed, 2u);
  EXPECT_EQ(history.size(), 0u);
}

TEST(MeasurementHistory, KeepsMeasurementsNewerThanTheBoundary)
{
  Graph2MeasurementHistory history(512);
  history.insert(make_measurement(10.0));
  history.insert(make_measurement(10.9));
  auto entries = history.in_window(9.0, 11.0);
  entries[0]->assigned_node = 2;
  entries[0]->has_assigned_node = true;
  entries[0]->contribution = Contribution::kFactorAdded;
  entries[1]->assigned_node = 9;
  entries[1]->has_assigned_node = true;
  entries[1]->contribution = Contribution::kFactorAdded;

  const auto report = history.prune_absorbed(4);
  EXPECT_EQ(report.removed, 1u);
  ASSERT_EQ(history.size(), 1u);
  EXPECT_DOUBLE_EQ(history.measurements().front().stamp, 10.9);
}

#ifdef NDEBUG
TEST(MeasurementHistory, Inv5BlocksPruningOfNeverBuiltMeasurements)
{
  // INV-5: a measurement that never became a factor must not be dropped by the
  // prune. In a debug build this is an assert (D34), so the release path - which
  // keeps the entry and reports it - is what is verified here.
  Graph2MeasurementHistory history(512);
  history.insert(make_measurement(10.0));
  auto entries = history.in_window(9.0, 11.0);
  entries[0]->assigned_node = 1;
  entries[0]->has_assigned_node = true;
  entries[0]->contribution = Contribution::kNotYetBuilt;

  const auto report = history.prune_absorbed(4);
  EXPECT_EQ(report.removed, 0u);
  EXPECT_EQ(report.not_yet_built_blocked, 1u);
  EXPECT_EQ(history.size(), 1u);
}
#endif

TEST(MeasurementHistory, SlamInterpolationAnchorSurvivesThePrune)
{
  // design §5.3.3c: the newest SLAM sample at or below the boundary is kept as the
  // start point of the next relative pair.
  Graph2MeasurementHistory history(512);
  history.insert(make_measurement(10.0, MeasurementSource::kSlamRelative));
  history.insert(make_measurement(10.1, MeasurementSource::kSlamRelative));
  auto entries = history.in_window(9.0, 11.0);
  for (auto * entry : entries) {
    entry->has_assigned_node = true;
    entry->contribution = Contribution::kFactorAdded;
  }
  entries[0]->assigned_node = 2;
  entries[1]->assigned_node = 3;

  const auto report = history.prune_absorbed(4);
  EXPECT_EQ(report.interpolation_anchors_kept, 1u);
  ASSERT_EQ(history.size(), 1u);
  EXPECT_DOUBLE_EQ(history.measurements().front().stamp, 10.1);
  EXPECT_TRUE(history.measurements().front().is_interpolation_anchor);
}

TEST(MeasurementHistory, DropBeforeReportsTheLostMeasurements)
{
  // (a-4) is one of the two explicit INV-5 exceptions: what is dropped here is
  // genuinely lost, so the count has to be observable.
  Graph2MeasurementHistory history(512);
  history.insert(make_measurement(9.0));
  history.insert(make_measurement(9.4));
  history.insert(make_measurement(10.0));

  size_t never_built = 0;
  const size_t removed = history.drop_before(9.5, &never_built);
  EXPECT_EQ(removed, 2u);
  EXPECT_EQ(never_built, 2u);
  EXPECT_EQ(history.size(), 1u);
}

TEST(MeasurementHistory, ForcedRebuildDropsUpToTheAnchor)
{
  Graph2MeasurementHistory history(512);
  history.insert(make_measurement(9.0));
  history.insert(make_measurement(9.9));
  history.insert(make_measurement(10.4));
  size_t never_built = 0;
  EXPECT_EQ(history.drop_up_to(10.0, &never_built), 2u);
  EXPECT_EQ(never_built, 2u);
  EXPECT_EQ(history.size(), 1u);
}

TEST(MeasurementHistory, SizeCapIsEnforcedAndCounted)
{
  Graph2MeasurementHistory history(4);
  for (int i = 0; i < 10; ++i) {
    history.insert(make_measurement(10.0 + 0.1 * i));
  }
  EXPECT_EQ(history.size(), 4u);
  EXPECT_EQ(history.overflow_count(), 6u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
