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

#include "plugins/factor_graph_fusion/measurement_history.hpp"

#include <algorithm>
#include <cassert>

namespace autoware::pose_twist_fusion::fg
{

Graph2MeasurementHistory::Graph2MeasurementHistory(size_t max_size) : max_size_(max_size)
{
}

bool Graph2MeasurementHistory::insert(const AcceptedMeasurement & measurement)
{
  const auto it = std::upper_bound(
    measurements_.begin(), measurements_.end(), measurement.stamp,
    [](double stamp, const AcceptedMeasurement & m) { return stamp < m.stamp; });
  measurements_.insert(it, measurement);

  if (measurements_.size() > max_size_) {
    // Runaway protection against an abnormal input rate; must be visible in
    // diagnostics because the dropped entry is genuinely lost.
    measurements_.pop_front();
    ++overflow_count_;
    return false;
  }
  return true;
}

std::vector<AcceptedMeasurement *> Graph2MeasurementHistory::in_window(
  double t_begin, double t_end)
{
  std::vector<AcceptedMeasurement *> out;
  for (auto & m : measurements_) {
    if (m.stamp >= t_begin - 1e-9 && m.stamp <= t_end + 1e-9) {
      out.push_back(&m);
    }
  }
  return out;
}

size_t Graph2MeasurementHistory::drop_before(double t_begin, size_t * dropped_not_yet_built)
{
  size_t removed = 0;
  size_t never_built = 0;
  while (!measurements_.empty() && measurements_.front().stamp < t_begin - 1e-9) {
    if (measurements_.front().contribution == Contribution::kNotYetBuilt) {
      ++never_built;
    }
    measurements_.pop_front();
    ++removed;
  }
  if (dropped_not_yet_built != nullptr) {
    *dropped_not_yet_built = never_built;
  }
  return removed;
}

Graph2MeasurementHistory::PruneReport Graph2MeasurementHistory::prune_absorbed(
  std::uint64_t boundary_node_index)
{
  PruneReport report;

  // The most recent SLAM sample at or below the boundary is kept as the start point
  // of the next relative pair (design §5.3.3c).
  const AcceptedMeasurement * keep_anchor = nullptr;
  for (const auto & m : measurements_) {
    if (
      m.source == MeasurementSource::kSlamRelative && m.has_assigned_node &&
      m.assigned_node <= boundary_node_index) {
      keep_anchor = &m;
    }
  }

  std::deque<AcceptedMeasurement> kept;
  for (auto & m : measurements_) {
    const bool absorbed = m.has_assigned_node && m.assigned_node <= boundary_node_index;
    if (!absorbed) {
      kept.push_back(m);
      continue;
    }
    if (keep_anchor != nullptr && &m == keep_anchor) {
      AcceptedMeasurement anchor = m;
      anchor.is_interpolation_anchor = true;
      anchor.contribution = Contribution::kIntentionallyExcluded;
      kept.push_back(anchor);
      ++report.interpolation_anchors_kept;
      continue;
    }
    if (m.contribution == Contribution::kNotYetBuilt) {
      // INV-5: a measurement that never became a factor must not be dropped here.
      // Keeping it is the safe direction (it will be built in a later tick or be
      // dropped by the explicit (a-4) exception with a counter).
      assert(false && "INV-5 violated: pruning a measurement that never became a factor");
      ++report.not_yet_built_blocked;
      kept.push_back(m);
      continue;
    }
    ++report.removed;
  }
  measurements_ = std::move(kept);

  // INV-1: nothing absorbed and already built may remain.
  assert(std::none_of(measurements_.begin(), measurements_.end(), [&](const AcceptedMeasurement & m) {
    return m.has_assigned_node && m.assigned_node <= boundary_node_index &&
           m.contribution == Contribution::kFactorAdded && !m.is_interpolation_anchor;
  }));

  return report;
}

size_t Graph2MeasurementHistory::drop_up_to(double t_anchor, size_t * dropped_not_yet_built)
{
  size_t removed = 0;
  size_t never_built = 0;
  while (!measurements_.empty() && measurements_.front().stamp <= t_anchor + 1e-9) {
    if (measurements_.front().contribution == Contribution::kNotYetBuilt) {
      ++never_built;
    }
    measurements_.pop_front();
    ++removed;
  }
  if (dropped_not_yet_built != nullptr) {
    *dropped_not_yet_built = never_built;
  }
  return removed;
}

void Graph2MeasurementHistory::clear()
{
  measurements_.clear();
}

}  // namespace autoware::pose_twist_fusion::fg
