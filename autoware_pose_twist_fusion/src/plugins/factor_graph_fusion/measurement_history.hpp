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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__MEASUREMENT_HISTORY_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__MEASUREMENT_HISTORY_HPP_

#include "plugins/factor_graph_fusion/fg_types.hpp"

#include <cstdint>
#include <deque>
#include <vector>

namespace autoware::pose_twist_fusion::fg
{

/// The single source of truth for the measurements inside the fixed lag window
/// (design §5.3.2a / D28). Every tick rebuilds all factors from here, which is what
/// makes the batch reconstruction independent of arrival order.
///
/// Owned exclusively by the Graph2 worker thread, therefore lock free. The
/// identifier is deliberately `measurement_history_`, never `state_history_`.
class Graph2MeasurementHistory
{
public:
  struct PruneReport
  {
    size_t removed{0};
    /// INV-5 violations: entries that would have been pruned although they never
    /// became a factor. They are **not** removed; they are reported instead.
    size_t not_yet_built_blocked{0};
    size_t interpolation_anchors_kept{0};
  };

  explicit Graph2MeasurementHistory(size_t max_size);

  /// Inserts keeping ascending stamp order (out-of-order arrivals are sorted here).
  /// Returns false when the size cap forced the oldest entry out.
  bool insert(const AcceptedMeasurement & measurement);

  /// Entries with t_begin <= stamp <= t_end, in stamp order. The returned pointers
  /// stay valid until the next insert / prune.
  [[nodiscard]] std::vector<AcceptedMeasurement *> in_window(double t_begin, double t_end);

  /// Step (a-4): entries older than the window boundary are lost for good. This is
  /// one of the two explicit exceptions to INV-5, so the count is reported and the
  /// caller must surface it as a diagnostics counter.
  size_t drop_before(double t_begin, size_t * dropped_not_yet_built);

  /// Step (g): drop what the boundary prior has absorbed.
  /// The decision is made on the assigned node index, never on the stamp, because a
  /// measurement merged into the boundary node can have stamp > t_next_begin (D29).
  PruneReport prune_absorbed(std::uint64_t boundary_node_index);

  /// Forced window rebuild after `optimizer_failure_limit` failures: everything up
  /// to the anchor is already reflected in it (INV-1). Second explicit exception to
  /// INV-5, so the number of never-built entries removed is reported.
  size_t drop_up_to(double t_anchor, size_t * dropped_not_yet_built);

  void clear();
  [[nodiscard]] size_t size() const { return measurements_.size(); }
  [[nodiscard]] bool empty() const { return measurements_.empty(); }
  [[nodiscard]] size_t overflow_count() const { return overflow_count_; }
  [[nodiscard]] const std::deque<AcceptedMeasurement> & measurements() const
  {
    return measurements_;
  }

private:
  std::deque<AcceptedMeasurement> measurements_;
  size_t max_size_;
  size_t overflow_count_{0};
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__MEASUREMENT_HISTORY_HPP_
