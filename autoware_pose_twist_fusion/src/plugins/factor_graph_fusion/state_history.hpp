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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__STATE_HISTORY_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__STATE_HISTORY_HPP_

#include "plugins/factor_graph_fusion/fg_types.hpp"

#include <deque>
#include <optional>

namespace autoware::pose_twist_fusion::fg
{

/// Graph1 state at the 20 ms grid, kept for `history_length_ms` (D12).
///
/// Not thread safe on its own: it is owned by Graph1Propagator and therefore
/// protected by `graph1_mutex_` (design §4.3, and the identifier is deliberately
/// `state_history_` to keep it apart from `measurement_history_`).
class StateHistory
{
public:
  StateHistory() = default;
  explicit StateHistory(double length_s);

  void push(const StateSample & sample);

  /// Drops every sample newer than `t`; used before the replay rebuilds them
  /// (design §5.3.4: samples older than the anchor are kept).
  void erase_after(double t);

  void clear();

  /// mu_1(t), Sigma_1(t) by SE(3) interpolation between the two neighbouring grid
  /// points. The covariance is interpolated linearly, which is a first order
  /// approximation and **not** a conservative bound (design §5.3.4).
  [[nodiscard]] std::optional<StateSample> interpolate(double t) const;

  [[nodiscard]] bool empty() const { return samples_.empty(); }
  [[nodiscard]] size_t size() const { return samples_.size(); }
  [[nodiscard]] double oldest_stamp() const;
  [[nodiscard]] double latest_stamp() const;
  [[nodiscard]] const std::deque<StateSample> & samples() const { return samples_; }

private:
  std::deque<StateSample> samples_;
  double length_s_{1.5};
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__STATE_HISTORY_HPP_
