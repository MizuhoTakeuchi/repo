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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__GRAPH2_WINDOW_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__GRAPH2_WINDOW_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"
#include "plugins/factor_graph_fusion/fg_types.hpp"

#include <cstdint>
#include <vector>

namespace autoware::pose_twist_fusion::fg
{

struct WindowNode
{
  double stamp{0.0};
  std::uint64_t index{0};
  bool is_begin{false};        //< carries the boundary prior (t_begin = t_prior_)
  bool is_next_begin{false};   //< j*, the boundary of the next tick (forced, D29)
  bool is_end{false};          //< anchor node t_end
  bool from_measurement{false};
};

struct WheelBinding
{
  std::uint64_t node_index{0};
  double stamp{0.0};
  double vx{0.0};
  double variance{0.04};
};

/// Node timeline of the fixed lag window (design §5.3.2 (b), D23 / D29 / D34).
class Graph2Window
{
public:
  struct BuildResult
  {
    std::vector<WindowNode> nodes;
    std::uint64_t begin_index{0};
    std::uint64_t next_begin_index{0};
    std::uint64_t end_index{0};
    /// Forced nodes removed because of `graph2_max_nodes` (diagnostics WARN).
    size_t thinned_nodes{0};
    /// Measurements moved off t_begin so they cannot be silently absorbed (D34).
    size_t boundary_shifted{0};
    /// First index that the next tick may hand out.
    std::uint64_t next_free_index{0};
    bool valid{false};
  };

  explicit Graph2Window(const FactorGraphParams & params);

  /// Builds the timeline and assigns every measurement to a node.
  /// `measurements` is modified: `assigned_node` and `node_time_offset` are set.
  ///
  /// Node indices are handed out from a monotonically increasing counter, and the
  /// t_begin node **reuses the index that t_next_begin had in the previous tick**.
  /// That keeps the marginal prior attached to the same keys across ticks and keeps
  /// the index order identical to the time order, which is what the INV-1 / INV-2
  /// checks (`assigned_node <= j*`) rely on.
  BuildResult build(
    double t_begin, double t_next_begin, double t_end,
    const std::vector<double> & previous_node_stamps,
    const std::vector<AcceptedMeasurement *> & measurements, std::uint64_t begin_index,
    std::uint64_t first_free_index) const;

  /// Wheel speed to node assignment of design §5.3.3a' (D30): every sample goes to
  /// its nearest node, each node keeps at most the closest sample, samples further
  /// away than `graph2_max_time_offset_ms` are dropped, and the boundary node gets
  /// none (its information is already inside the prior).
  [[nodiscard]] std::vector<WheelBinding> bind_wheel_speed(
    const BuildResult & window, const std::vector<TwistSample> & samples) const;

private:
  FactorGraphParams params_;
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__GRAPH2_WINDOW_HPP_
