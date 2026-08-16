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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__GRAPH2_OPTIMIZER_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__GRAPH2_OPTIMIZER_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"
#include "plugins/factor_graph_fusion/fg_types.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <string>

namespace autoware::pose_twist_fusion::fg
{

/// Batch optimization of the fixed lag window (D24). The window is rebuilt from
/// scratch every tick, which is what makes the result independent of the arrival
/// order of the measurements (試験 17).
class Graph2Optimizer
{
public:
  struct Result
  {
    bool success{false};
    gtsam::Values values;
    std::string error;
    /// Anchor state at t_end and its joint 15x15 covariance (design §5.3.7 (1)).
    Anchor anchor;
    double optimize_time_ms{0.0};
  };

  explicit Graph2Optimizer(const FactorGraphParams & params);

  Result optimize(
    const gtsam::NonlinearFactorGraph & graph, const gtsam::Values & initial,
    std::uint64_t anchor_node_index, double anchor_stamp) const;

private:
  FactorGraphParams params_;
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__GRAPH2_OPTIMIZER_HPP_
