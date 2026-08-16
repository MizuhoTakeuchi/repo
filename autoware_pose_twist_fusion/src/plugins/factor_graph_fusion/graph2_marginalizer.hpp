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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__GRAPH2_MARGINALIZER_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__GRAPH2_MARGINALIZER_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstdint>
#include <string>

namespace autoware::pose_twist_fusion::fg
{

/// Marginalizes everything older than the next boundary into a single linear factor
/// (design §5.3.7 (3), D23).
///
/// Because t_next_begin is always a node (D29), the factor set splits exactly into
/// "depends only on nodes <= j*" and "depends only on nodes >= j*", so the Schur
/// complement is taken without any approximation and INV-1 / INV-2 hold by
/// construction.
class Graph2Marginalizer
{
public:
  struct Result
  {
    bool success{false};
    gtsam::NonlinearFactorGraph prior;
    std::string error;
    size_t marginalized_factors{0};
    /// INV-2 check: number of factors that straddle the boundary (must be 0).
    size_t straddling_factors{0};
  };

  explicit Graph2Marginalizer(const FactorGraphParams & params);

  Result marginalize(
    const gtsam::NonlinearFactorGraph & graph, const gtsam::Values & values,
    std::uint64_t boundary_node_index) const;

  /// INV-2 verification used by the tests and by the debug build assertions.
  static size_t count_straddling_factors(
    const gtsam::NonlinearFactorGraph & graph, std::uint64_t boundary_node_index);

private:
  FactorGraphParams params_;
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__GRAPH2_MARGINALIZER_HPP_
