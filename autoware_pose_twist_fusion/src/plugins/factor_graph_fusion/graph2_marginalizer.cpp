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

#include "plugins/factor_graph_fusion/graph2_marginalizer.hpp"

#include "plugins/factor_graph_fusion/factors/factor_builders.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cassert>
#include <set>
#include <utility>

namespace autoware::pose_twist_fusion::fg
{

namespace
{
std::uint64_t node_index_of(gtsam::Key key)
{
  return gtsam::Symbol(key).index();
}

struct FactorSpan
{
  std::uint64_t min_index{0};
  std::uint64_t max_index{0};
};

FactorSpan span_of(const gtsam::NonlinearFactor & factor)
{
  FactorSpan span;
  bool first = true;
  for (const gtsam::Key key : factor.keys()) {
    const std::uint64_t index = node_index_of(key);
    if (first) {
      span.min_index = index;
      span.max_index = index;
      first = false;
    } else {
      span.min_index = std::min(span.min_index, index);
      span.max_index = std::max(span.max_index, index);
    }
  }
  return span;
}
}  // namespace

Graph2Marginalizer::Graph2Marginalizer(const FactorGraphParams & params) : params_(params)
{
}

size_t Graph2Marginalizer::count_straddling_factors(
  const gtsam::NonlinearFactorGraph & graph, std::uint64_t boundary_node_index)
{
  size_t straddling = 0;
  for (const auto & factor : graph) {
    if (!factor) {
      continue;
    }
    const FactorSpan span = span_of(*factor);
    if (span.min_index < boundary_node_index && span.max_index > boundary_node_index) {
      ++straddling;
    }
  }
  return straddling;
}

Graph2Marginalizer::Result Graph2Marginalizer::marginalize(
  const gtsam::NonlinearFactorGraph & graph, const gtsam::Values & values,
  std::uint64_t boundary_node_index) const
{
  Result result;

  gtsam::NonlinearFactorGraph old_part;
  for (const auto & factor : graph) {
    if (!factor) {
      continue;
    }
    const FactorSpan span = span_of(*factor);
    if (span.max_index <= boundary_node_index) {
      old_part.push_back(factor);
    } else if (span.min_index < boundary_node_index) {
      ++result.straddling_factors;
    }
  }
  // INV-2: with the forced node at t_next_begin no factor can straddle the boundary.
  assert(result.straddling_factors == 0 && "INV-2 violated: factor straddles the marginalization boundary");
  result.marginalized_factors = old_part.size();

  if (old_part.empty()) {
    result.success = true;
    return result;
  }

  const gtsam::Key xk = pose_key(boundary_node_index);
  const gtsam::Key vk = velocity_key(boundary_node_index);
  const gtsam::Key bk = bias_key(boundary_node_index);

  gtsam::Values boundary_values;
  if (!values.exists(xk) || !values.exists(vk) || !values.exists(bk)) {
    result.error = "boundary node is missing from the solution";
    return result;
  }
  boundary_values.insert(xk, values.at<gtsam::Pose3>(xk));
  boundary_values.insert(vk, values.at<gtsam::Vector3>(vk));
  boundary_values.insert(bk, values.at<gtsam::imuBias::ConstantBias>(bk));

  try {
    const gtsam::GaussianFactorGraph::shared_ptr linear = old_part.linearize(values);

    std::set<gtsam::Key> keys_to_eliminate;
    for (const gtsam::Key key : linear->keys()) {
      if (key != xk && key != vk && key != bk) {
        keys_to_eliminate.insert(key);
      }
    }

    gtsam::GaussianFactorGraph::shared_ptr remaining;
    if (keys_to_eliminate.empty()) {
      remaining = linear;
    } else {
      gtsam::Ordering ordering;
      for (const gtsam::Key key : keys_to_eliminate) {
        ordering.push_back(key);
      }
      const auto eliminated = linear->eliminatePartialSequential(ordering);
      remaining = eliminated.second;
    }

    if (params_.graph2.marginalization == "prior_triple") {
      // Simplification that drops the cross correlations; it is not conservative,
      // so the same Cauchy-Schwarz inflation as in D10 is applied (D33).
      const auto bayes_tree = remaining->eliminateMultifrontal();
      const double inflation = params_.reset.cross_covariance_inflation;
      result.prior.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        xk, boundary_values.at<gtsam::Pose3>(xk),
        gtsam::noiseModel::Gaussian::Covariance(inflation * bayes_tree->marginalCovariance(xk)));
      result.prior.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        vk, boundary_values.at<gtsam::Vector3>(vk),
        gtsam::noiseModel::Gaussian::Covariance(inflation * bayes_tree->marginalCovariance(vk)));
      result.prior.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
        bk, boundary_values.at<gtsam::imuBias::ConstantBias>(bk),
        gtsam::noiseModel::Gaussian::Covariance(inflation * bayes_tree->marginalCovariance(bk)));
    } else {
      // Default: keep the correlations by carrying the linear factors themselves.
      for (const auto & factor : *remaining) {
        if (factor) {
          result.prior.emplace_shared<gtsam::LinearContainerFactor>(factor, boundary_values);
        }
      }
    }
    result.success = true;
  } catch (const std::exception & e) {
    result.error = std::string("marginalization failed: ") + e.what();
    result.success = false;
  }
  return result;
}

}  // namespace autoware::pose_twist_fusion::fg
