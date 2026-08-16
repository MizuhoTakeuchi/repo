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

#include "plugins/factor_graph_fusion/graph2_window.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

namespace autoware::pose_twist_fusion::fg
{

namespace
{
struct Candidate
{
  double stamp{0.0};
  bool is_begin{false};
  bool is_next_begin{false};
  bool is_end{false};
  bool from_measurement{false};
};
}  // namespace

Graph2Window::Graph2Window(const FactorGraphParams & params) : params_(params)
{
}

Graph2Window::BuildResult Graph2Window::build(
  double t_begin, double t_next_begin, double t_end,
  const std::vector<double> & previous_node_stamps,
  const std::vector<AcceptedMeasurement *> & measurements, std::uint64_t begin_index,
  std::uint64_t first_free_index) const
{
  BuildResult result;
  if (!(t_begin < t_next_begin && t_next_begin < t_end)) {
    // §6.2 guarantees this ordering through the parameter validation; if it is
    // violated at run time the window is degenerate and must not be built.
    return result;
  }

  const double tolerance = params_.node_merge_tolerance_ms * 1e-3;
  const double max_interval = params_.graph2_max_node_interval_ms * 1e-3;

  std::vector<Candidate> candidates;
  candidates.push_back({t_begin, true, false, false, false});
  // D29: t_next_begin is always a node. Without it a chain factor could straddle
  // the next marginalization boundary, and then neither keeping nor marginalizing
  // it is correct (INV-2).
  candidates.push_back({t_next_begin, false, true, false, false});
  candidates.push_back({t_end, false, false, true, false});

  for (const double stamp : previous_node_stamps) {
    if (stamp > t_begin + tolerance && stamp < t_end - tolerance) {
      candidates.push_back({stamp, false, false, false, false});
    }
  }
  for (const auto * measurement : measurements) {
    double stamp = measurement->stamp;
    if (stamp < t_begin - 1e-9 || stamp > t_end + 1e-9) {
      continue;
    }
    if (stamp <= t_begin + tolerance) {
      // D34: t_begin must never absorb a measurement, otherwise the measurement is
      // pruned as "absorbed" although no factor was ever built from it.
      stamp = t_begin + tolerance;
      ++result.boundary_shifted;
    }
    candidates.push_back({stamp, false, false, false, true});
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
    return a.stamp < b.stamp;
  });

  // Merge candidates that are closer than the tolerance. Protected candidates
  // (begin / next_begin / end) keep their exact time.
  std::vector<Candidate> merged;
  for (const auto & candidate : candidates) {
    if (merged.empty()) {
      merged.push_back(candidate);
      continue;
    }
    Candidate & last = merged.back();
    if (candidate.stamp - last.stamp <= tolerance) {
      if (last.is_begin) {
        // never merge into the boundary node
        merged.push_back(candidate);
        continue;
      }
      last.is_next_begin = last.is_next_begin || candidate.is_next_begin;
      last.is_end = last.is_end || candidate.is_end;
      last.from_measurement = last.from_measurement || candidate.from_measurement;
      if (candidate.is_next_begin || candidate.is_end) {
        last.stamp = candidate.stamp;  // protected times win
      }
      continue;
    }
    merged.push_back(candidate);
  }

  // Forced nodes so that no interval exceeds graph2_max_node_interval_ms.
  std::vector<Candidate> with_forced;
  for (size_t i = 0; i < merged.size(); ++i) {
    with_forced.push_back(merged[i]);
    if (i + 1 >= merged.size()) {
      break;
    }
    const double gap = merged[i + 1].stamp - merged[i].stamp;
    if (gap <= max_interval) {
      continue;
    }
    const int splits = static_cast<int>(std::ceil(gap / max_interval)) - 1;
    for (int s = 1; s <= splits; ++s) {
      Candidate forced;
      forced.stamp = merged[i].stamp + gap * s / (splits + 1);
      with_forced.push_back(forced);
    }
  }

  // Cap the node count by thinning forced nodes only (measurement and protected
  // nodes are kept) - runaway protection of design §5.3.2.
  const size_t max_nodes = static_cast<size_t>(std::max(params_.graph2_max_nodes, 3));
  if (with_forced.size() > max_nodes) {
    std::vector<Candidate> thinned;
    thinned.reserve(with_forced.size());
    size_t removable = with_forced.size() - max_nodes;
    for (const auto & candidate : with_forced) {
      const bool protectable =
        candidate.is_begin || candidate.is_next_begin || candidate.is_end ||
        candidate.from_measurement;
      if (!protectable && removable > 0) {
        --removable;
        ++result.thinned_nodes;
        continue;
      }
      thinned.push_back(candidate);
    }
    with_forced.swap(thinned);
  }

  result.nodes.reserve(with_forced.size());
  std::uint64_t next_index = std::max(first_free_index, begin_index + 1);
  for (size_t i = 0; i < with_forced.size(); ++i) {
    WindowNode node;
    node.stamp = with_forced[i].stamp;
    node.index = with_forced[i].is_begin ? begin_index : next_index++;
    node.is_begin = with_forced[i].is_begin;
    node.is_next_begin = with_forced[i].is_next_begin;
    node.is_end = with_forced[i].is_end;
    node.from_measurement = with_forced[i].from_measurement;
    if (node.is_begin) {
      result.begin_index = node.index;
    }
    if (node.is_next_begin) {
      result.next_begin_index = node.index;
    }
    if (node.is_end) {
      result.end_index = node.index;
    }
    result.nodes.push_back(node);
  }
  result.next_free_index = next_index;

  // Assign every measurement to its nearest node.
  for (auto * measurement : measurements) {
    if (measurement->stamp < t_begin - 1e-9 || measurement->stamp > t_end + 1e-9) {
      measurement->has_assigned_node = false;
      continue;
    }
    double best_distance = std::numeric_limits<double>::max();
    const WindowNode * best_node = nullptr;
    for (const auto & node : result.nodes) {
      if (node.is_begin) {
        continue;  // D34: the boundary node never receives a unary measurement
      }
      const double distance = std::abs(node.stamp - measurement->stamp);
      if (distance < best_distance) {
        best_distance = distance;
        best_node = &node;
      }
    }
    if (best_node == nullptr) {
      measurement->has_assigned_node = false;
      continue;
    }
    measurement->assigned_node = best_node->index;
    measurement->has_assigned_node = true;
    measurement->node_time_offset = best_node->stamp - measurement->stamp;
  }

  result.valid = result.nodes.size() >= 3;
  return result;
}

std::vector<WheelBinding> Graph2Window::bind_wheel_speed(
  const BuildResult & window, const std::vector<TwistSample> & samples) const
{
  std::vector<WheelBinding> bindings;
  if (!window.valid || params_.wheel_speed.graph2_binding == "none") {
    return bindings;
  }

  const double max_offset = params_.wheel_speed.graph2_max_time_offset_ms * 1e-3;
  // node index -> best (distance, sample)
  std::map<std::uint64_t, std::pair<double, const TwistSample *>> best;

  for (const auto & sample : samples) {
    // D42: a sample rejected by the layer A gate in Graph1 is not used by Graph2.
    if (sample.rejected) {
      continue;
    }
    double best_distance = std::numeric_limits<double>::max();
    const WindowNode * best_node = nullptr;
    for (const auto & node : window.nodes) {
      if (node.is_begin) {
        continue;  // already absorbed into the boundary prior
      }
      const double distance = std::abs(node.stamp - sample.stamp);
      if (distance < best_distance) {
        best_distance = distance;
        best_node = &node;
      }
    }
    if (best_node == nullptr || best_distance > max_offset) {
      continue;
    }
    auto it = best.find(best_node->index);
    if (it == best.end() || best_distance < it->second.first) {
      best[best_node->index] = {best_distance, &sample};
    }
  }

  for (const auto & [node_index, entry] : best) {
    const double offset = entry.first;
    WheelBinding binding;
    binding.node_index = node_index;
    binding.stamp = entry.second->stamp;
    binding.vx = entry.second->vx;
    // The time offset is converted into extra noise with an acceleration bound
    // (design §5.3.3a' step 5).
    binding.variance =
      entry.second->variance + std::pow(params_.wheel_speed.graph2_time_offset_accel * offset, 2);
    bindings.push_back(binding);
  }
  return bindings;
}

}  // namespace autoware::pose_twist_fusion::fg
