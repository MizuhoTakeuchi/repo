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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__FACTOR_BUILDERS_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__FACTOR_BUILDERS_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"
#include "plugins/factor_graph_fusion/fg_types.hpp"
#include "plugins/factor_graph_fusion/graph2_window.hpp"
#include "plugins/factor_graph_fusion/imu_buffer.hpp"
#include "plugins/factor_graph_fusion/state_history.hpp"

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <vector>

namespace autoware::pose_twist_fusion::fg
{

gtsam::Key pose_key(std::uint64_t index);
gtsam::Key velocity_key(std::uint64_t index);
gtsam::Key bias_key(std::uint64_t index);

/// Builds the whole window graph from scratch every tick (D24).
class FactorBuilders
{
public:
  struct Input
  {
    const Graph2Window::BuildResult * window{nullptr};
    std::vector<AcceptedMeasurement *> measurements;
    std::vector<WheelBinding> wheel_bindings;
    const ImuBuffer * imu_buffer{nullptr};
    const StateHistory * state_history{nullptr};
    /// Boundary prior of the previous marginalization (empty on the first tick).
    gtsam::NonlinearFactorGraph boundary_prior;
    /// Linearization point / initial values from the previous solution.
    gtsam::Values previous_solution;
    gtsam::imuBias::ConstantBias bias_estimate;
    gtsam::NavState begin_state;
    bool has_begin_state{false};
  };

  struct Output
  {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    size_t imu_factors{0};
    size_t measurement_factors{0};
    size_t wheel_factors{0};
    size_t slam_pairs_split{0};
    bool valid{false};
  };

  explicit FactorBuilders(const FactorGraphParams & params, gtsam::Point3 gnss_lever_arm);

  Output build(const Input & input) const;

  void set_gnss_lever_arm(const gtsam::Point3 & lever_arm) { gnss_lever_arm_ = lever_arm; }

private:
  FactorGraphParams params_;
  gtsam::Point3 gnss_lever_arm_;
  boost::shared_ptr<gtsam::PreintegrationParams> preint_params_;
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__FACTORS__FACTOR_BUILDERS_HPP_
