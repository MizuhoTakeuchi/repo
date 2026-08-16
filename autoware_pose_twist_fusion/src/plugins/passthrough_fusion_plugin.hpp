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

#ifndef PLUGINS__PASSTHROUGH_FUSION_PLUGIN_HPP_
#define PLUGINS__PASSTHROUGH_FUSION_PLUGIN_HPP_

#include "autoware/pose_twist_fusion/fusion_plugin_base.hpp"

#include <mutex>

namespace autoware::pose_twist_fusion
{

/// Minimal plugin used to verify the wiring of the node (Phase 0): it forwards the
/// pose_estimator input as the internal estimate without any filtering.
class PassthroughFusionPlugin : public FusionPlugin
{
public:
  void initialize(rclcpp::Node & node, const Parameters & params) override;
  [[nodiscard]] InputRequirements required_inputs() const override;
  void on_ndt_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) override;
  void on_vehicle_twist(const geometry_msgs::msg::TwistWithCovarianceStamped & msg) override;
  void set_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) override;
  void reset() override;
  [[nodiscard]] InternalEstimate estimate_internal(const rclcpp::Time & stamp) const override;
  [[nodiscard]] EstimateGeneration generation() const override { return generation_; }
  [[nodiscard]] diagnostic_msgs::msg::DiagnosticStatus diagnostics() const override;
  [[nodiscard]] bool is_running() const override;

private:
  rclcpp::Node * node_{nullptr};
  Parameters params_;
  mutable std::mutex mutex_;
  InternalEstimate estimate_;
  EstimateGeneration generation_{0};
};

}  // namespace autoware::pose_twist_fusion

#endif  // PLUGINS__PASSTHROUGH_FUSION_PLUGIN_HPP_
