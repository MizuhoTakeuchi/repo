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

#ifndef AUTOWARE__POSE_TWIST_FUSION__DIAGNOSTICS_BUILDER_HPP_
#define AUTOWARE__POSE_TWIST_FUSION__DIAGNOSTICS_BUILDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::pose_twist_fusion
{

/// Builds the diagnostics of all three modes from one place, so that
/// pose_instability_detector sees identical semantics regardless of fusion_method
/// (design §10 "診断の不統一").
class DiagnosticsBuilder
{
public:
  struct NodeStatus
  {
    std::string fusion_method;
    bool trigger_enabled{true};
    bool plugin_running{false};
    bool has_estimate{false};
    double extrapolation_ms{0.0};
    bool extrapolation_exceeded{false};
    double processing_time_ms{0.0};
    bool smoothing_active{false};
    /// Smoothing activation is INFO + counter; only exceeding the rate limit is WARN (D45).
    std::uint64_t smoothing_count{0};
    std::uint64_t smoothing_rate_exceeded_count{0};
    bool smoothing_rate_exceeded_now{false};
    std::uint64_t covariance_repair_count{0};
    bool tf_authority_conflict{false};
    std::vector<std::string> startup_warnings;
    std::vector<std::string> startup_infos;
  };

  DiagnosticsBuilder() = default;

  diagnostic_msgs::msg::DiagnosticArray build(
    const NodeStatus & status, const diagnostic_msgs::msg::DiagnosticStatus & plugin_status,
    const rclcpp::Time & stamp, const std::string & hardware_id = "pose_twist_fusion") const;
};

/// Convenience helpers shared by the plugins so that key/value formatting is uniform.
void add_key_value(
  diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key,
  const std::string & value);
void add_key_value(
  diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key, double value);
void add_key_value(
  diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key, std::uint64_t value);
void add_key_value(
  diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key, bool value);

}  // namespace autoware::pose_twist_fusion

#endif  // AUTOWARE__POSE_TWIST_FUSION__DIAGNOSTICS_BUILDER_HPP_
