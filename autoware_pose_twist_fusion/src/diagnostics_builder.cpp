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

#include "autoware/pose_twist_fusion/diagnostics_builder.hpp"

#include <algorithm>
#include <string>

namespace autoware::pose_twist_fusion
{

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

void add_key_value(DiagnosticStatus & status, const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  status.values.push_back(kv);
}

void add_key_value(DiagnosticStatus & status, const std::string & key, double value)
{
  add_key_value(status, key, std::to_string(value));
}

void add_key_value(DiagnosticStatus & status, const std::string & key, std::uint64_t value)
{
  add_key_value(status, key, std::to_string(value));
}

void add_key_value(DiagnosticStatus & status, const std::string & key, bool value)
{
  add_key_value(status, key, std::string(value ? "true" : "false"));
}

diagnostic_msgs::msg::DiagnosticArray DiagnosticsBuilder::build(
  const NodeStatus & s, const DiagnosticStatus & plugin_status, const rclcpp::Time & stamp,
  const std::string & hardware_id) const
{
  DiagnosticStatus node_status;
  node_status.name = "pose_twist_fusion: node";
  node_status.hardware_id = hardware_id;
  node_status.level = DiagnosticStatus::OK;
  node_status.message = "OK";

  add_key_value(node_status, "fusion_method", s.fusion_method);
  add_key_value(node_status, "trigger_enabled", s.trigger_enabled);
  add_key_value(node_status, "plugin_running", s.plugin_running);
  add_key_value(node_status, "extrapolation_ms", s.extrapolation_ms);
  add_key_value(node_status, "processing_time_ms", s.processing_time_ms);
  add_key_value(node_status, "smoothing_active", s.smoothing_active);
  add_key_value(node_status, "smoothing_count", s.smoothing_count);
  add_key_value(node_status, "smoothing_rate_exceeded_count", s.smoothing_rate_exceeded_count);
  add_key_value(node_status, "covariance_repair_count", s.covariance_repair_count);

  if (!s.trigger_enabled) {
    node_status.level = std::max<unsigned char>(node_status.level, DiagnosticStatus::WARN);
    node_status.message = "estimation stopped by trigger_node_srv";
  } else if (!s.plugin_running || !s.has_estimate) {
    node_status.level = DiagnosticStatus::ERROR;
    node_status.message = "not initialized";
  }
  if (s.extrapolation_exceeded) {
    node_status.level = std::max<unsigned char>(node_status.level, DiagnosticStatus::WARN);
    node_status.message = "extrapolation limit exceeded; publishing the last estimate time";
  }
  // D45: smoothing itself is INFO + counter. Only breaking the rate limit is a WARN.
  if (s.smoothing_rate_exceeded_now) {
    node_status.level = std::max<unsigned char>(node_status.level, DiagnosticStatus::WARN);
    node_status.message = "anchor jump rate exceeded";
  }
  if (s.tf_authority_conflict) {
    node_status.level = std::max<unsigned char>(node_status.level, DiagnosticStatus::WARN);
    node_status.message = "another publisher of map->base_link detected (design §7)";
  }
  for (const auto & w : s.startup_warnings) {
    add_key_value(node_status, "startup_warning", w);
    node_status.level = std::max<unsigned char>(node_status.level, DiagnosticStatus::WARN);
  }
  for (const auto & i : s.startup_infos) {
    add_key_value(node_status, "startup_info", i);
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.push_back(node_status);
  array.status.push_back(plugin_status);
  return array;
}

}  // namespace autoware::pose_twist_fusion
