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

#include "plugins/passthrough_fusion_plugin.hpp"

#include "autoware/pose_twist_fusion/diagnostics_builder.hpp"
#include "ros_conversions.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <mutex>

namespace autoware::pose_twist_fusion
{

void PassthroughFusionPlugin::initialize(rclcpp::Node & node, const Parameters & params)
{
  node_ = &node;
  params_ = params;
}

InputRequirements PassthroughFusionPlugin::required_inputs() const
{
  InputRequirements req;
  req.ndt_pose = true;
  req.ndt_pose_topic = params_.factor_graph_fusion.ndt.topic;
  req.vehicle_twist = true;
  req.vehicle_twist_topic = params_.factor_graph_fusion.wheel_speed.topic;
  return req;
}

void PassthroughFusionPlugin::on_ndt_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  estimate_.valid = true;
  estimate_.stamp = rclcpp::Time(msg.header.stamp).seconds();
  estimate_.pose = to_gtsam(msg.pose.pose);
  estimate_.biased_pose = estimate_.pose;
  const auto status = check_input_covariance(msg.pose.covariance);
  estimate_.pose_cov =
    (status == CovarianceStatus::kOk)
      ? covariance_convert::ros_pose_to_gtsam(
          to_array(msg.pose.covariance), estimate_.pose.rotation())
      : diagonal_pose_cov(
          params_.factor_graph_fusion.ndt.fallback_pos_stddev,
          params_.factor_graph_fusion.ndt.fallback_rot_stddev);
  ++generation_;
}

void PassthroughFusionPlugin::on_vehicle_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double vx = msg.twist.twist.linear.x;
  estimate_.velocity_world = estimate_.pose.rotation() * gtsam::Vector3(vx, 0.0, 0.0);
  estimate_.angular_velocity_body = to_gtsam(msg.twist.twist.angular);
  const double v_var = params_.factor_graph_fusion.wheel_speed.fallback_stddev *
                       params_.factor_graph_fusion.wheel_speed.fallback_stddev;
  estimate_.twist_cov = gtsam::Matrix6::Identity() * v_var;
}

void PassthroughFusionPlugin::set_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  on_ndt_pose(msg);
}

void PassthroughFusionPlugin::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  estimate_ = InternalEstimate{};
}

InternalEstimate PassthroughFusionPlugin::estimate_internal(const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  InternalEstimate out = estimate_;
  if (out.valid) {
    // No propagation model here; report the age so that the node can flag the clamp.
    const double max_extrapolation_s = params_.output.max_extrapolation_ms * 1e-3;
    out.stamp = std::min(stamp.seconds(), estimate_.stamp + max_extrapolation_s);
  }
  return out;
}

diagnostic_msgs::msg::DiagnosticStatus PassthroughFusionPlugin::diagnostics() const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "pose_twist_fusion: passthrough";
  status.hardware_id = "pose_twist_fusion";
  std::lock_guard<std::mutex> lock(mutex_);
  status.level = estimate_.valid ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                 : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = estimate_.valid ? "OK" : "no pose received yet";
  add_key_value(status, "generation", generation_);
  return status;
}

bool PassthroughFusionPlugin::is_running() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return estimate_.valid;
}

}  // namespace autoware::pose_twist_fusion

PLUGINLIB_EXPORT_CLASS(
  autoware::pose_twist_fusion::PassthroughFusionPlugin, autoware::pose_twist_fusion::FusionPlugin)
