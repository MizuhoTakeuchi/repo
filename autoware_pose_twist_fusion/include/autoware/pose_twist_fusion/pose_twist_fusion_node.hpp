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

#ifndef AUTOWARE__POSE_TWIST_FUSION__POSE_TWIST_FUSION_NODE_HPP_
#define AUTOWARE__POSE_TWIST_FUSION__POSE_TWIST_FUSION_NODE_HPP_

#include "autoware/pose_twist_fusion/diagnostics_builder.hpp"
#include "autoware/pose_twist_fusion/fusion_plugin_base.hpp"
#include "autoware/pose_twist_fusion/output_publisher.hpp"
#include "autoware/pose_twist_fusion/output_stage.hpp"
#include "autoware/pose_twist_fusion/parameters.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::pose_twist_fusion
{

/// The one and only node of this package (design §4.1). It owns the ROS surface;
/// the estimation algorithm is a pluginlib strategy selected by `fusion_method`.
class PoseTwistFusionNode : public rclcpp::Node
{
public:
  explicit PoseTwistFusionNode(const rclcpp::NodeOptions & options);

private:
  void on_output_timer();
  void on_tf_timer();
  void on_diagnostics_timer();
  void on_trigger(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void on_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /// Timestamp policy of design §3.3: extrapolate to now(), but never further than
  /// `output.max_extrapolation_ms` past the last estimate.
  rclcpp::Time output_stamp(bool * extrapolation_exceeded, double * extrapolation_ms) const;

  Parameters params_;
  std::vector<std::string> startup_warnings_;
  std::vector<std::string> startup_infos_;

  std::unique_ptr<pluginlib::ClassLoader<FusionPlugin>> plugin_loader_;
  std::shared_ptr<FusionPlugin> plugin_;

  std::unique_ptr<OutputStage> output_stage_;
  std::unique_ptr<OutputPublisher> output_publisher_;
  DiagnosticsBuilder diagnostics_builder_;

  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
  rclcpp::CallbackGroup::SharedPtr measurement_cb_group_;
  rclcpp::CallbackGroup::SharedPtr output_cb_group_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ndt_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gnss_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_slam_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr trigger_service_;

  rclcpp::TimerBase::SharedPtr output_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  bool trigger_enabled_{true};
  bool last_smoothing_rate_exceeded_{false};
  double last_processing_time_ms_{0.0};
  double last_extrapolation_ms_{0.0};
  bool last_extrapolation_exceeded_{false};
  bool last_estimate_valid_{false};
  /// Last OutputStage result, reused by the TF timer (never fed back into the stage).
  PublishedEstimate last_published_;
  rclcpp::Time last_published_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace autoware::pose_twist_fusion

#endif  // AUTOWARE__POSE_TWIST_FUSION__POSE_TWIST_FUSION_NODE_HPP_
