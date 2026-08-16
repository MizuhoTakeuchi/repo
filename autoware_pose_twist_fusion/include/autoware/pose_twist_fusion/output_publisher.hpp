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

#ifndef AUTOWARE__POSE_TWIST_FUSION__OUTPUT_PUBLISHER_HPP_
#define AUTOWARE__POSE_TWIST_FUSION__OUTPUT_PUBLISHER_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"
#include "autoware/pose_twist_fusion/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::pose_twist_fusion
{

/// Single owner of every output topic of design §3.1, so that the drop-in
/// compatibility contract lives in one place for all three modes.
class OutputPublisher
{
public:
  OutputPublisher(rclcpp::Node & node, const Parameters & params);

  /// Publishes #1, #2a, #2b, #3, #4, #5, #6, #7, #8 with the same stamp and the
  /// same content where design §3.1 requires it.
  void publish(const PublishedEstimate & estimate, const rclcpp::Time & stamp);

  void publish_tf(const PublishedEstimate & estimate, const rclcpp::Time & stamp);

  void publish_processing_time(double milliseconds, const rclcpp::Time & stamp);

  void publish_debug(const std::string & key, double value, const rclcpp::Time & stamp);

  /// Number of times the covariance had to be repaired before publishing.
  [[nodiscard]] uint64_t covariance_repair_count() const { return covariance_repair_count_; }

private:
  geometry_msgs::msg::PoseWithCovarianceStamped make_pose_msg(
    const gtsam::Pose3 & pose, const gtsam::Matrix6 & mse, const PublishedEstimate & estimate,
    const rclcpp::Time & stamp);

  geometry_msgs::msg::TwistWithCovarianceStamped make_twist_msg(
    const PublishedEstimate & estimate, const rclcpp::Time & stamp);

  rclcpp::Node * node_;
  Parameters params_;
  bool flatten_to_2d_{false};
  uint64_t covariance_repair_count_{0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_ns_pose_cov_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_no_bias_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_biased_pose_cov_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_biased_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr pub_yaw_bias_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;
  std::map<std::string, rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::
                          SharedPtr>
    pub_debug_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace autoware::pose_twist_fusion

#endif  // AUTOWARE__POSE_TWIST_FUSION__OUTPUT_PUBLISHER_HPP_
