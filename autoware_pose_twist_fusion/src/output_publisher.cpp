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

#include "autoware/pose_twist_fusion/output_publisher.hpp"

#include "autoware/pose_twist_fusion/covariance_convert.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>

namespace autoware::pose_twist_fusion
{

namespace
{
geometry_msgs::msg::Pose to_msg(const gtsam::Pose3 & pose)
{
  geometry_msgs::msg::Pose out;
  out.position.x = pose.x();
  out.position.y = pose.y();
  out.position.z = pose.z();
  const gtsam::Quaternion q = pose.rotation().toQuaternion();
  out.orientation.w = q.w();
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  return out;
}

/// z / roll / pitch replacement for output.flatten_to_2d (design §3.3 / D20).
gtsam::Pose3 flatten(const gtsam::Pose3 & pose, const PublishedEstimate & estimate)
{
  const gtsam::Vector3 rpy = pose.rotation().rpy();
  const double z = estimate.has_flat_reference() ? estimate.flat_z() : pose.z();
  const double roll = estimate.has_flat_reference() ? estimate.flat_roll() : rpy(0);
  const double pitch = estimate.has_flat_reference() ? estimate.flat_pitch() : rpy(1);
  return gtsam::Pose3(
    gtsam::Rot3::RzRyRx(roll, pitch, rpy(2)), gtsam::Point3(pose.x(), pose.y(), z));
}
}  // namespace

OutputPublisher::OutputPublisher(rclcpp::Node & node, const Parameters & params)
: node_(&node), params_(params)
{
  // The ekf mode is 2D by construction; flatten_to_2d is forced there (D20).
  flatten_to_2d_ = params.output.flatten_to_2d || params.fusion_method == "ekf";

  const rclcpp::QoS qos{static_cast<size_t>(std::max(params.qos.output_depth, 1))};

  pub_odom_ = node.create_publisher<nav_msgs::msg::Odometry>("kinematic_state", qos);
  pub_pose_cov_ = node.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose_with_covariance", qos);
  pub_ns_pose_cov_ = node.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ns_pose_with_covariance", qos);
  pub_pose_cov_no_bias_ = node.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose_with_covariance_no_yawbias", qos);
  pub_biased_pose_cov_ = node.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "biased_pose_with_covariance", qos);
  pub_twist_cov_ = node.create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", qos);
  pub_pose_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("pose", qos);
  pub_biased_pose_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("biased_pose", qos);
  pub_twist_ = node.create_publisher<geometry_msgs::msg::TwistStamped>("twist", qos);
  pub_yaw_bias_ = node.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "estimated_yaw_bias", qos);
  pub_processing_time_ = node.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", qos);

  if (params.enable_tf) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  }
}

geometry_msgs::msg::PoseWithCovarianceStamped OutputPublisher::make_pose_msg(
  const gtsam::Pose3 & pose, const gtsam::Matrix6 & mse, const PublishedEstimate & estimate,
  const rclcpp::Time & stamp)
{
  const gtsam::Pose3 out_pose = flatten_to_2d_ ? flatten(pose, estimate) : pose;

  bool repaired = false;
  const gtsam::Matrix6 healthy =
    covariance_convert::sanitize(mse, params_.output.undetermined_variance, &repaired);
  if (repaired) {
    ++covariance_repair_count_;
  }
  const auto ros_cov = covariance_convert::gtsam_pose_to_ros(healthy, out_pose.rotation());

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = params_.pose_frame_id;
  msg.pose.pose = to_msg(out_pose);
  for (size_t i = 0; i < 36; ++i) {
    msg.pose.covariance[i] = ros_cov[i];
  }
  return msg;
}

geometry_msgs::msg::TwistWithCovarianceStamped OutputPublisher::make_twist_msg(
  const PublishedEstimate & estimate, const rclcpp::Time & stamp)
{
  bool repaired = false;
  const gtsam::Matrix6 healthy = covariance_convert::sanitize(
    estimate.twist_mse(), params_.output.undetermined_variance, &repaired);
  if (repaired) {
    ++covariance_repair_count_;
  }
  const auto ros_cov = covariance_convert::gtsam_twist_to_ros(healthy);

  geometry_msgs::msg::TwistWithCovarianceStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = params_.base_frame_id;
  msg.twist.twist.linear.x = estimate.linear_velocity_body().x();
  msg.twist.twist.linear.y = flatten_to_2d_ ? 0.0 : estimate.linear_velocity_body().y();
  msg.twist.twist.linear.z = flatten_to_2d_ ? 0.0 : estimate.linear_velocity_body().z();
  msg.twist.twist.angular.x = flatten_to_2d_ ? 0.0 : estimate.angular_velocity_body().x();
  msg.twist.twist.angular.y = flatten_to_2d_ ? 0.0 : estimate.angular_velocity_body().y();
  msg.twist.twist.angular.z = estimate.angular_velocity_body().z();
  for (size_t i = 0; i < 36; ++i) {
    msg.twist.covariance[i] = ros_cov[i];
  }
  return msg;
}

void OutputPublisher::publish(const PublishedEstimate & estimate, const rclcpp::Time & stamp)
{
  if (!estimate.valid()) {
    return;
  }

  // #2a / #2b / #3 must be byte-identical (design §3.1). They are literally the
  // same message object, so no drift between them is possible.
  const auto pose_msg = make_pose_msg(estimate.pose(), estimate.pose_mse(), estimate, stamp);
  pub_pose_cov_->publish(pose_msg);
  pub_ns_pose_cov_->publish(pose_msg);
  pub_pose_cov_no_bias_->publish(pose_msg);

  const auto biased_msg =
    make_pose_msg(estimate.biased_pose(), estimate.pose_mse(), estimate, stamp);
  pub_biased_pose_cov_->publish(biased_msg);

  const auto twist_msg = make_twist_msg(estimate, stamp);
  pub_twist_cov_->publish(twist_msg);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = params_.pose_frame_id;
  odom.child_frame_id = params_.base_frame_id;
  odom.pose = pose_msg.pose;
  odom.twist = twist_msg.twist;
  pub_odom_->publish(odom);

  geometry_msgs::msg::PoseStamped pose_only;
  pose_only.header = pose_msg.header;
  pose_only.pose = pose_msg.pose.pose;
  pub_pose_->publish(pose_only);

  geometry_msgs::msg::PoseStamped biased_pose_only;
  biased_pose_only.header = biased_msg.header;
  biased_pose_only.pose = biased_msg.pose.pose;
  pub_biased_pose_->publish(biased_pose_only);

  geometry_msgs::msg::TwistStamped twist_only;
  twist_only.header = twist_msg.header;
  twist_only.twist = twist_msg.twist.twist;
  pub_twist_->publish(twist_only);

  autoware_internal_debug_msgs::msg::Float64Stamped yaw_bias;
  yaw_bias.stamp = stamp;
  yaw_bias.data = estimate.yaw_bias();
  pub_yaw_bias_->publish(yaw_bias);
}

void OutputPublisher::publish_tf(const PublishedEstimate & estimate, const rclcpp::Time & stamp)
{
  if (!tf_broadcaster_ || !estimate.valid()) {
    return;
  }
  const gtsam::Pose3 pose =
    flatten_to_2d_ ? flatten(estimate.pose(), estimate) : estimate.pose();

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = params_.pose_frame_id;
  tf.child_frame_id = params_.base_frame_id;
  tf.transform.translation.x = pose.x();
  tf.transform.translation.y = pose.y();
  tf.transform.translation.z = pose.z();
  const gtsam::Quaternion q = pose.rotation().toQuaternion();
  tf.transform.rotation.w = q.w();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf_broadcaster_->sendTransform(tf);
}

void OutputPublisher::publish_processing_time(double milliseconds, const rclcpp::Time & stamp)
{
  autoware_internal_debug_msgs::msg::Float64Stamped msg;
  msg.stamp = stamp;
  msg.data = milliseconds;
  pub_processing_time_->publish(msg);
}

void OutputPublisher::publish_debug(
  const std::string & key, double value, const rclcpp::Time & stamp)
{
  auto it = pub_debug_.find(key);
  if (it == pub_debug_.end()) {
    it = pub_debug_
           .emplace(
             key, node_->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
                    "debug/covariance_correction/" + key, rclcpp::QoS{1}))
           .first;
  }
  autoware_internal_debug_msgs::msg::Float64Stamped msg;
  msg.stamp = stamp;
  msg.data = value;
  it->second->publish(msg);
}

}  // namespace autoware::pose_twist_fusion
