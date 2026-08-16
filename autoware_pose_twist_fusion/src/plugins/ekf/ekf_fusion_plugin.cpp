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

#include "plugins/ekf/ekf_fusion_plugin.hpp"

#include "autoware/pose_twist_fusion/covariance_convert.hpp"
#include "autoware/pose_twist_fusion/diagnostics_builder.hpp"
#include "ros_conversions.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <tf2/utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

namespace autoware::pose_twist_fusion
{

void EkfFusionPlugin::initialize(rclcpp::Node & node, const Parameters & params)
{
  node_ = &node;
  params_ = params;

  // design §6.1: the EKF hyper parameters live under the `ekf.*` namespace of this
  // package; the algorithm itself is untouched.
  hyper_params_.ekf_rate = params.ekf.predict_frequency;
  hyper_params_.ekf_dt = 1.0 / std::max(params.ekf.predict_frequency, 0.1);
  hyper_params_.tf_rate_ = params.tf_rate_hz;
  hyper_params_.enable_yaw_bias_estimation = params.ekf.enable_yaw_bias_estimation;
  hyper_params_.extend_state_step = static_cast<size_t>(std::max(params.ekf.extend_state_step, 1));
  hyper_params_.pose_frame_id = params.pose_frame_id;
  hyper_params_.pose_additional_delay = params.ekf.pose_additional_delay;
  hyper_params_.pose_gate_dist = params.ekf.pose_gate_dist;
  hyper_params_.pose_smoothing_steps =
    static_cast<size_t>(std::max(params.ekf.pose_smoothing_steps, 1.0));
  hyper_params_.twist_additional_delay = params.ekf.twist_additional_delay;
  hyper_params_.twist_gate_dist = params.ekf.twist_gate_dist;
  hyper_params_.twist_smoothing_steps =
    static_cast<size_t>(std::max(params.ekf.twist_smoothing_steps, 1.0));
  hyper_params_.proc_stddev_vx_c = params.ekf.proc_stddev_vx_c;
  hyper_params_.proc_stddev_wz_c = params.ekf.proc_stddev_wz_c;
  hyper_params_.proc_stddev_yaw_c = params.ekf.proc_stddev_yaw_c;
  hyper_params_.z_filter_proc_dev = params.ekf.z_filter_proc_dev;
  hyper_params_.roll_filter_proc_dev = params.ekf.roll_filter_proc_dev;
  hyper_params_.pitch_filter_proc_dev = params.ekf.pitch_filter_proc_dev;
  hyper_params_.threshold_observable_velocity_mps = params.ekf.threshold_observable_velocity_mps;

  ekf_dt_ = hyper_params_.ekf_dt;
  warning_ = std::make_shared<ekf_vendored::Warning>(&node);
  ekf_module_ = std::make_unique<ekf_vendored::EKFModule>(warning_, hyper_params_);

  pose_queue_ = std::make_unique<
    ekf_vendored::AgedObjectQueue<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>>(
    hyper_params_.pose_smoothing_steps, hyper_params_.max_pose_queue_size);
  twist_queue_ = std::make_unique<
    ekf_vendored::AgedObjectQueue<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr>>(
    hyper_params_.twist_smoothing_steps, hyper_params_.max_twist_queue_size);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  timer_cb_group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = node.create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(hyper_params_.ekf_dt)),
    std::bind(&EkfFusionPlugin::on_timer, this), timer_cb_group_);
}

InputRequirements EkfFusionPlugin::required_inputs() const
{
  InputRequirements req;
  req.ndt_pose = true;
  req.ndt_pose_topic = params_.ekf.input_pose_topic;
  req.vehicle_twist = true;
  // D19: the ekf mode keeps the original gyro_odometer output for compatibility.
  req.vehicle_twist_topic = params_.ekf.input_twist_topic;
  return req;
}

void EkfFusionPlugin::on_ndt_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    return;
  }
  pose_queue_->push(std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(msg));
  if (pose_queue_->exceeded()) {
    warning_->warn_throttle("[EKF] pose queue exceeded max_queue_size", 2000);
    pose_queue_->pop();
  }
}

void EkfFusionPlugin::on_vehicle_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    return;
  }
  auto copy = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>(msg);
  // Original behaviour: below the observable velocity the vx measurement is
  // effectively disabled by inflating its variance (not by dropping it).
  if (std::abs(copy->twist.twist.linear.x) < hyper_params_.threshold_observable_velocity_mps) {
    copy->twist.covariance[0] = 10000.0;
  }
  twist_queue_->push(copy);
  if (twist_queue_->exceeded()) {
    warning_->warn_throttle("[EKF] twist queue exceeded max_queue_size", 2000);
    twist_queue_->pop();
  }
}

void EkfFusionPlugin::set_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation.w = 1.0;
  try {
    transform = tf_buffer_->lookupTransform(
      params_.pose_frame_id, msg.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(
      node_->get_logger(), "[EKF] TF %s <- %s failed (%s); assuming identity",
      params_.pose_frame_id.c_str(), msg.header.frame_id.c_str(), e.what());
  }

  std::lock_guard<std::mutex> lock(mutex_);
  ekf_module_->initialize(msg, transform);
  initialized_ = true;
  last_predict_time_.reset();
}

void EkfFusionPlugin::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  initialized_ = false;
  pose_queue_->clear();
  twist_queue_->clear();
  last_predict_time_.reset();
  ekf_module_ = std::make_unique<ekf_vendored::EKFModule>(warning_, hyper_params_);
}

void EkfFusionPlugin::on_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    return;
  }
  const rclcpp::Time current_time = node_->now();

  // Measure the true timer period, exactly as the original node does.
  if (last_predict_time_) {
    if (current_time < *last_predict_time_) {
      warning_->warn("Detected jump back in time");
    } else {
      ekf_dt_ = (current_time - *last_predict_time_).seconds();
      ekf_dt_ = std::min(ekf_dt_, 10.0);
      ekf_module_->accumulate_delay_time(ekf_dt_);
    }
  }
  last_predict_time_ = std::make_shared<const rclcpp::Time>(current_time);

  ekf_module_->predict_with_delay(ekf_dt_);

  bool pose_updated = false;
  if (!pose_queue_->empty()) {
    pose_diag_info_.is_passed_delay_gate = true;
    pose_diag_info_.is_passed_mahalanobis_gate = true;
    const size_t n = pose_queue_->size();
    for (size_t i = 0; i < n; ++i) {
      const auto pose = pose_queue_->pop_increment_age();
      pose_updated =
        ekf_module_->measurement_update_pose(*pose, current_time, pose_diag_info_) || pose_updated;
    }
  }
  pose_diag_info_.no_update_count = pose_updated ? 0 : pose_diag_info_.no_update_count + 1;
  pose_diag_info_.queue_size = pose_queue_->size();

  bool twist_updated = false;
  if (!twist_queue_->empty()) {
    twist_diag_info_.is_passed_delay_gate = true;
    twist_diag_info_.is_passed_mahalanobis_gate = true;
    const size_t n = twist_queue_->size();
    for (size_t i = 0; i < n; ++i) {
      const auto twist = twist_queue_->pop_increment_age();
      twist_updated =
        ekf_module_->measurement_update_twist(*twist, current_time, twist_diag_info_) ||
        twist_updated;
    }
  }
  twist_diag_info_.no_update_count = twist_updated ? 0 : twist_diag_info_.no_update_count + 1;
  twist_diag_info_.queue_size = twist_queue_->size();
}

InternalEstimate EkfFusionPlugin::estimate_internal(const rclcpp::Time & stamp) const
{
  InternalEstimate out;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    return out;
  }

  const auto pose = ekf_module_->get_current_pose(stamp, false);
  const auto biased_pose = ekf_module_->get_current_pose(stamp, true);
  const auto twist = ekf_module_->get_current_twist(stamp);

  out.valid = true;
  out.stamp = stamp.seconds();
  out.pose = to_gtsam(pose.pose);
  out.biased_pose = to_gtsam(biased_pose.pose);
  out.yaw_bias = ekf_module_->get_yaw_bias();

  // §3.3: the EKF state is [x, y, yaw, yaw_bias, vx, wz]; the other twist
  // components stay at zero (2D semantics).
  out.velocity_world = out.pose.rotation() * gtsam::Vector3(twist.twist.linear.x, 0.0, 0.0);
  out.angular_velocity_body = gtsam::Vector3(0.0, 0.0, twist.twist.angular.z);

  out.pose_cov = covariance_convert::ros_pose_to_gtsam(
    to_array(ekf_module_->get_current_pose_covariance()), out.pose.rotation());
  out.twist_cov =
    covariance_convert::ros_twist_to_gtsam(to_array(ekf_module_->get_current_twist_covariance()));

  // z / roll / pitch already come from the Simple1DFilter inside the module, so no
  // separate flat reference is needed.
  out.has_flat_reference = false;
  return out;
}

diagnostic_msgs::msg::DiagnosticStatus EkfFusionPlugin::diagnostics() const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "pose_twist_fusion: ekf";
  status.hardware_id = "pose_twist_fusion";
  std::lock_guard<std::mutex> lock(mutex_);
  status.level = initialized_ ? diagnostic_msgs::msg::DiagnosticStatus::OK
                              : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = initialized_ ? "OK" : "initial pose is not set";
  add_key_value(status, "pose_no_update_count", static_cast<std::uint64_t>(pose_diag_info_.no_update_count));
  add_key_value(
    status, "twist_no_update_count", static_cast<std::uint64_t>(twist_diag_info_.no_update_count));
  add_key_value(status, "pose_mahalanobis_distance", pose_diag_info_.mahalanobis_distance);
  add_key_value(status, "twist_mahalanobis_distance", twist_diag_info_.mahalanobis_distance);
  add_key_value(status, "pose_passed_delay_gate", pose_diag_info_.is_passed_delay_gate);
  add_key_value(status, "twist_passed_delay_gate", twist_diag_info_.is_passed_delay_gate);
  if (initialized_ && (!pose_diag_info_.is_passed_mahalanobis_gate ||
                       !twist_diag_info_.is_passed_mahalanobis_gate)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "measurement rejected by the mahalanobis gate";
  }
  return status;
}

bool EkfFusionPlugin::is_running() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

}  // namespace autoware::pose_twist_fusion

PLUGINLIB_EXPORT_CLASS(
  autoware::pose_twist_fusion::EkfFusionPlugin, autoware::pose_twist_fusion::FusionPlugin)
