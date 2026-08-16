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

#include "plugins/factor_graph_fusion/imu_frame_adapter.hpp"

#include "ros_conversions.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <string>

namespace autoware::pose_twist_fusion::fg
{

ImuFrameAdapter::ImuFrameAdapter(const ImuParams & params) : params_(params)
{
  t_base_imu_ = from_pose_param(params.extrinsic_fallback);
}

bool ImuFrameAdapter::resolve(
  tf2_ros::Buffer & tf_buffer, const std::string & base_frame, std::string * message)
{
  if (!params_.use_tf_extrinsic) {
    t_base_imu_ = from_pose_param(params_.extrinsic_fallback);
    resolved_ = true;
    if (message != nullptr) {
      *message = "imu extrinsic taken from imu.extrinsic_fallback (use_tf_extrinsic=false)";
    }
    return true;
  }
  try {
    const auto transform =
      tf_buffer.lookupTransform(base_frame, params_.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    t_base_imu_ = to_gtsam(pose);
    resolved_ = true;
    if (message != nullptr) {
      *message = "imu extrinsic from TF " + base_frame + " <- " + params_.frame_id;
    }
    return true;
  } catch (const tf2::TransformException & e) {
    t_base_imu_ = from_pose_param(params_.extrinsic_fallback);
    resolved_ = false;
    if (message != nullptr) {
      *message = std::string("TF lookup for the IMU extrinsic failed: ") + e.what();
    }
    return false;
  }
}

double ImuFrameAdapter::extra_accel_variance() const
{
  if (params_.lever_arm_compensation == "centrifugal") {
    return params_.euler_term_sigma * params_.euler_term_sigma;
  }
  return 0.0;
}

ImuSample ImuFrameAdapter::convert(const sensor_msgs::msg::Imu & msg)
{
  ImuSample sample;
  sample.stamp = rclcpp::Time(msg.header.stamp).seconds();

  const gtsam::Matrix3 rotation = t_base_imu_.rotation().matrix();
  const gtsam::Vector3 lever_arm = t_base_imu_.translation();

  const gtsam::Vector3 omega = rotation * to_gtsam(msg.angular_velocity);
  gtsam::Vector3 accel = rotation * to_gtsam(msg.linear_acceleration);

  if (params_.lever_arm_compensation != "rotation_only") {
    accel -= omega.cross(omega.cross(lever_arm));
  }
  if (params_.lever_arm_compensation == "full" && has_previous_) {
    const double dt = sample.stamp - previous_stamp_;
    if (dt > 1e-6) {
      const gtsam::Vector3 raw_omega_dot = (omega - previous_omega_) / dt;
      // 20 Hz first order low pass; the raw derivative of a gyro signal is far too
      // noisy to be used directly.
      const double cutoff_hz = 20.0;
      const double alpha = dt / (dt + 1.0 / (2.0 * M_PI * cutoff_hz));
      filtered_omega_dot_ = (1.0 - alpha) * filtered_omega_dot_ + alpha * raw_omega_dot;
      accel -= filtered_omega_dot_.cross(lever_arm);
    }
  }

  has_previous_ = true;
  previous_stamp_ = sample.stamp;
  previous_omega_ = omega;

  sample.accel = accel;
  sample.gyro = omega;
  return sample;
}

}  // namespace autoware::pose_twist_fusion::fg
