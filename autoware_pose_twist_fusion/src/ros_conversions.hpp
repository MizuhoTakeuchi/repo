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

#ifndef ROS_CONVERSIONS_HPP_
#define ROS_CONVERSIONS_HPP_

#include "autoware/pose_twist_fusion/covariance_convert.hpp"
#include "autoware/pose_twist_fusion/parameters.hpp"

#include <gtsam/geometry/Pose3.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <array>

namespace autoware::pose_twist_fusion
{

inline gtsam::Pose3 to_gtsam(const geometry_msgs::msg::Pose & pose)
{
  const gtsam::Rot3 rotation(gtsam::Quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));
  return gtsam::Pose3(rotation, gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
}

inline gtsam::Vector3 to_gtsam(const geometry_msgs::msg::Vector3 & v)
{
  return gtsam::Vector3(v.x, v.y, v.z);
}

inline covariance_convert::RosCovariance to_array(const std::array<double, 36> & cov)
{
  covariance_convert::RosCovariance out{};
  for (size_t i = 0; i < 36; ++i) {
    out[i] = cov[i];
  }
  return out;
}

inline gtsam::Pose3 from_pose_param(const PoseParam & p)
{
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
}

inline gtsam::Matrix6 diagonal_pose_cov(double pos_stddev, double rot_stddev)
{
  gtsam::Vector6 diag;
  diag << rot_stddev * rot_stddev, rot_stddev * rot_stddev, rot_stddev * rot_stddev,
    pos_stddev * pos_stddev, pos_stddev * pos_stddev, pos_stddev * pos_stddev;
  return diag.asDiagonal();
}

/// Validity layer 0 of design §5.3.9 applied to an incoming ROS covariance.
enum class CovarianceStatus { kOk, kMissing, kInvalid };

inline CovarianceStatus check_input_covariance(const std::array<double, 36> & cov)
{
  const auto arr = to_array(cov);
  if (!covariance_convert::is_finite(arr)) {
    return CovarianceStatus::kInvalid;  // NaN / Inf -> drop
  }
  if (covariance_convert::is_all_zero(arr)) {
    return CovarianceStatus::kMissing;  // "not provided" -> fall back to parameters
  }
  for (size_t i = 0; i < 6; ++i) {
    if (cov[i * 6 + i] < 0.0) {
      return CovarianceStatus::kInvalid;  // negative variance -> drop
    }
  }
  return CovarianceStatus::kOk;
}

}  // namespace autoware::pose_twist_fusion

#endif  // ROS_CONVERSIONS_HPP_
