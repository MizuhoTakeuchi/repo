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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__IMU_FRAME_ADAPTER_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__IMU_FRAME_ADAPTER_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"
#include "plugins/factor_graph_fusion/fg_types.hpp"

#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/buffer.h>

#include <string>

namespace autoware::pose_twist_fusion::fg
{

/// imu_link -> base_link conversion (design §5.3.1 / D13).
///
/// The navigation body frame of this package is base_link, so the measurement is
/// rotated and (by default) compensated for the centrifugal term. The Euler term is
/// dropped in the default mode because differentiating omega numerically is very
/// noisy; the residual is accounted for by `imu.euler_term_sigma`.
class ImuFrameAdapter
{
public:
  explicit ImuFrameAdapter(const ImuParams & params);

  /// Looks up TF base_link <- imu_link, falling back to `imu.extrinsic_fallback`.
  /// Returns false when neither is available (start-up failure, design §5.3.1).
  bool resolve(
    tf2_ros::Buffer & tf_buffer, const std::string & base_frame, std::string * message);

  ImuSample convert(const sensor_msgs::msg::Imu & msg);

  [[nodiscard]] const gtsam::Pose3 & extrinsic() const { return t_base_imu_; }
  [[nodiscard]] bool resolved() const { return resolved_; }
  /// Extra accelerometer variance to account for the neglected Euler term.
  [[nodiscard]] double extra_accel_variance() const;

private:
  ImuParams params_;
  gtsam::Pose3 t_base_imu_;
  bool resolved_{false};
  bool has_previous_{false};
  double previous_stamp_{0.0};
  gtsam::Vector3 previous_omega_{gtsam::Vector3::Zero()};
  gtsam::Vector3 filtered_omega_dot_{gtsam::Vector3::Zero()};
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__IMU_FRAME_ADAPTER_HPP_
