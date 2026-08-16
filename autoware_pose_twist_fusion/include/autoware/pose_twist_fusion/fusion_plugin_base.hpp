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

#ifndef AUTOWARE__POSE_TWIST_FUSION__FUSION_PLUGIN_BASE_HPP_
#define AUTOWARE__POSE_TWIST_FUSION__FUSION_PLUGIN_BASE_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"
#include "autoware/pose_twist_fusion/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <optional>
#include <string>
#include <vector>

namespace autoware::pose_twist_fusion
{

/// Which sensor streams a plugin wants the node to subscribe to (design §4.1).
/// The node creates exactly these subscriptions, so a mode never pays for inputs
/// it does not consume.
struct InputRequirements
{
  bool imu{false};
  bool vehicle_twist{false};
  bool ndt_pose{false};
  bool gnss_pose{false};
  bool slam_odometry{false};

  /// Topic names the node should use. Empty means "use the node level default".
  std::string imu_topic;
  std::string vehicle_twist_topic;
  std::string ndt_pose_topic;
  std::string gnss_pose_topic;
  std::string slam_odometry_topic;
};

/// Additional debug values a plugin wants published under `debug/covariance_correction/*`.
struct DebugValue
{
  std::string key;
  double value{0.0};
};

/// Strategy interface for the three estimation algorithms (D2 / D43).
///
/// Contract (D43): a plugin returns the **internal** estimate only. It owns no
/// smoothing state, which is what makes `estimate_internal()` const and lets the
/// three modes share one smoothing implementation in OutputStage.
class FusionPlugin
{
public:
  virtual ~FusionPlugin() = default;

  /// Called once, right after pluginlib construction.
  virtual void initialize(rclcpp::Node & node, const Parameters & params) = 0;

  [[nodiscard]] virtual InputRequirements required_inputs() const = 0;

  virtual void on_imu(const sensor_msgs::msg::Imu & /*msg*/) {}
  virtual void on_vehicle_twist(const geometry_msgs::msg::TwistWithCovarianceStamped & /*msg*/) {}
  virtual void on_ndt_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & /*msg*/) {}
  virtual void on_gnss_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & /*msg*/) {}
  virtual void on_slam_odometry(const nav_msgs::msg::Odometry & /*msg*/) {}

  virtual void set_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & /*msg*/) {}

  /// trigger_node(false), divergence recovery, re-initialization.
  virtual void reset() = 0;

  /// Internal estimate extrapolated to `stamp`. Must be non-blocking and must not
  /// touch the optimizer: the output timer calls it at `output_rate_hz` (D17).
  [[nodiscard]] virtual InternalEstimate estimate_internal(const rclcpp::Time & stamp) const = 0;

  /// Same, but evaluated from the **previous** generation's snapshot.
  ///
  /// OutputStage needs `X_internal_old(t_now)` to move a fresh anchor jump into the
  /// smoothing offset without moving the published pose:
  ///   c <- Log(X_new^-1 . X_old . Exp(c))      (design §5.3.7 "アンカー更新の検出")
  /// Plugins that keep the previous snapshot alive (a shared_ptr, as required by
  /// D37) implement this; the others return nullopt and the anchor jump is then
  /// applied immediately, which is correct for modes that cannot jump (ekf).
  [[nodiscard]] virtual std::optional<InternalEstimate> estimate_internal_previous(
    const rclcpp::Time & /*stamp*/) const
  {
    return std::nullopt;
  }

  /// Incremented on every anchor update (D37). OutputStage compares it to detect
  /// anchor updates; it must never compare poses.
  [[nodiscard]] virtual EstimateGeneration generation() const = 0;

  [[nodiscard]] virtual diagnostic_msgs::msg::DiagnosticStatus diagnostics() const = 0;

  /// Values published under `debug/covariance_correction/*` (design §3.1 #10).
  [[nodiscard]] virtual std::vector<DebugValue> debug_values() const { return {}; }

  /// The estimator is initialized and allowed to publish.
  [[nodiscard]] virtual bool is_running() const = 0;
};

}  // namespace autoware::pose_twist_fusion

#endif  // AUTOWARE__POSE_TWIST_FUSION__FUSION_PLUGIN_BASE_HPP_
