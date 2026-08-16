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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__FACTOR_GRAPH_FUSION_PLUGIN_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__FACTOR_GRAPH_FUSION_PLUGIN_HPP_

#include "autoware/pose_twist_fusion/fusion_plugin_base.hpp"
#include "plugins/factor_graph_fusion/anchor_manager.hpp"
#include "plugins/factor_graph_fusion/covariance_corrector.hpp"
#include "plugins/factor_graph_fusion/graph1_propagator.hpp"
#include "plugins/factor_graph_fusion/imu_frame_adapter.hpp"
#include "plugins/factor_graph_fusion/initializer.hpp"
#include "plugins/factor_graph_fusion/measurement_queue.hpp"
#include "plugins/factor_graph_fusion/slam_pair_gate.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::pose_twist_fusion
{

/// Two stage GTSAM factor graph fusion (design §5.3), the mode required by R4-R11.
class FactorGraphFusionPlugin : public FusionPlugin
{
public:
  ~FactorGraphFusionPlugin() override;

  void initialize(rclcpp::Node & node, const Parameters & params) override;
  [[nodiscard]] InputRequirements required_inputs() const override;
  void on_imu(const sensor_msgs::msg::Imu & msg) override;
  void on_vehicle_twist(const geometry_msgs::msg::TwistWithCovarianceStamped & msg) override;
  void on_ndt_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) override;
  void on_gnss_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) override;
  void on_slam_odometry(const nav_msgs::msg::Odometry & msg) override;
  void set_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) override;
  void reset() override;
  [[nodiscard]] InternalEstimate estimate_internal(const rclcpp::Time & stamp) const override;
  [[nodiscard]] std::optional<InternalEstimate> estimate_internal_previous(
    const rclcpp::Time & stamp) const override;
  [[nodiscard]] EstimateGeneration generation() const override;
  [[nodiscard]] diagnostic_msgs::msg::DiagnosticStatus diagnostics() const override;
  [[nodiscard]] std::vector<DebugValue> debug_values() const override;
  [[nodiscard]] bool is_running() const override;

private:
  /// Common part of the measurement callbacks (design §5.3.4 steps 1-5).
  /// Returns false when the measurement was dropped, with `reason` set.
  bool accept_absolute_pose(
    fg::MeasurementSource source, double stamp, const gtsam::Pose3 & pose,
    const std::array<double, 36> & covariance, std::string * reason);

  void handle_slam_relative(
    double stamp, const gtsam::Pose3 & pose, const std::array<double, 36> & covariance);

  void try_initialize(double now);
  void resolve_gnss_lever_arm();
  void on_tick();

  [[nodiscard]] InternalEstimate evaluate(
    const fg::Graph1Propagator::Snapshot & snapshot, double target) const;

  rclcpp::Node * node_{nullptr};
  Parameters params_;

  std::shared_ptr<fg::CovarianceCorrector> corrector_;
  std::unique_ptr<fg::ImuFrameAdapter> imu_adapter_;
  std::unique_ptr<fg::ImuBuffer> imu_buffer_;
  std::unique_ptr<fg::VehicleTwistBuffer> twist_buffer_;
  std::unique_ptr<fg::Graph1Propagator> graph1_;
  std::unique_ptr<fg::PendingMeasurementQueue> pending_;
  std::unique_ptr<fg::AnchorManager> anchor_manager_;
  std::unique_ptr<fg::SlamPairGate> slam_gate_;
  std::unique_ptr<fg::Initializer> initializer_;
  boost::shared_ptr<gtsam::PreintegrationParams> preint_params_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  gtsam::Point3 gnss_lever_arm_{gtsam::Point3(0, 0, 0)};
  bool gnss_lever_arm_resolved_{false};
  std::string extrinsic_message_;
  std::string factor_builders_lever_arm_message_{"gnss lever arm from gnss.lever_arm_fallback"};

  rclcpp::CallbackGroup::SharedPtr tick_cb_group_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  mutable std::mutex counters_mutex_;
  std::atomic<bool> running_{false};
  double last_imu_stamp_{0.0};
  double last_twist_stamp_{0.0};
  double last_ndt_stamp_{0.0};
  double last_gnss_stamp_{0.0};
  double last_slam_stamp_{0.0};
  double last_gnss_accepted_{0.0};
  double last_ndt_accepted_{0.0};
  double last_slam_accepted_{0.0};
  std::uint64_t gnss_rejected_{0};
  std::uint64_t ndt_rejected_{0};
  std::uint64_t slam_rejected_{0};
  std::uint64_t dropped_stale_{0};
  std::uint64_t dropped_future_{0};
  std::uint64_t dropped_before_window_{0};
  std::uint64_t dropped_invalid_covariance_{0};
  double last_gnss_d2_{0.0};
  double last_gnss_w_{1.0};
  double last_ndt_d2_{0.0};
  double last_ndt_w_{1.0};
  double last_slam_d2_{0.0};
  double last_slam_w_{1.0};
};

}  // namespace autoware::pose_twist_fusion

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__FACTOR_GRAPH_FUSION_PLUGIN_HPP_
