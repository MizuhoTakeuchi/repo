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

#ifndef PLUGINS__EKF__EKF_FUSION_PLUGIN_HPP_
#define PLUGINS__EKF__EKF_FUSION_PLUGIN_HPP_

#include "autoware/pose_twist_fusion/fusion_plugin_base.hpp"
#include "plugins/ekf/vendored/aged_object_queue.hpp"
#include "plugins/ekf/vendored/ekf_module.hpp"
#include "plugins/ekf/vendored/hyper_parameters.hpp"
#include "plugins/ekf/vendored/warning.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>

namespace autoware::pose_twist_fusion
{

/// Autoware's 2D EKF (design §5.1). The estimation core is the vendored
/// `ekf_module` of autoware_ekf_localizer, unchanged; only the I/O is rewired to
/// the common output interface of §3.
class EkfFusionPlugin : public FusionPlugin
{
public:
  void initialize(rclcpp::Node & node, const Parameters & params) override;
  [[nodiscard]] InputRequirements required_inputs() const override;
  void on_vehicle_twist(const geometry_msgs::msg::TwistWithCovarianceStamped & msg) override;
  void on_ndt_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) override;
  void set_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) override;
  void reset() override;
  [[nodiscard]] InternalEstimate estimate_internal(const rclcpp::Time & stamp) const override;
  /// The EKF updates continuously and therefore never produces an anchor jump; a
  /// constant generation tells OutputStage that there is nothing to smooth.
  [[nodiscard]] EstimateGeneration generation() const override { return 1; }
  [[nodiscard]] diagnostic_msgs::msg::DiagnosticStatus diagnostics() const override;
  [[nodiscard]] bool is_running() const override;

private:
  void on_timer();

  rclcpp::Node * node_{nullptr};
  Parameters params_;
  ekf_vendored::HyperParameters hyper_params_;
  std::shared_ptr<ekf_vendored::Warning> warning_;
  std::unique_ptr<ekf_vendored::EKFModule> ekf_module_;

  mutable std::mutex mutex_;
  bool initialized_{false};
  double ekf_dt_{0.02};
  std::shared_ptr<const rclcpp::Time> last_predict_time_;
  ekf_vendored::EKFDiagnosticInfo pose_diag_info_;
  ekf_vendored::EKFDiagnosticInfo twist_diag_info_;

  std::unique_ptr<ekf_vendored::AgedObjectQueue<
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>>
    pose_queue_;
  std::unique_ptr<ekf_vendored::AgedObjectQueue<
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr>>
    twist_queue_;

  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace autoware::pose_twist_fusion

#endif  // PLUGINS__EKF__EKF_FUSION_PLUGIN_HPP_
