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

#include "autoware/pose_twist_fusion/pose_twist_fusion_node.hpp"

#include "autoware/pose_twist_fusion/param_validator.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::pose_twist_fusion
{

namespace
{
std::string plugin_class_of(const std::string & fusion_method)
{
  static const std::map<std::string, std::string> kMap = {
    {"ekf", "autoware::pose_twist_fusion::EkfFusionPlugin"},
    {"lio_sam_imu_preintegration", "autoware::pose_twist_fusion::LioSamImuPreintegrationPlugin"},
    {"factor_graph_fusion", "autoware::pose_twist_fusion::FactorGraphFusionPlugin"},
    {"passthrough", "autoware::pose_twist_fusion::PassthroughFusionPlugin"},
  };
  const auto it = kMap.find(fusion_method);
  if (it == kMap.end()) {
    throw std::runtime_error("unknown fusion_method: " + fusion_method);
  }
  return it->second;
}

std::string pick(const std::string & from_plugin, const std::string & fallback)
{
  return from_plugin.empty() ? fallback : from_plugin;
}
}  // namespace

PoseTwistFusionNode::PoseTwistFusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pose_twist_fusion", options)
{
  // Qualified: rclcpp::Node also has a declare_parameters() member.
  params_ = autoware::pose_twist_fusion::declare_parameters(*this);

  const ValidationResult validation = validate_parameters(params_);
  for (const auto & info : validation.infos) {
    RCLCPP_INFO_STREAM(get_logger(), "parameter: " << info);
    startup_infos_.push_back(info);
  }
  for (const auto & warning : validation.warnings) {
    RCLCPP_WARN_STREAM(get_logger(), "parameter: " << warning);
    startup_warnings_.push_back(warning);
  }
  if (!validation.ok()) {
    // Fail loudly instead of running in a silently broken configuration (§6.2).
    RCLCPP_ERROR_STREAM(get_logger(), validation.error_message());
    throw std::runtime_error(validation.error_message());
  }

  sensor_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  measurement_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  output_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<FusionPlugin>>(
    "autoware_pose_twist_fusion", "autoware::pose_twist_fusion::FusionPlugin");
  plugin_ = plugin_loader_->createSharedInstance(plugin_class_of(params_.fusion_method));
  plugin_->initialize(*this, params_);

  OutputStage::Config output_config;
  output_config.output_rate_hz = params_.output_rate_hz;
  output_config.reset = params_.factor_graph_fusion.reset;
  output_config.twist_smoothing_consistency = params_.output.twist_smoothing_consistency;
  output_config.exact_smoothing_adjoint = params_.output.exact_smoothing_adjoint;
  output_config.undetermined_variance = params_.output.undetermined_variance;
  output_stage_ = std::make_unique<OutputStage>(output_config);

  output_publisher_ = std::make_unique<OutputPublisher>(*this, params_);
  pub_diagnostics_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS{1});

  // --- subscriptions requested by the plugin --------------------------------
  const InputRequirements inputs = plugin_->required_inputs();
  auto sensor_options = rclcpp::SubscriptionOptions();
  sensor_options.callback_group = sensor_cb_group_;
  auto measurement_options = rclcpp::SubscriptionOptions();
  measurement_options.callback_group = measurement_cb_group_;

  if (inputs.imu) {
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      pick(inputs.imu_topic, params_.imu_topic),
      rclcpp::QoS{static_cast<size_t>(params_.qos.imu_depth)},
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (trigger_enabled_) {
          plugin_->on_imu(*msg);
        }
      },
      sensor_options);
  }
  if (inputs.vehicle_twist) {
    sub_twist_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      pick(inputs.vehicle_twist_topic, params_.factor_graph_fusion.wheel_speed.topic),
      rclcpp::QoS{static_cast<size_t>(params_.qos.twist_depth)},
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        if (trigger_enabled_) {
          plugin_->on_vehicle_twist(*msg);
        }
      },
      sensor_options);
  }
  if (inputs.ndt_pose) {
    sub_ndt_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pick(inputs.ndt_pose_topic, params_.factor_graph_fusion.ndt.topic),
      rclcpp::QoS{static_cast<size_t>(params_.qos.pose_depth)},
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (trigger_enabled_) {
          plugin_->on_ndt_pose(*msg);
        }
      },
      measurement_options);
  }
  if (inputs.gnss_pose) {
    sub_gnss_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pick(inputs.gnss_pose_topic, params_.factor_graph_fusion.gnss.topic),
      rclcpp::QoS{static_cast<size_t>(params_.qos.pose_depth)},
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (trigger_enabled_) {
          plugin_->on_gnss_pose(*msg);
        }
      },
      measurement_options);
  }
  if (inputs.slam_odometry) {
    sub_slam_ = create_subscription<nav_msgs::msg::Odometry>(
      pick(inputs.slam_odometry_topic, params_.factor_graph_fusion.slam.topic),
      rclcpp::QoS{static_cast<size_t>(params_.qos.pose_depth)},
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (trigger_enabled_) {
          plugin_->on_slam_odometry(*msg);
        }
      },
      measurement_options);
  }

  sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    params_.initialpose_topic, rclcpp::QoS{1},
    std::bind(&PoseTwistFusionNode::on_initial_pose, this, std::placeholders::_1),
    measurement_options);

  trigger_service_ = create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &PoseTwistFusionNode::on_trigger, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  // --- timers (output callback group owns the smoothing state, D37) ---------
  const auto output_period = std::chrono::duration<double>(1.0 / params_.output_rate_hz);
  output_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(output_period),
    std::bind(&PoseTwistFusionNode::on_output_timer, this), output_cb_group_);

  if (params_.enable_tf) {
    const auto tf_period = std::chrono::duration<double>(1.0 / params_.tf_rate_hz);
    tf_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(tf_period),
      std::bind(&PoseTwistFusionNode::on_tf_timer, this), output_cb_group_);
  }

  diagnostics_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&PoseTwistFusionNode::on_diagnostics_timer, this),
    output_cb_group_);

  RCLCPP_INFO_STREAM(
    get_logger(), "pose_twist_fusion started with fusion_method=" << params_.fusion_method);
}

void PoseTwistFusionNode::on_output_timer()
{
  if (!trigger_enabled_ || !plugin_->is_running()) {
    last_estimate_valid_ = false;
    return;
  }

  const auto start = std::chrono::steady_clock::now();
  const rclcpp::Time t_now = now();

  const InternalEstimate current = plugin_->estimate_internal(t_now);
  if (!current.valid) {
    last_estimate_valid_ = false;
    return;
  }

  // §3.3: the plugin clamps the extrapolation at max_extrapolation_ms and reports
  // the instant it actually evaluated, so the node can flag the clamp.
  const double extrapolation_ms = (t_now.seconds() - current.stamp) * 1000.0;
  last_extrapolation_ms_ = extrapolation_ms;
  last_extrapolation_exceeded_ =
    extrapolation_ms > static_cast<double>(params_.output.max_extrapolation_ms) + 1.0;
  const rclcpp::Time stamp =
    last_extrapolation_exceeded_ ? rclcpp::Time(static_cast<int64_t>(current.stamp * 1e9), t_now.get_clock_type())
                                 : t_now;

  // The provider is only called when OutputStage detects an anchor update, so the
  // (potentially expensive) propagation of the previous snapshot happens once per
  // anchor rather than once per output step.
  const auto previous_provider =
    [this, &stamp](double) -> std::optional<InternalEstimate> {
    return plugin_->estimate_internal_previous(stamp);
  };

  OutputStage::Event event;
  const PublishedEstimate published = output_stage_->step(
    stamp.seconds(), current, previous_provider, plugin_->generation(), &event);
  last_smoothing_rate_exceeded_ = event.rate_limit_exceeded;
  last_estimate_valid_ = published.valid();
  last_published_ = published;
  last_published_stamp_ = stamp;

  if (event.smoothing_triggered) {
    RCLCPP_INFO(
      get_logger(), "anchor jump smoothing started: |c_lin|=%.4f m |c_rot|=%.5f rad", event.jump_lin,
      event.jump_rot);
  }
  if (event.rate_limit_exceeded) {
    RCLCPP_WARN(
      get_logger(), "anchor jump rate exceeded: |c_lin|=%.4f m |c_rot|=%.5f rad", event.jump_lin,
      event.jump_rot);
  }

  output_publisher_->publish(published, stamp);

  for (const auto & value : plugin_->debug_values()) {
    output_publisher_->publish_debug(value.key, value.value, stamp);
  }
  output_publisher_->publish_debug("smoothing_c_norm", published.smoothing_offset_lin(), stamp);
  output_publisher_->publish_debug("smoothing_c_rot", published.smoothing_offset_rot(), stamp);

  const auto end = std::chrono::steady_clock::now();
  last_processing_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();
  output_publisher_->publish_processing_time(last_processing_time_ms_, stamp);
}

void PoseTwistFusionNode::on_tf_timer()
{
  // TF must show exactly the pose that was published, so it reuses the last
  // OutputStage result instead of stepping the smoothing a second time (which
  // would advance `c` at output_rate + tf_rate).
  if (!trigger_enabled_ || !last_published_.valid()) {
    return;
  }
  output_publisher_->publish_tf(last_published_, last_published_stamp_);
}

void PoseTwistFusionNode::on_diagnostics_timer()
{
  DiagnosticsBuilder::NodeStatus status;
  status.fusion_method = params_.fusion_method;
  status.trigger_enabled = trigger_enabled_;
  status.plugin_running = plugin_->is_running();
  status.has_estimate = last_estimate_valid_;
  status.extrapolation_ms = last_extrapolation_ms_;
  status.extrapolation_exceeded = last_extrapolation_exceeded_;
  status.processing_time_ms = last_processing_time_ms_;
  status.smoothing_active =
    output_stage_->state().mode == detail::SmoothingState::Mode::kSmoothing;
  status.smoothing_count = output_stage_->smoothing_count();
  status.smoothing_rate_exceeded_count = output_stage_->rate_exceeded_count();
  status.smoothing_rate_exceeded_now = last_smoothing_rate_exceeded_;
  status.covariance_repair_count = output_publisher_->covariance_repair_count();
  status.startup_warnings = startup_warnings_;
  status.startup_infos = startup_infos_;

  pub_diagnostics_->publish(diagnostics_builder_.build(status, plugin_->diagnostics(), now()));
}

void PoseTwistFusionNode::on_trigger(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  trigger_enabled_ = request->data;
  if (!trigger_enabled_) {
    plugin_->reset();
    output_stage_->apply_immediately();
  }
  response->success = true;
  response->message = trigger_enabled_ ? "estimation resumed" : "estimation stopped and reset";
  RCLCPP_INFO_STREAM(get_logger(), "trigger_node_srv: " << response->message);
}

void PoseTwistFusionNode::on_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  plugin_->set_initial_pose(*msg);
  // §5.3.7: initialpose is applied immediately, never smoothed.
  output_stage_->apply_immediately();
}

}  // namespace autoware::pose_twist_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pose_twist_fusion::PoseTwistFusionNode)
