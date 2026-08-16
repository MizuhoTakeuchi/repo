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

#include "plugins/factor_graph_fusion/factor_graph_fusion_plugin.hpp"

#include "autoware/pose_twist_fusion/covariance_convert.hpp"
#include "autoware/pose_twist_fusion/diagnostics_builder.hpp"
#include "plugins/factor_graph_fusion/factors/gnss_lever_arm_factor.hpp"
#include "ros_conversions.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pose_twist_fusion
{

using fg::MeasurementSource;

FactorGraphFusionPlugin::~FactorGraphFusionPlugin()
{
  if (anchor_manager_) {
    anchor_manager_->stop();
  }
}

void FactorGraphFusionPlugin::initialize(rclcpp::Node & node, const Parameters & params)
{
  node_ = &node;
  params_ = params;

  // §5.3.1: in `centrifugal` mode the Euler term is deliberately not compensated
  // (differentiating omega is too noisy); the residual is folded into the
  // accelerometer noise instead of being ignored.
  if (params_.factor_graph_fusion.imu.lever_arm_compensation == "centrifugal") {
    auto & imu = params_.factor_graph_fusion.imu;
    imu.accel_noise_stddev =
      std::sqrt(std::pow(imu.accel_noise_stddev, 2) + std::pow(imu.euler_term_sigma, 2));
  }
  const auto & fg_params = params_.factor_graph_fusion;

  corrector_ = std::make_shared<fg::CovarianceCorrector>(
    fg_params.covariance_correction, fg_params.robust_kernel, fg_params.robust_kernel_param);
  imu_adapter_ = std::make_unique<fg::ImuFrameAdapter>(fg_params.imu);
  imu_buffer_ = std::make_unique<fg::ImuBuffer>(fg_params.buffer.imu_buffer_ms * 1e-3);
  twist_buffer_ =
    std::make_unique<fg::VehicleTwistBuffer>(fg_params.buffer.vehicle_twist_buffer_ms * 1e-3);
  graph1_ = std::make_unique<fg::Graph1Propagator>(fg_params, corrector_);
  pending_ = std::make_unique<fg::PendingMeasurementQueue>();
  slam_gate_ = std::make_unique<fg::SlamPairGate>(fg_params.slam, corrector_);
  initializer_ = std::make_unique<fg::Initializer>(fg_params);

  preint_params_ = gtsam::PreintegrationParams::MakeSharedU(fg_params.imu.gravity);
  preint_params_->accelerometerCovariance =
    gtsam::Matrix33::Identity() * std::pow(fg_params.imu.accel_noise_stddev, 2);
  preint_params_->gyroscopeCovariance =
    gtsam::Matrix33::Identity() * std::pow(fg_params.imu.gyro_noise_stddev, 2);
  preint_params_->integrationCovariance =
    gtsam::Matrix33::Identity() * fg_params.imu.integration_cov;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Extrinsics (design §5.3.1). The TF is not available at construction time yet,
  // so the fallback is installed now and TF is retried on the first tick.
  std::string message;
  imu_adapter_->resolve(*tf_buffer_, params_.base_frame_id, &message);
  extrinsic_message_ = message;
  gnss_lever_arm_ = gtsam::Point3(
    fg_params.gnss.lever_arm_fallback.x, fg_params.gnss.lever_arm_fallback.y,
    fg_params.gnss.lever_arm_fallback.z);
  resolve_gnss_lever_arm();

  anchor_manager_ = std::make_unique<fg::AnchorManager>(
    fg_params, *graph1_, *imu_buffer_, *twist_buffer_, *pending_, gnss_lever_arm_);
  anchor_manager_->start();

  tick_cb_group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tick_timer_ = node.create_wall_timer(
    std::chrono::milliseconds(fg_params.graph2_optimize_interval_ms),
    std::bind(&FactorGraphFusionPlugin::on_tick, this), tick_cb_group_);
}

InputRequirements FactorGraphFusionPlugin::required_inputs() const
{
  const auto & fg_params = params_.factor_graph_fusion;
  InputRequirements req;
  req.imu = true;
  req.imu_topic = params_.imu_topic;
  req.vehicle_twist = fg_params.wheel_speed.use;
  // D19: the raw wheel speed, not the gyro_odometer output, otherwise the IMU would
  // be counted twice (once as a factor, once inside the twist).
  req.vehicle_twist_topic = fg_params.wheel_speed.topic;
  req.ndt_pose = fg_params.ndt.use;
  req.ndt_pose_topic = fg_params.ndt.topic;
  req.gnss_pose = fg_params.gnss.use;
  req.gnss_pose_topic = fg_params.gnss.topic;
  req.slam_odometry = fg_params.slam.use;
  req.slam_odometry_topic = fg_params.slam.topic;
  return req;
}

void FactorGraphFusionPlugin::resolve_gnss_lever_arm()
{
  // r_bg = translation of TF base_link <- gnss_link, falling back to
  // `gnss.lever_arm_fallback` (never to the IMU extrinsic, design §5.3.1).
  const auto & gnss = params_.factor_graph_fusion.gnss;
  if (!gnss.use || !gnss.use_tf_extrinsic || gnss_lever_arm_resolved_) {
    return;
  }
  try {
    const auto transform =
      tf_buffer_->lookupTransform(params_.base_frame_id, gnss.frame_id, tf2::TimePointZero);
    gnss_lever_arm_ = gtsam::Point3(
      transform.transform.translation.x, transform.transform.translation.y,
      transform.transform.translation.z);
    gnss_lever_arm_resolved_ = true;
    factor_builders_lever_arm_message_ =
      "gnss lever arm from TF " + params_.base_frame_id + " <- " + gnss.frame_id;
    if (anchor_manager_) {
      anchor_manager_->set_gnss_lever_arm(gnss_lever_arm_);
    }
  } catch (const tf2::TransformException & e) {
    factor_builders_lever_arm_message_ =
      std::string("gnss lever arm from gnss.lever_arm_fallback (") + e.what() + ")";
  }
}

void FactorGraphFusionPlugin::on_tick()
{
  resolve_gnss_lever_arm();
  // The timer only posts a request; the optimization itself runs on the worker
  // thread so that a slow tick can never delay the 50 Hz output (D17).
  if (!running_.load()) {
    try_initialize(node_->now().seconds());
    return;
  }
  if (anchor_manager_->needs_reinitialization()) {
    RCLCPP_ERROR(
      node_->get_logger(), "[factor_graph_fusion] repeated optimizer failures, re-initializing");
    anchor_manager_->clear_reinitialization_request();
    reset();
    return;
  }
  anchor_manager_->request_tick();
}

void FactorGraphFusionPlugin::try_initialize(double now)
{
  if (running_.load()) {
    return;
  }
  auto anchor = initializer_->try_initialize(now);
  if (!anchor.has_value()) {
    return;
  }
  anchor_manager_->set_initial_anchor(*anchor);
  running_.store(true);
  RCLCPP_INFO(
    node_->get_logger(), "[factor_graph_fusion] initialized (gnss yaw source: %s)",
    initializer_->gnss_yaw_source_used().c_str());
}

void FactorGraphFusionPlugin::on_imu(const sensor_msgs::msg::Imu & msg)
{
  const fg::ImuSample sample = imu_adapter_->convert(msg);
  imu_buffer_->push(sample);
  initializer_->on_imu(sample);
  {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    last_imu_stamp_ = sample.stamp;
  }
  if (running_.load()) {
    graph1_->apply_imu(sample);
  } else {
    try_initialize(node_->now().seconds());
  }
}

void FactorGraphFusionPlugin::on_vehicle_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg)
{
  const auto & ws = params_.factor_graph_fusion.wheel_speed;
  fg::TwistSample sample;
  sample.stamp = rclcpp::Time(msg.header.stamp).seconds();
  sample.vx = msg.twist.twist.linear.x;

  const double reported = msg.twist.covariance[0];
  sample.variance = (std::isfinite(reported) && reported > 0.0)
                      ? reported
                      : ws.fallback_stddev * ws.fallback_stddev;

  twist_buffer_->push(sample);
  initializer_->on_wheel_speed(sample.stamp, sample.vx);
  {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    last_twist_stamp_ = sample.stamp;
  }

  if (!running_.load()) {
    return;
  }
  // The gate decision is taken inside Graph1 and written back to the buffer so that
  // Graph2 and the replay see the very same decision (D42).
  graph1_->apply_wheel_speed(sample);
  twist_buffer_->apply_gate_results({sample});
}

bool FactorGraphFusionPlugin::accept_absolute_pose(
  MeasurementSource source, double stamp, const gtsam::Pose3 & pose,
  const std::array<double, 36> & covariance, std::string * reason)
{
  const auto & fg_params = params_.factor_graph_fusion;
  const auto & timing = fg_params.timing;

  // 1. stamp validity (design §5.3.4 step 1)
  const double now = node_->now().seconds();
  const double age = now - stamp;
  if (age < -timing.future_tolerance_ms * 1e-3) {
    *reason = "future stamp";
    std::lock_guard<std::mutex> lock(counters_mutex_);
    ++dropped_future_;
    return false;
  }
  if (age > timing.max_measurement_delay_ms * 1e-3) {
    *reason = "stale measurement";
    std::lock_guard<std::mutex> lock(counters_mutex_);
    ++dropped_stale_;
    return false;
  }
  if (stamp < anchor_manager_->window_begin()) {
    *reason = "before graph2 window";
    std::lock_guard<std::mutex> lock(counters_mutex_);
    ++dropped_before_window_;
    return false;
  }

  // 2. mu_1(t), Sigma_1(t) from the *internal* estimate (D27)
  const auto state = graph1_->interpolate(stamp);
  if (!state.has_value()) {
    *reason = "too old for the state history";
    std::lock_guard<std::mutex> lock(counters_mutex_);
    ++dropped_stale_;
    return false;
  }

  const auto status = check_input_covariance(covariance);
  if (status == CovarianceStatus::kInvalid) {
    *reason = "invalid covariance";
    std::lock_guard<std::mutex> lock(counters_mutex_);
    ++dropped_invalid_covariance_;
    return false;
  }

  const double speed = state->velocity.norm();
  const gtsam::Matrix6 sigma1 = corrector_->relax_low_speed(state->pose_cov, speed);

  // §5.3.9: after a dropout longer than `long_dropout_s` the first measurement is
  // gated with a relaxed threshold so the estimator can re-acquire the input.
  bool recovery = false;
  {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    const double last_accepted = (source == MeasurementSource::kGnss)   ? last_gnss_accepted_
                                 : (source == MeasurementSource::kNdt)  ? last_ndt_accepted_
                                                                        : last_slam_accepted_;
    recovery = last_accepted > 0.0 && (stamp - last_accepted) > fg_params.failure.long_dropout_s;
  }

  fg::AcceptedMeasurement measurement;
  measurement.stamp = stamp;
  measurement.source = source;

  fg::CovarianceCorrector::Input input;
  if (source == MeasurementSource::kGnss) {
    // 3./4. GNSS: the residual lives in the map frame, so the ROS covariance block
    // is used as is and the lever arm Jacobian maps Sigma_1 into it (§5.3.3b).
    const gtsam::Point3 measured = pose.translation();
    gtsam::Matrix3 r_z;
    if (status == CovarianceStatus::kOk) {
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          r_z(i, j) = covariance[static_cast<size_t>(i * 6 + j)];
        }
      }
    } else {
      r_z = gtsam::Matrix3::Zero();
      r_z(0, 0) = std::pow(fg_params.gnss.fallback_stddev.x, 2);
      r_z(1, 1) = std::pow(fg_params.gnss.fallback_stddev.y, 2);
      r_z(2, 2) = std::pow(fg_params.gnss.fallback_stddev.z, 2);
    }
    const gtsam::Matrix jacobian = fg::gnss_lever_arm_jacobian(state->pose, gnss_lever_arm_);
    const gtsam::Vector3 predicted =
      state->pose.translation() + state->pose.rotation().matrix() * gnss_lever_arm_;

    input.dof = 3;
    input.innovation = predicted - measured;
    input.measurement_covariance = r_z;
    input.innovation_covariance = jacobian * sigma1 * jacobian.transpose() + r_z;
    measurement.value.position = measured;
  } else {
    // NDT / SLAM absolute: H = I_6, so S = Sigma_1 + R_z, both in the body tangent
    // of mu_1 (§5.3.5).
    gtsam::Matrix6 r_z;
    if (status == CovarianceStatus::kOk) {
      r_z = covariance_convert::ros_pose_to_gtsam(to_array(covariance), state->pose.rotation());
    } else {
      r_z = diagonal_pose_cov(
        fg_params.ndt.fallback_pos_stddev, fg_params.ndt.fallback_rot_stddev);
    }
    if (source == MeasurementSource::kSlamAbsolute) {
      r_z *= fg_params.slam.covariance_scale;
    }
    input.dof = 6;
    input.innovation = gtsam::Pose3::Logmap(state->pose.inverse() * pose);
    input.measurement_covariance = r_z;
    input.innovation_covariance = sigma1 + r_z;
    measurement.value.pose = pose;
  }

  input.recovery = recovery;
  const auto correction = corrector_->correct(input);
  measurement.nis = correction.d2;
  measurement.weight = correction.weight;
  measurement.corrected_covariance = correction.corrected_covariance;
  measurement.noise = corrector_->make_noise_model(correction.corrected_covariance);

  {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    if (source == MeasurementSource::kGnss) {
      last_gnss_d2_ = correction.d2;
      last_gnss_w_ = correction.weight;
      last_gnss_stamp_ = stamp;
      if (correction.accepted) {
        last_gnss_accepted_ = stamp;
      } else {
        ++gnss_rejected_;
      }
    } else if (source == MeasurementSource::kNdt) {
      last_ndt_d2_ = correction.d2;
      last_ndt_w_ = correction.weight;
      last_ndt_stamp_ = stamp;
      if (correction.accepted) {
        last_ndt_accepted_ = stamp;
      } else {
        ++ndt_rejected_;
      }
    } else {
      last_slam_d2_ = correction.d2;
      last_slam_w_ = correction.weight;
      last_slam_stamp_ = stamp;
      if (correction.accepted) {
        last_slam_accepted_ = stamp;
      } else {
        ++slam_rejected_;
      }
    }
  }

  if (!correction.accepted) {
    *reason = "rejected by the chi-squared gate";
    return false;
  }

  // 5. hand over; the noise model is frozen from here on (INV-3).
  pending_->push(measurement);
  return true;
}

void FactorGraphFusionPlugin::on_ndt_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  const double stamp = rclcpp::Time(msg.header.stamp).seconds();
  const gtsam::Pose3 pose = to_gtsam(msg.pose.pose);
  initializer_->on_ndt(
    stamp, pose, covariance_convert::ros_pose_to_gtsam(to_array(msg.pose.covariance), pose.rotation()));
  if (!running_.load()) {
    try_initialize(node_->now().seconds());
    return;
  }
  std::string reason;
  if (!accept_absolute_pose(MeasurementSource::kNdt, stamp, pose, msg.pose.covariance, &reason)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "[factor_graph_fusion] ndt dropped: %s",
      reason.c_str());
  }
}

void FactorGraphFusionPlugin::on_gnss_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  const double stamp = rclcpp::Time(msg.header.stamp).seconds();
  const gtsam::Pose3 pose = to_gtsam(msg.pose.pose);

  // A non-identity orientation means the receiver provides a heading (dual antenna),
  // which the initializer may use for the yaw (D41).
  std::optional<double> yaw;
  const auto & q = msg.pose.pose.orientation;
  const bool has_orientation = std::abs(q.w) < 1.0 - 1e-9 || std::abs(q.x) > 1e-9 ||
                               std::abs(q.y) > 1e-9 || std::abs(q.z) > 1e-9;
  if (has_orientation) {
    yaw = pose.rotation().yaw();
  }
  gtsam::Matrix3 position_cov = gtsam::Matrix3::Identity();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      position_cov(i, j) = msg.pose.covariance[static_cast<size_t>(i * 6 + j)];
    }
  }
  initializer_->on_gnss(stamp, pose.translation(), position_cov, yaw, msg.pose.covariance[35]);

  if (!running_.load()) {
    try_initialize(node_->now().seconds());
    return;
  }
  std::string reason;
  if (!accept_absolute_pose(MeasurementSource::kGnss, stamp, pose, msg.pose.covariance, &reason)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "[factor_graph_fusion] gnss dropped: %s",
      reason.c_str());
  }
}

void FactorGraphFusionPlugin::handle_slam_relative(
  double stamp, const gtsam::Pose3 & pose, const std::array<double, 36> & covariance)
{
  const auto & fg_params = params_.factor_graph_fusion;
  const auto state = graph1_->interpolate(stamp);
  if (!state.has_value()) {
    return;
  }
  const gtsam::Matrix6 odom_cov =
    covariance_convert::ros_pose_to_gtsam(to_array(covariance), pose.rotation());

  // The predicted side is the Graph1 **relative** motion over the same interval; the
  // absolute Sigma_1 must not be used here because the anchor error cancels in a
  // relative quantity and would make the gate useless (D38).
  gtsam::Pose3 delta_graph1;
  gtsam::Matrix6 sigma_relative = gtsam::Matrix6::Identity() * 1e-4;
  // The gate owns the previous sample, so ask it for the interval to evaluate the
  // Graph1 relative motion over.
  const double t_previous = slam_gate_->previous_stamp();
  if (t_previous > 0.0) {
    const auto previous_state = graph1_->interpolate(t_previous);
    if (previous_state.has_value()) {
      delta_graph1 = previous_state->pose.inverse() * state->pose;
      sigma_relative = graph1_->propagate_interval(t_previous, stamp, *imu_buffer_);
    }
  }
  const fg::SlamPairGate::PairResult result =
    slam_gate_->evaluate(stamp, pose, odom_cov, delta_graph1, sigma_relative);

  fg::AcceptedMeasurement measurement;
  measurement.stamp = stamp;
  measurement.source = MeasurementSource::kSlamRelative;
  measurement.value.pose = pose;

  if (!result.has_pair) {
    // First sample after a dropout: it only becomes the start point of the next pair.
    measurement.is_interpolation_anchor = true;
    measurement.contribution = fg::Contribution::kIntentionallyExcluded;
    measurement.corrected_covariance = odom_cov;
    measurement.noise = corrector_->make_noise_model(odom_cov);
    pending_->push(measurement);
    return;
  }

  measurement.value.relative = result.relative;
  measurement.value.previous_stamp = result.previous_stamp;
  measurement.nis = result.d2;
  measurement.weight = result.weight;
  measurement.corrected_covariance =
    result.covariance.size() != 0 ? result.covariance : Eigen::MatrixXd(odom_cov);
  measurement.noise = corrector_->make_noise_model(measurement.corrected_covariance);

  {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    last_slam_d2_ = result.d2;
    last_slam_w_ = result.weight;
    last_slam_stamp_ = stamp;
    if (!result.accepted) {
      ++slam_rejected_;
    }
  }

  if (!result.accepted) {
    // Kept as an interpolation anchor so the next pair still has a start point; the
    // enum records that not building a factor was a decision, not an omission.
    measurement.is_interpolation_anchor = true;
    measurement.contribution = fg::Contribution::kIntentionallyExcluded;
  }
  (void)fg_params;
  pending_->push(measurement);
}

void FactorGraphFusionPlugin::on_slam_odometry(const nav_msgs::msg::Odometry & msg)
{
  if (!running_.load()) {
    return;
  }
  const double stamp = rclcpp::Time(msg.header.stamp).seconds();
  const gtsam::Pose3 pose = to_gtsam(msg.pose.pose);
  if (params_.factor_graph_fusion.slam.use_as == "absolute") {
    std::string reason;
    accept_absolute_pose(
      MeasurementSource::kSlamAbsolute, stamp, pose, msg.pose.covariance, &reason);
    return;
  }
  handle_slam_relative(stamp, pose, msg.pose.covariance);
}

void FactorGraphFusionPlugin::set_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  const double stamp = rclcpp::Time(msg.header.stamp).seconds();
  const gtsam::Pose3 pose = to_gtsam(msg.pose.pose);
  const gtsam::Matrix6 cov =
    covariance_convert::ros_pose_to_gtsam(to_array(msg.pose.covariance), pose.rotation());
  initializer_->on_initial_pose(stamp, pose, cov);
  // A new initial pose always restarts the estimator; it is applied immediately and
  // never smoothed (§5.3.7).
  running_.store(false);
  anchor_manager_->reset();
  slam_gate_->reset();
  try_initialize(node_->now().seconds());
}

void FactorGraphFusionPlugin::reset()
{
  running_.store(false);
  anchor_manager_->reset();
  slam_gate_->reset();
  initializer_->reset();
  imu_buffer_->clear();
  twist_buffer_->clear();
  pending_->clear();
}

InternalEstimate FactorGraphFusionPlugin::evaluate(
  const fg::Graph1Propagator::Snapshot & snapshot, double target) const
{
  InternalEstimate out;
  if (!snapshot.valid) {
    return out;
  }
  const double max_extrapolation_s = params_.output.max_extrapolation_ms * 1e-3;
  const double clamped = std::min(target, snapshot.stamp + max_extrapolation_s);
  const double dt = clamped - snapshot.stamp;

  gtsam::NavState state = snapshot.state;
  fg::Matrix15 covariance = snapshot.covariance;

  if (dt > 1e-9) {
    // Forward integration with the last measurement only; the output path never
    // touches graph1_mutex_ nor the optimizer (D17).
    gtsam::PreintegratedImuMeasurements step(preint_params_, snapshot.bias);
    step.integrateMeasurement(snapshot.last_accel, snapshot.last_gyro, dt);
    gtsam::Matrix99 h_state = gtsam::Matrix99::Identity();
    gtsam::Matrix96 h_bias = gtsam::Matrix96::Zero();
    state = step.predict(snapshot.state, snapshot.bias, h_state, h_bias);

    fg::Matrix15 f = fg::Matrix15::Identity();
    f.block<9, 9>(0, 0) = h_state;
    f.block<9, 6>(0, fg::kBiasIndex) = h_bias;
    fg::Matrix15 q = fg::Matrix15::Zero();
    q.block<9, 9>(0, 0) = step.preintMeasCov();
    covariance = f * covariance * f.transpose() + q;
  }

  out.valid = true;
  out.stamp = clamped;
  out.pose = state.pose();
  out.biased_pose = out.pose;  // no yaw bias state in this mode (design §3.2)
  out.velocity_world = state.v();
  out.angular_velocity_body = snapshot.last_gyro - snapshot.bias.gyroscope();
  out.yaw_bias = 0.0;
  out.pose_cov = covariance.block<6, 6>(fg::kPoseIndex, fg::kPoseIndex);

  // twist covariance in the body tangent order [omega; nu]
  const gtsam::Matrix3 rotation = out.pose.rotation().matrix();
  gtsam::Matrix6 twist_cov = gtsam::Matrix6::Zero();
  twist_cov.topLeftCorner<3, 3>() =
    gtsam::Matrix3::Identity() * std::pow(params_.factor_graph_fusion.imu.gyro_noise_stddev, 2) +
    covariance.block<3, 3>(fg::kBiasIndex + 3, fg::kBiasIndex + 3);
  twist_cov.bottomRightCorner<3, 3>() =
    rotation.transpose() * covariance.block<3, 3>(fg::kVelIndex, fg::kVelIndex) * rotation;
  out.twist_cov = twist_cov;
  return out;
}

InternalEstimate FactorGraphFusionPlugin::estimate_internal(const rclcpp::Time & stamp) const
{
  const auto snapshot = graph1_->latest_snapshot();
  if (!snapshot) {
    return InternalEstimate{};
  }
  return evaluate(*snapshot, stamp.seconds());
}

std::optional<InternalEstimate> FactorGraphFusionPlugin::estimate_internal_previous(
  const rclcpp::Time & stamp) const
{
  const auto snapshot = anchor_manager_->previous_snapshot();
  if (!snapshot) {
    return std::nullopt;
  }
  // The pre-reset state is propagated to the *same* instant as the current one, so
  // the difference is a pure anchor jump and contains no vehicle motion (D37).
  InternalEstimate out;
  const double target = stamp.seconds();
  fg::Graph1Propagator::Snapshot propagated = *snapshot;
  const auto samples = imu_buffer_->since(propagated.stamp);
  for (const auto & sample : samples) {
    if (sample.stamp > target) {
      break;
    }
    const double dt = sample.stamp - propagated.stamp;
    if (dt <= 0.0) {
      continue;
    }
    gtsam::PreintegratedImuMeasurements step(preint_params_, propagated.bias);
    step.integrateMeasurement(sample.accel, sample.gyro, dt);
    gtsam::Matrix99 h_state = gtsam::Matrix99::Identity();
    gtsam::Matrix96 h_bias = gtsam::Matrix96::Zero();
    propagated.state = step.predict(propagated.state, propagated.bias, h_state, h_bias);
    fg::Matrix15 f = fg::Matrix15::Identity();
    f.block<9, 9>(0, 0) = h_state;
    f.block<9, 6>(0, fg::kBiasIndex) = h_bias;
    fg::Matrix15 q = fg::Matrix15::Zero();
    q.block<9, 9>(0, 0) = step.preintMeasCov();
    propagated.covariance = f * propagated.covariance * f.transpose() + q;
    propagated.stamp = sample.stamp;
    propagated.last_accel = sample.accel;
    propagated.last_gyro = sample.gyro;
  }
  out = evaluate(propagated, target);
  if (!out.valid) {
    return std::nullopt;
  }
  return out;
}

EstimateGeneration FactorGraphFusionPlugin::generation() const
{
  return anchor_manager_->generation();
}

diagnostic_msgs::msg::DiagnosticStatus FactorGraphFusionPlugin::diagnostics() const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "pose_twist_fusion: factor_graph_fusion";
  status.hardware_id = "pose_twist_fusion";

  const auto stats = anchor_manager_->stats();
  const double now = node_->now().seconds();
  const auto & fg_params = params_.factor_graph_fusion;

  std::lock_guard<std::mutex> lock(counters_mutex_);
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "OK";

  if (!running_.load()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = initializer_->message();
  }

  const auto check_timeout = [&](double last, int timeout_ms, const char * name, unsigned char level) {
    if (last <= 0.0) {
      return;
    }
    if ((now - last) * 1000.0 > timeout_ms) {
      status.level = std::max(status.level, level);
      status.message = std::string(name) + " timeout";
    }
  };
  check_timeout(
    last_imu_stamp_, fg_params.failure.imu_timeout_ms, "imu",
    diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  check_timeout(
    last_twist_stamp_, fg_params.wheel_speed.timeout_ms, "wheel_speed",
    diagnostic_msgs::msg::DiagnosticStatus::WARN);
  check_timeout(
    last_ndt_stamp_, fg_params.ndt.timeout_ms, "ndt",
    diagnostic_msgs::msg::DiagnosticStatus::WARN);
  check_timeout(
    last_gnss_stamp_, fg_params.gnss.timeout_ms, "gnss",
    diagnostic_msgs::msg::DiagnosticStatus::WARN);
  if (fg_params.slam.use) {
    check_timeout(
      last_slam_stamp_, fg_params.slam.timeout_ms, "slam",
      diagnostic_msgs::msg::DiagnosticStatus::WARN);
  }

  if (stats.optimizer_failures > 0 && stats.consecutive_failures > 0) {
    status.level = std::max<unsigned char>(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    status.message = "graph2 optimization failing: " + stats.last_error;
  }
  if (stats.inv5_blocked > 0) {
    status.level = std::max<unsigned char>(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    status.message = "INV-5 violation: a measurement would have been pruned unused";
  }
  if (stats.straddling_factors > 0) {
    status.level = std::max<unsigned char>(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    status.message = "INV-2 violation: a factor straddles the marginalization boundary";
  }

  add_key_value(status, "state", std::string(running_.load() ? "RUNNING" : "NOT_RUNNING"));
  add_key_value(status, "init_message", initializer_->message());
  add_key_value(status, "gnss_yaw_source", initializer_->gnss_yaw_source_used());
  add_key_value(status, "imu_extrinsic", extrinsic_message_);
  add_key_value(status, "gnss_lever_arm", factor_builders_lever_arm_message_);
  add_key_value(status, "generation", anchor_manager_->generation());
  add_key_value(status, "graph2_ticks", stats.ticks);
  add_key_value(status, "graph2_skipped_ticks", stats.skipped_ticks);
  add_key_value(status, "graph2_failures", stats.optimizer_failures);
  add_key_value(status, "graph2_forced_rebuilds", stats.forced_rebuilds);
  add_key_value(status, "graph2_nodes", static_cast<std::uint64_t>(stats.node_count));
  add_key_value(status, "graph2_optimize_ms", stats.last_optimize_time_ms);
  add_key_value(status, "measurement_history", static_cast<std::uint64_t>(stats.history_size));
  add_key_value(status, "history_overflow", stats.history_overflow);
  add_key_value(status, "dropped_before_window", stats.dropped_before_window);
  add_key_value(status, "dropped_not_yet_built", stats.dropped_not_yet_built);
  add_key_value(status, "inv5_blocked", stats.inv5_blocked);
  add_key_value(status, "straddling_factors", stats.straddling_factors);
  add_key_value(status, "slam_pairs_split", static_cast<std::uint64_t>(stats.slam_pairs_split));
  add_key_value(status, "wheel_speed_rejected", graph1_->wheel_speed_reject_count());
  add_key_value(status, "gnss_rejected", gnss_rejected_);
  add_key_value(status, "ndt_rejected", ndt_rejected_);
  add_key_value(status, "slam_rejected", slam_rejected_);
  add_key_value(status, "slam_reject_chi2", slam_gate_->chi2_reject_count());
  add_key_value(status, "slam_reject_dynamic", slam_gate_->dynamic_reject_count());
  return status;
}

std::vector<DebugValue> FactorGraphFusionPlugin::debug_values() const
{
  const auto stats = anchor_manager_->stats();
  std::lock_guard<std::mutex> lock(counters_mutex_);
  return {
    {"gnss_d2", last_gnss_d2_},
    {"gnss_w", last_gnss_w_},
    {"gnss_rejected", static_cast<double>(gnss_rejected_)},
    {"ndt_d2", last_ndt_d2_},
    {"ndt_w", last_ndt_w_},
    {"ndt_rejected", static_cast<double>(ndt_rejected_)},
    {"slam_d2", last_slam_d2_},
    {"slam_w", last_slam_w_},
    {"slam_reject_chi2", static_cast<double>(slam_gate_->chi2_reject_count())},
    {"slam_reject_dynamic", static_cast<double>(slam_gate_->dynamic_reject_count())},
    {"wheel_speed_rejected", static_cast<double>(graph1_->wheel_speed_reject_count())},
    {"dropped_before_window", static_cast<double>(stats.dropped_before_window)},
    {"dropped_not_yet_built", static_cast<double>(stats.dropped_not_yet_built)},
    {"graph2_optimize_ms", stats.last_optimize_time_ms},
    {"graph2_nodes", static_cast<double>(stats.node_count)},
    {"measurement_history", static_cast<double>(stats.history_size)},
  };
}

bool FactorGraphFusionPlugin::is_running() const
{
  return running_.load() && graph1_->has_anchor();
}

}  // namespace autoware::pose_twist_fusion

PLUGINLIB_EXPORT_CLASS(
  autoware::pose_twist_fusion::FactorGraphFusionPlugin, autoware::pose_twist_fusion::FusionPlugin)
