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

// Portions of this file are a port of LIO-SAM's src/imuPreintegration.cpp,
// distributed by Tixiao Shan under the BSD-3-Clause license; see
// src/universe/external/LIO-SAM/LICENSE for the full text.

#include "plugins/lio_sam/lio_sam_imu_preintegration_plugin.hpp"

#include "autoware/pose_twist_fusion/covariance_convert.hpp"
#include "autoware/pose_twist_fusion/diagnostics_builder.hpp"
#include "ros_conversions.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace autoware::pose_twist_fusion
{

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace
{
constexpr double kDeltaT = 0.01;  // LIO-SAM "delta_t"
constexpr double kDefaultImuPeriod = 1.0 / 500.0;
}  // namespace

void LioSamImuPreintegrationPlugin::initialize(rclcpp::Node & node, const Parameters & params)
{
  node_ = &node;
  params_ = params;
  t_base_imu_ = from_pose_param(params.lio_sam.extrinsic_imu_to_base);

  preint_params_ = gtsam::PreintegrationParams::MakeSharedU(params.lio_sam.imu_gravity);
  preint_params_->accelerometerCovariance =
    gtsam::Matrix33::Identity() * std::pow(params.lio_sam.imu_acc_noise, 2);
  preint_params_->gyroscopeCovariance =
    gtsam::Matrix33::Identity() * std::pow(params.lio_sam.imu_gyr_noise, 2);
  preint_params_->integrationCovariance = gtsam::Matrix33::Identity() * std::pow(1e-4, 2);

  prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
  prior_vel_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);
  prior_bias_noise_ = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
  correction_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
  correction_noise2_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished());
  noise_model_between_bias_ =
    (gtsam::Vector(6) << params.lio_sam.imu_acc_bias_n, params.lio_sam.imu_acc_bias_n,
     params.lio_sam.imu_acc_bias_n, params.lio_sam.imu_gyr_bias_n, params.lio_sam.imu_gyr_bias_n,
     params.lio_sam.imu_gyr_bias_n)
      .finished();

  const gtsam::imuBias::ConstantBias zero_bias;
  imu_integrator_opt_ =
    std::make_shared<gtsam::PreintegratedImuMeasurements>(preint_params_, zero_bias);
}

InputRequirements LioSamImuPreintegrationPlugin::required_inputs() const
{
  InputRequirements req;
  req.imu = true;
  req.imu_topic = params_.lio_sam.imu_topic;
  // D3: the absolute pose input is the LiDAR odometry of an externally launched
  // LIO-SAM mapOptimization; no wheel speed constraint is added.
  req.slam_odometry = true;
  req.slam_odometry_topic = params_.lio_sam.lidar_odometry_topic;
  return req;
}

void LioSamImuPreintegrationPlugin::reset_optimization()
{
  gtsam::ISAM2Params opt_parameters;
  opt_parameters.relinearizeThreshold = 0.1;
  opt_parameters.relinearizeSkip = 1;
  optimizer_ = gtsam::ISAM2(opt_parameters);
  graph_factors_ = gtsam::NonlinearFactorGraph();
  graph_values_ = gtsam::Values();
}

void LioSamImuPreintegrationPlugin::reset_params()
{
  done_first_opt_ = false;
  system_initialized_ = false;
  last_imu_t_opt_ = -1.0;
}

bool LioSamImuPreintegrationPlugin::failure_detection(
  const gtsam::Vector3 & velocity, const gtsam::imuBias::ConstantBias & bias) const
{
  if (velocity.norm() > params_.lio_sam.failure_velocity_mps) {
    return true;
  }
  return bias.accelerometer().norm() > params_.lio_sam.failure_bias ||
         bias.gyroscope().norm() > params_.lio_sam.failure_bias;
}

void LioSamImuPreintegrationPlugin::on_imu(const sensor_msgs::msg::Imu & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // The original imuConverter rotates the measurement into the body frame; here the
  // body frame is base_link (design §5.3.1 keeps one convention for all modes).
  sensor_msgs::msg::Imu converted = msg;
  const gtsam::Rot3 & r = t_base_imu_.rotation();
  const gtsam::Vector3 acc = r * to_gtsam(msg.linear_acceleration);
  const gtsam::Vector3 gyr = r * to_gtsam(msg.angular_velocity);
  converted.linear_acceleration.x = acc.x();
  converted.linear_acceleration.y = acc.y();
  converted.linear_acceleration.z = acc.z();
  converted.angular_velocity.x = gyr.x();
  converted.angular_velocity.y = gyr.y();
  converted.angular_velocity.z = gyr.z();

  imu_que_opt_.push_back(converted);
  imu_que_imu_.push_back(converted);

  if (!done_first_opt_ || !current_) {
    return;
  }

  const double imu_time = rclcpp::Time(converted.header.stamp).seconds();
  const auto integrate = [&](Snapshot & snapshot) {
    const double dt = (snapshot.last_imu_time < 0.0) ? kDefaultImuPeriod
                                                     : (imu_time - snapshot.last_imu_time);
    if (dt > 0.0) {
      snapshot.pim->integrateMeasurement(acc, gyr, dt);
    }
    snapshot.last_imu_time = imu_time;
    snapshot.last_accel = acc;
    snapshot.last_gyro = gyr;
  };
  integrate(*current_);
  if (previous_) {
    // Keep the previous generation propagating too, so OutputStage can compare both
    // at the same instant instead of comparing different times (D37).
    integrate(*previous_);
  }
}

lio_sam::Matrix15 LioSamImuPreintegrationPlugin::anchor_covariance(
  const gtsam::Values & result, std::uint64_t key) const
{
  lio_sam::Matrix15 covariance = lio_sam::Matrix15::Identity();
  try {
    gtsam::Marginals marginals(optimizer_.getFactorsUnsafe(), result);
    gtsam::JointMarginal joint =
      marginals.jointMarginalCovariance(gtsam::KeyVector{X(key), V(key), B(key)});
    covariance.block<6, 6>(0, 0) = joint.at(X(key), X(key));
    covariance.block<6, 3>(0, 6) = joint.at(X(key), V(key));
    covariance.block<6, 6>(0, 9) = joint.at(X(key), B(key));
    covariance.block<3, 6>(6, 0) = joint.at(V(key), X(key));
    covariance.block<3, 3>(6, 6) = joint.at(V(key), V(key));
    covariance.block<3, 6>(6, 9) = joint.at(V(key), B(key));
    covariance.block<6, 6>(9, 0) = joint.at(B(key), X(key));
    covariance.block<6, 3>(9, 6) = joint.at(B(key), V(key));
    covariance.block<6, 6>(9, 9) = joint.at(B(key), B(key));
  } catch (const std::exception & e) {
    // Never silently return a fabricated covariance: fall back to the marginals and
    // flag it, because dropping the cross terms is not conservative (D33).
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "[lio_sam] jointMarginalCovariance failed (%s); falling back to block diagonal", e.what());
    covariance.setZero();
    try {
      covariance.block<6, 6>(0, 0) = optimizer_.marginalCovariance(X(key));
      covariance.block<3, 3>(6, 6) = optimizer_.marginalCovariance(V(key));
      covariance.block<6, 6>(9, 9) = optimizer_.marginalCovariance(B(key));
    } catch (const std::exception &) {
      covariance = lio_sam::Matrix15::Identity();
    }
    const_cast<LioSamImuPreintegrationPlugin *>(this)->marginals_failed_ = true;
  }
  return covariance;
}

void LioSamImuPreintegrationPlugin::on_slam_odometry(const nav_msgs::msg::Odometry & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const double current_correction_time = rclcpp::Time(msg.header.stamp).seconds();
  if (imu_que_opt_.empty()) {
    return;
  }

  const gtsam::Pose3 lidar_pose = to_gtsam(msg.pose.pose);
  // Original degenerate flag convention: covariance[0] == 1 selects the loose noise.
  const bool degenerate = static_cast<int>(msg.pose.covariance[0]) == 1;

  if (!system_initialized_) {
    reset_optimization();
    while (!imu_que_opt_.empty()) {
      if (
        rclcpp::Time(imu_que_opt_.front().header.stamp).seconds() <
        current_correction_time - kDeltaT) {
        last_imu_t_opt_ = rclcpp::Time(imu_que_opt_.front().header.stamp).seconds();
        imu_que_opt_.pop_front();
      } else {
        break;
      }
    }
    prev_pose_ = lidar_pose;
    graph_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), prev_pose_, prior_pose_noise_));
    prev_vel_ = gtsam::Vector3::Zero();
    graph_factors_.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), prev_vel_, prior_vel_noise_));
    prev_bias_ = gtsam::imuBias::ConstantBias();
    graph_factors_.add(
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), prev_bias_, prior_bias_noise_));
    graph_values_.insert(X(0), prev_pose_);
    graph_values_.insert(V(0), prev_vel_);
    graph_values_.insert(B(0), prev_bias_);
    optimizer_.update(graph_factors_, graph_values_);
    graph_factors_.resize(0);
    graph_values_.clear();

    imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);
    prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);

    auto snapshot = std::make_shared<Snapshot>();
    snapshot->state = prev_state_;
    snapshot->bias = prev_bias_;
    snapshot->pim =
      std::make_shared<gtsam::PreintegratedImuMeasurements>(preint_params_, prev_bias_);
    snapshot->anchor_time = current_correction_time;
    snapshot->last_imu_time = -1.0;
    snapshot->covariance = anchor_covariance(optimizer_.calculateEstimate(), 0);
    previous_.reset();
    current_ = snapshot;

    key_ = 1;
    system_initialized_ = true;
    done_first_opt_ = true;
    ++generation_;
    last_odometry_time_ = current_correction_time;
    return;
  }

  if (key_ == static_cast<std::uint64_t>(params_.lio_sam.reset_key_count)) {
    // Original reset discipline: carry X / V / B priors from the marginals.
    const auto updated_pose_noise =
      gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(X(key_ - 1)));
    const auto updated_vel_noise =
      gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(V(key_ - 1)));
    const auto updated_bias_noise =
      gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(B(key_ - 1)));
    reset_optimization();
    graph_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), prev_pose_, updated_pose_noise));
    graph_factors_.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), prev_vel_, updated_vel_noise));
    graph_factors_.add(
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), prev_bias_, updated_bias_noise));
    graph_values_.insert(X(0), prev_pose_);
    graph_values_.insert(V(0), prev_vel_);
    graph_values_.insert(B(0), prev_bias_);
    optimizer_.update(graph_factors_, graph_values_);
    graph_factors_.resize(0);
    graph_values_.clear();
    key_ = 1;
  }

  // 1. integrate the IMU up to the correction time and optimize
  while (!imu_que_opt_.empty()) {
    const auto & this_imu = imu_que_opt_.front();
    const double imu_time = rclcpp::Time(this_imu.header.stamp).seconds();
    if (imu_time >= current_correction_time - kDeltaT) {
      break;
    }
    const double dt = (last_imu_t_opt_ < 0.0) ? kDefaultImuPeriod : (imu_time - last_imu_t_opt_);
    if (dt > 0.0) {
      imu_integrator_opt_->integrateMeasurement(
        to_gtsam(this_imu.linear_acceleration), to_gtsam(this_imu.angular_velocity), dt);
    }
    last_imu_t_opt_ = imu_time;
    imu_que_opt_.pop_front();
  }

  graph_factors_.add(gtsam::ImuFactor(
    X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_ - 1), *imu_integrator_opt_));
  graph_factors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
    B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
    gtsam::noiseModel::Diagonal::Sigmas(
      std::sqrt(imu_integrator_opt_->deltaTij()) * noise_model_between_bias_)));
  graph_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(
    X(key_), lidar_pose, degenerate ? correction_noise2_ : correction_noise_));

  const gtsam::NavState prop_state = imu_integrator_opt_->predict(prev_state_, prev_bias_);
  graph_values_.insert(X(key_), prop_state.pose());
  graph_values_.insert(V(key_), prop_state.v());
  graph_values_.insert(B(key_), prev_bias_);

  optimizer_.update(graph_factors_, graph_values_);
  optimizer_.update();
  graph_factors_.resize(0);
  graph_values_.clear();

  const gtsam::Values result = optimizer_.calculateEstimate();
  prev_pose_ = result.at<gtsam::Pose3>(X(key_));
  prev_vel_ = result.at<gtsam::Vector3>(V(key_));
  prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
  prev_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(key_));
  imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);

  if (failure_detection(prev_vel_, prev_bias_)) {
    RCLCPP_WARN(node_->get_logger(), "[lio_sam] failure detected, resetting IMU preintegration");
    ++failure_count_;
    reset_params();
    return;
  }

  const lio_sam::Matrix15 covariance = anchor_covariance(result, key_);

  // 2. after optimization, re-propagate the high rate integrator (original :454-462)
  double last_imu_qt = -1.0;
  while (!imu_que_imu_.empty() && rclcpp::Time(imu_que_imu_.front().header.stamp).seconds() <
                                    current_correction_time - kDeltaT) {
    last_imu_qt = rclcpp::Time(imu_que_imu_.front().header.stamp).seconds();
    imu_que_imu_.pop_front();
  }

  auto snapshot = std::make_shared<Snapshot>();
  snapshot->state = prev_state_;
  snapshot->bias = prev_bias_;
  snapshot->pim = std::make_shared<gtsam::PreintegratedImuMeasurements>(preint_params_, prev_bias_);
  snapshot->anchor_time = current_correction_time;
  snapshot->last_imu_time = last_imu_qt;
  snapshot->covariance = covariance;

  for (const auto & imu : imu_que_imu_) {
    const double imu_time = rclcpp::Time(imu.header.stamp).seconds();
    const double dt =
      (snapshot->last_imu_time < 0.0) ? kDefaultImuPeriod : (imu_time - snapshot->last_imu_time);
    if (dt > 0.0) {
      snapshot->pim->integrateMeasurement(
        to_gtsam(imu.linear_acceleration), to_gtsam(imu.angular_velocity), dt);
    }
    snapshot->last_imu_time = imu_time;
    snapshot->last_accel = to_gtsam(imu.linear_acceleration);
    snapshot->last_gyro = to_gtsam(imu.angular_velocity);
  }

  previous_ = current_;
  current_ = snapshot;
  ++generation_;
  ++key_;
  done_first_opt_ = true;
  last_odometry_time_ = current_correction_time;
}

InternalEstimate LioSamImuPreintegrationPlugin::evaluate(
  const Snapshot & snapshot, double stamp) const
{
  InternalEstimate out;
  const double evaluated_at = snapshot.last_imu_time > 0.0 ? snapshot.last_imu_time
                                                           : snapshot.anchor_time;
  const double max_extrapolation_s = params_.output.max_extrapolation_ms * 1e-3;
  const double target = std::min(stamp, evaluated_at + max_extrapolation_s);

  // Extrapolate the residual interval with the last measurement so that the output
  // refers to the requested instant rather than to the last IMU sample.
  gtsam::PreintegratedImuMeasurements pim = *snapshot.pim;
  const double residual_dt = target - evaluated_at;
  if (residual_dt > 1e-6 && snapshot.last_imu_time > 0.0) {
    pim.integrateMeasurement(snapshot.last_accel, snapshot.last_gyro, residual_dt);
  }

  const gtsam::NavState state = pim.predict(snapshot.state, snapshot.bias);

  out.valid = true;
  out.stamp = target;
  out.pose = state.pose();
  out.biased_pose = out.pose;  // no yaw bias state in this mode (design §3.2)
  out.velocity_world = state.v();
  out.angular_velocity_body = snapshot.last_gyro - snapshot.bias.gyroscope();
  out.yaw_bias = 0.0;

  if (params_.lio_sam.covariance_source == "fixed") {
    out.pose_cov = diagonal_pose_cov(
      params_.lio_sam.fallback_pose_pos_stddev, params_.lio_sam.fallback_pose_rot_stddev);
    out.twist_cov = diagonal_pose_cov(
      params_.lio_sam.fallback_twist_lin_stddev, params_.lio_sam.fallback_twist_ang_stddev);
    return out;
  }

  const lio_sam::PropagatedCovariance propagated = lio_sam::propagate_joint(
    pim, snapshot.state, snapshot.bias, snapshot.covariance, std::max(target - snapshot.anchor_time, 0.0),
    params_.lio_sam.imu_acc_bias_n, params_.lio_sam.imu_gyr_bias_n);

  out.pose_cov = propagated.pose;
  // twist: world velocity covariance rotated into base_link; angular part from the
  // gyro noise plus the propagated gyro bias variance (design §5.2.1).
  const gtsam::Matrix3 r = out.pose.rotation().matrix();
  gtsam::Matrix6 twist_cov = gtsam::Matrix6::Zero();
  twist_cov.bottomRightCorner<3, 3>() = r.transpose() * propagated.velocity * r;
  twist_cov.topLeftCorner<3, 3>() =
    gtsam::Matrix3::Identity() * std::pow(params_.lio_sam.imu_gyr_noise, 2) +
    propagated.bias.bottomRightCorner<3, 3>();
  out.twist_cov = twist_cov;
  return out;
}

InternalEstimate LioSamImuPreintegrationPlugin::estimate_internal(const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_) {
    return InternalEstimate{};
  }
  return evaluate(*current_, stamp.seconds());
}

std::optional<InternalEstimate> LioSamImuPreintegrationPlugin::estimate_internal_previous(
  const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!previous_) {
    return std::nullopt;
  }
  return evaluate(*previous_, stamp.seconds());
}

EstimateGeneration LioSamImuPreintegrationPlugin::generation() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return generation_;
}

void LioSamImuPreintegrationPlugin::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  reset_optimization();
  reset_params();
  imu_que_opt_.clear();
  imu_que_imu_.clear();
  current_.reset();
  previous_.reset();
  key_ = 1;
}

diagnostic_msgs::msg::DiagnosticStatus LioSamImuPreintegrationPlugin::diagnostics() const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "pose_twist_fusion: lio_sam_imu_preintegration";
  status.hardware_id = "pose_twist_fusion";
  std::lock_guard<std::mutex> lock(mutex_);
  status.level = system_initialized_ ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                     : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = system_initialized_ ? "OK" : "waiting for lidar odometry";
  add_key_value(status, "generation", generation_);
  add_key_value(status, "key", key_);
  add_key_value(status, "failure_count", failure_count_);
  add_key_value(status, "imu_queue", static_cast<std::uint64_t>(imu_que_imu_.size()));
  add_key_value(status, "covariance_source", params_.lio_sam.covariance_source);
  if (marginals_failed_) {
    status.level = std::max<unsigned char>(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    status.message = "joint marginal covariance unavailable; using block diagonal";
  }
  return status;
}

bool LioSamImuPreintegrationPlugin::is_running() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return system_initialized_ && current_ != nullptr;
}

}  // namespace autoware::pose_twist_fusion

PLUGINLIB_EXPORT_CLASS(
  autoware::pose_twist_fusion::LioSamImuPreintegrationPlugin,
  autoware::pose_twist_fusion::FusionPlugin)
