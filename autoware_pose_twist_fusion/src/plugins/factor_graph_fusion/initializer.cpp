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

#include "plugins/factor_graph_fusion/initializer.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace autoware::pose_twist_fusion::fg
{

namespace
{
constexpr double kStaticSpeedThreshold = 0.05;   // m/s
constexpr double kStaticOmegaThreshold = 0.01;   // rad/s
constexpr double kCourseMinDistance = 2.0;       // m (design §5.3.8, source 2)
}  // namespace

Initializer::Initializer(const FactorGraphParams & params) : params_(params)
{
}

void Initializer::reset()
{
  state_ = State::kUninitialized;
  message_ = "waiting for the initial pose";
  gnss_yaw_source_used_ = "none";
  static_window_.clear();
  imu_count_ = 0;
  static_since_ = -1.0;
  bias_init_started_ = -1.0;
  has_wheel_ = false;
  moving_since_ = -1.0;
  initial_pose_.reset();
  ndt_pose_.reset();
  gnss_samples_.clear();
  first_seen_ = -1.0;
}

void Initializer::on_imu(const ImuSample & sample)
{
  if (imu_count_ == 0) {
    first_imu_stamp_ = sample.stamp;
  }
  latest_imu_stamp_ = sample.stamp;
  ++imu_count_;

  const bool imu_static = sample.gyro.norm() < kStaticOmegaThreshold;
  const bool wheel_static = !has_wheel_ || std::abs(latest_vx_) < kStaticSpeedThreshold;
  if (imu_static && wheel_static) {
    if (static_since_ < 0.0) {
      static_since_ = sample.stamp;
      static_window_.clear();
    }
    static_window_.push_back(sample);
  } else {
    static_since_ = -1.0;
    static_window_.clear();
  }

  // Keep the window bounded even when the vehicle stands still for a long time.
  const size_t max_window = 4000;
  if (static_window_.size() > max_window) {
    static_window_.erase(static_window_.begin(), static_window_.begin() + (static_window_.size() - max_window));
  }
}

void Initializer::on_wheel_speed(double stamp, double vx)
{
  latest_wheel_stamp_ = stamp;
  latest_vx_ = vx;
  has_wheel_ = true;
  if (std::abs(vx) >= params_.initialization.gnss_yaw_min_speed_mps) {
    if (moving_since_ < 0.0) {
      moving_since_ = stamp;
    }
  } else {
    moving_since_ = -1.0;
  }
}

void Initializer::on_initial_pose(
  double stamp, const gtsam::Pose3 & pose, const gtsam::Matrix6 & covariance)
{
  (void)stamp;
  initial_pose_ = pose;
  initial_pose_cov_ = covariance;
}

void Initializer::on_ndt(
  double stamp, const gtsam::Pose3 & pose, const gtsam::Matrix6 & covariance)
{
  (void)stamp;
  ndt_pose_ = pose;
  ndt_pose_cov_ = covariance;
}

void Initializer::on_gnss(
  double stamp, const gtsam::Point3 & position, const gtsam::Matrix3 & covariance,
  std::optional<double> yaw, double yaw_var)
{
  GnssSample sample;
  sample.stamp = stamp;
  sample.position = position;
  sample.covariance = covariance;
  sample.yaw = yaw;
  sample.yaw_var = yaw_var;
  gnss_samples_.push_back(sample);
  if (gnss_samples_.size() > 200) {
    gnss_samples_.erase(gnss_samples_.begin());
  }
}

bool Initializer::is_static(double now) const
{
  if (static_since_ < 0.0) {
    return false;
  }
  return (now - static_since_) >= params_.initialization.bias_init_static_s;
}

std::optional<double> Initializer::resolve_gnss_yaw(double * yaw_stddev)
{
  const auto & init = params_.initialization;
  const std::string & source = init.gnss_yaw_source;
  const bool automatic = source == "auto";

  // 1. dual antenna heading
  if ((automatic || source == "dual_antenna") && !gnss_samples_.empty()) {
    const auto & latest = gnss_samples_.back();
    if (latest.yaw.has_value() &&
        std::sqrt(std::max(latest.yaw_var, 0.0)) < init.gnss_yaw_max_stddev_rad) {
      gnss_yaw_source_used_ = "dual_antenna";
      *yaw_stddev = std::sqrt(latest.yaw_var * init.pose_cov_scale);
      return latest.yaw;
    }
    if (!automatic) {
      return std::nullopt;
    }
  }

  // 2. course over ground
  if ((automatic || source == "course") && moving_since_ >= 0.0 && gnss_samples_.size() >= 2) {
    const double moving_for = latest_wheel_stamp_ - moving_since_;
    if (moving_for >= init.gnss_yaw_min_duration_s && latest_vx_ > 0.0) {
      const auto & last = gnss_samples_.back();
      // Oldest sample that is still inside the motion interval.
      const GnssSample * first = &gnss_samples_.front();
      for (const auto & sample : gnss_samples_) {
        if (sample.stamp >= moving_since_) {
          first = &sample;
          break;
        }
      }
      const gtsam::Vector3 delta = last.position - first->position;
      const double distance = delta.head<2>().norm();
      if (distance >= kCourseMinDistance) {
        gnss_yaw_source_used_ = "course";
        const double position_stddev = std::sqrt(std::max(last.covariance(0, 0), 1e-6));
        *yaw_stddev = std::max(position_stddev * std::sqrt(2.0) / distance, 0.05);
        return std::atan2(delta.y(), delta.x());
      }
    }
    if (!automatic) {
      return std::nullopt;
    }
  }

  // 3. yaw of an explicitly given initialpose
  if ((automatic || source == "initialpose") && initial_pose_.has_value()) {
    gnss_yaw_source_used_ = "initialpose";
    *yaw_stddev = std::sqrt(initial_pose_cov_(2, 2) * init.pose_cov_scale);
    return initial_pose_->rotation().yaw();
  }

  // 4. test-only escape hatch, never reached by `auto`
  if (source == "zero_with_large_covariance") {
    gnss_yaw_source_used_ = "zero_with_large_covariance";
    *yaw_stddev = init.gnss_yaw_fallback_stddev_rad;
    return 0.0;
  }

  return std::nullopt;
}

std::optional<Anchor> Initializer::try_initialize(double now)
{
  const auto & init = params_.initialization;
  if (first_seen_ < 0.0) {
    first_seen_ = now;
  }

  if (imu_count_ < static_cast<size_t>(init.required_imu_samples)) {
    message_ = "waiting for IMU samples (" + std::to_string(imu_count_) + "/" +
               std::to_string(init.required_imu_samples) + ")";
    return std::nullopt;
  }

  // --- X0 -------------------------------------------------------------------
  gtsam::Pose3 pose0;
  gtsam::Matrix6 pose_cov = gtsam::Matrix6::Identity();
  double yaw_stddev_override = -1.0;

  if (init.pose_source == "initialpose") {
    if (!initial_pose_.has_value()) {
      message_ = "waiting for /initialpose3d";
      return std::nullopt;
    }
    pose0 = *initial_pose_;
    pose_cov = initial_pose_cov_ * init.pose_cov_scale;
  } else if (init.pose_source == "ndt") {
    if (!ndt_pose_.has_value()) {
      message_ = "waiting for the pose_estimator output";
      return std::nullopt;
    }
    pose0 = *ndt_pose_;
    pose_cov = ndt_pose_cov_ * init.pose_cov_scale;
  } else {  // gnss
    if (gnss_samples_.empty()) {
      message_ = "waiting for GNSS";
      return std::nullopt;
    }
    const auto yaw = resolve_gnss_yaw(&yaw_stddev_override);
    if (!yaw.has_value()) {
      // D41: refuse rather than start with a plausible but wrong heading.
      state_ = State::kUninitialized;
      message_ = "gnss init: yaw source unavailable";
      if ((now - first_seen_) > init.gnss_yaw_timeout_s) {
        message_ += " (gnss_yaw_timeout_s exceeded)";
      }
      return std::nullopt;
    }
    const auto & latest = gnss_samples_.back();
    pose0 = gtsam::Pose3(gtsam::Rot3::Yaw(*yaw), latest.position);
    pose_cov = gtsam::Matrix6::Zero();
    const double rot_var = std::pow(init.init_pose_rot_stddev, 2);
    pose_cov(0, 0) = rot_var;
    pose_cov(1, 1) = rot_var;
    pose_cov(2, 2) = std::pow(yaw_stddev_override, 2);
    pose_cov.block<3, 3>(3, 3) = latest.covariance * init.pose_cov_scale;
  }

  // --- bias / gravity alignment (design §5.3.8) -----------------------------
  state_ = State::kBiasInit;
  if (bias_init_started_ < 0.0) {
    bias_init_started_ = now;
  }
  gtsam::Vector3 accel_bias(
    params_.imu.initial_bias[0], params_.imu.initial_bias[1], params_.imu.initial_bias[2]);
  gtsam::Vector3 gyro_bias(
    params_.imu.initial_bias[3], params_.imu.initial_bias[4], params_.imu.initial_bias[5]);
  double bias_stddev_scale = 1.0;

  const bool timed_out = (now - bias_init_started_) > init.bias_init_timeout_s;
  if (is_static(now) && !static_window_.empty()) {
    gtsam::Vector3 accel_mean = gtsam::Vector3::Zero();
    gtsam::Vector3 gyro_mean = gtsam::Vector3::Zero();
    for (const auto & sample : static_window_) {
      accel_mean += sample.accel;
      gyro_mean += sample.gyro;
    }
    accel_mean /= static_cast<double>(static_window_.size());
    gyro_mean /= static_cast<double>(static_window_.size());

    // Earth rotation (15 deg/h) is far below the noise floor of an automotive IMU.
    gyro_bias = gyro_mean;

    if (init.pose_source == "gnss") {
      // roll / pitch and b_a cannot be separated from one static IMU; with GNSS the
      // attitude is the unknown, so b_a is assumed and the attitude is derived.
      const double roll = std::atan2(accel_mean.y(), accel_mean.z());
      const double pitch =
        std::atan2(-accel_mean.x(), std::hypot(accel_mean.y(), accel_mean.z()));
      pose0 = gtsam::Pose3(
        gtsam::Rot3::RzRyRx(roll, pitch, pose0.rotation().yaw()), pose0.translation());
    } else {
      // Attitude is given, so it is trusted and the accelerometer bias follows from
      //   f = R_wb^T (0,0,g) + b_a   (GTSAM MakeSharedU specific force convention).
      const gtsam::Vector3 gravity_body =
        pose0.rotation().matrix().transpose() * gtsam::Vector3(0.0, 0.0, params_.imu.gravity);
      accel_bias = accel_mean - gravity_body;
    }
  } else if (!timed_out) {
    message_ = "collecting a static IMU window for the bias initialization";
    return std::nullopt;
  } else {
    // Moving at start-up: keep the configured bias but double its uncertainty.
    bias_stddev_scale = 2.0;
  }

  Anchor anchor;
  anchor.stamp = std::max(latest_imu_stamp_, now);
  gtsam::Vector3 velocity = gtsam::Vector3::Zero();
  if (has_wheel_) {
    velocity = pose0.rotation() * gtsam::Vector3(latest_vx_, 0.0, 0.0);
  }
  anchor.state = gtsam::NavState(pose0, velocity);
  anchor.bias = gtsam::imuBias::ConstantBias(accel_bias, gyro_bias);

  anchor.covariance = Matrix15::Zero();
  anchor.covariance.block<6, 6>(kPoseIndex, kPoseIndex) = pose_cov;
  anchor.covariance.block<3, 3>(kVelIndex, kVelIndex) =
    gtsam::Matrix3::Identity() * std::pow(init.init_vel_stddev, 2);
  anchor.covariance.block<3, 3>(kBiasIndex, kBiasIndex) =
    gtsam::Matrix3::Identity() * std::pow(init.init_accel_bias_stddev * bias_stddev_scale, 2);
  anchor.covariance.block<3, 3>(kBiasIndex + 3, kBiasIndex + 3) =
    gtsam::Matrix3::Identity() * std::pow(init.init_gyro_bias_stddev * bias_stddev_scale, 2);
  anchor.valid = true;

  state_ = State::kRunning;
  message_ = "OK";
  return anchor;
}

}  // namespace autoware::pose_twist_fusion::fg
