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

#include "plugins/factor_graph_fusion/graph1_propagator.hpp"

#include "autoware/pose_twist_fusion/covariance_convert.hpp"

#include <gtsam/base/Matrix.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace autoware::pose_twist_fusion::fg
{

namespace
{
/// Joseph form keeps the updated covariance symmetric and PSD even when the gain is
/// numerically imperfect.
Matrix15 joseph_update(
  const Matrix15 & covariance, const Eigen::MatrixXd & h, const Eigen::MatrixXd & k,
  const Eigen::MatrixXd & r)
{
  const Matrix15 i_kh = Matrix15::Identity() - k * h;
  Matrix15 updated = i_kh * covariance * i_kh.transpose() + k * r * k.transpose();
  return 0.5 * (updated + updated.transpose());
}
}  // namespace

Graph1Propagator::Graph1Propagator(
  const FactorGraphParams & params, std::shared_ptr<CovarianceCorrector> corrector)
: params_(params), corrector_(std::move(corrector))
{
  preint_params_ = gtsam::PreintegrationParams::MakeSharedU(params.imu.gravity);
  preint_params_->accelerometerCovariance =
    gtsam::Matrix33::Identity() * std::pow(params.imu.accel_noise_stddev, 2);
  preint_params_->gyroscopeCovariance =
    gtsam::Matrix33::Identity() * std::pow(params.imu.gyro_noise_stddev, 2);
  preint_params_->integrationCovariance = gtsam::Matrix33::Identity() * params.imu.integration_cov;
  state_history_ = StateHistory(params.timing.history_length_ms * 1e-3);
}

void Graph1Propagator::reset_to_anchor(const Anchor & anchor)
{
  std::lock_guard<std::mutex> lock(graph1_mutex_);
  has_anchor_ = anchor.valid;
  anchor_stamp_ = anchor.stamp;
  stamp_ = anchor.stamp;
  state_ = anchor.state;
  bias_ = anchor.bias;
  covariance_ = anchor.covariance;
  next_grid_stamp_ = anchor.stamp;
  last_wheel_interval_start_ = -1.0;
  // §5.3.4: only samples newer than the anchor are rebuilt; older ones stay so that
  // a delayed measurement arriving right after an anchor update can still be located.
  state_history_.erase_after(anchor.stamp);
  write_grid_point_locked();
  publish_snapshot_locked();
}

void Graph1Propagator::publish_snapshot_locked()
{
  auto snapshot = std::make_shared<Snapshot>();
  snapshot->valid = has_anchor_;
  snapshot->stamp = stamp_;
  snapshot->anchor_stamp = anchor_stamp_;
  snapshot->state = state_;
  snapshot->bias = bias_;
  snapshot->covariance = covariance_;
  snapshot->last_gyro = last_gyro_;
  snapshot->last_accel = last_accel_;
  snapshot->last_wheel_speed = last_wheel_speed_;
  std::atomic_store(&latest_snapshot_, std::shared_ptr<const Snapshot>(snapshot));
}

std::shared_ptr<const Graph1Propagator::Snapshot> Graph1Propagator::latest_snapshot() const
{
  return std::atomic_load(&latest_snapshot_);
}

StateSample Graph1Propagator::current_sample_locked() const
{
  StateSample sample;
  sample.stamp = stamp_;
  sample.pose = state_.pose();
  sample.velocity = state_.v();
  sample.pose_cov = covariance_.block<6, 6>(kPoseIndex, kPoseIndex);
  sample.vel_cov = covariance_.block<3, 3>(kVelIndex, kVelIndex);
  return sample;
}

void Graph1Propagator::write_grid_point_locked()
{
  if (stamp_ < next_grid_stamp_ - 1e-9) {
    return;
  }
  state_history_.push(current_sample_locked());
  next_grid_stamp_ = stamp_ + params_.graph1_node_interval_ms * 1e-3;
}

void Graph1Propagator::apply_imu_locked(const ImuSample & sample)
{
  if (!has_anchor_) {
    return;
  }
  const double dt = sample.stamp - stamp_;
  last_accel_ = sample.accel;
  last_gyro_ = sample.gyro;
  if (dt <= 0.0) {
    // Older than the current state (can happen right after a replay); ignore, the
    // information is already contained in the state.
    return;
  }

  // One-step analytic propagation. The Jacobians and the process noise both come
  // from GTSAM's preintegration, so Graph1 stays consistent with the ImuFactor that
  // Graph2 builds from the very same measurements.
  gtsam::PreintegratedImuMeasurements step(preint_params_, bias_);
  step.integrateMeasurement(sample.accel, sample.gyro, dt);

  gtsam::Matrix99 h_state = gtsam::Matrix99::Identity();
  gtsam::Matrix96 h_bias = gtsam::Matrix96::Zero();
  const gtsam::NavState predicted = step.predict(state_, bias_, h_state, h_bias);

  Matrix15 f = Matrix15::Identity();
  f.block<9, 9>(0, 0) = h_state;
  f.block<9, 6>(0, kBiasIndex) = h_bias;

  Matrix15 q = Matrix15::Zero();
  q.block<9, 9>(0, 0) = step.preintMeasCov();
  q.block<3, 3>(kBiasIndex, kBiasIndex) =
    gtsam::Matrix3::Identity() * std::pow(params_.imu.accel_bias_rw_stddev, 2) * dt;
  q.block<3, 3>(kBiasIndex + 3, kBiasIndex + 3) =
    gtsam::Matrix3::Identity() * std::pow(params_.imu.gyro_bias_rw_stddev, 2) * dt;

  covariance_ = f * covariance_ * f.transpose() + q;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  state_ = predicted;
  stamp_ = sample.stamp;

  write_grid_point_locked();
  publish_snapshot_locked();
}

bool Graph1Propagator::apply_wheel_speed_locked(TwistSample & sample)
{
  if (!has_anchor_ || !params_.wheel_speed.use) {
    return false;
  }

  // §5.3.2: at most one wheel speed sample per graph1 grid interval. Streaming has
  // to take the first sample of the interval (taking "the latest" would require
  // delaying the update by a full interval); at the assumed >= 50 Hz input the two
  // rules coincide, and both normal reception and replay use this one.
  const double interval_s = params_.graph1_node_interval_ms * 1e-3;
  const double interval_start = std::floor(sample.stamp / interval_s) * interval_s;
  if (last_wheel_interval_start_ >= 0.0 &&
      std::abs(interval_start - last_wheel_interval_start_) < 1e-9) {
    return false;
  }

  const gtsam::Matrix3 rotation = state_.pose().rotation().matrix();
  const gtsam::Vector3 body_velocity = rotation.transpose() * state_.v();

  // --- wheel speed: 1-D measurement -----------------------------------------
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(1, 15);
  h.block<1, 3>(0, kPoseIndex) = gtsam::skewSymmetric(body_velocity).row(0);
  h.block<1, 3>(0, kVelIndex) = rotation.transpose().row(0);

  Eigen::MatrixXd r(1, 1);
  r(0, 0) = sample.variance;
  const Eigen::MatrixXd s = h * covariance_ * h.transpose() + r;
  const double innovation = body_velocity.x() - sample.vx;
  const double d2 = (s(0, 0) > 0.0) ? innovation * innovation / s(0, 0)
                                    : std::numeric_limits<double>::infinity();

  sample.gate_evaluated = true;
  sample.d2 = d2;
  // Layer A only (D42): a 1-D scalar must not additionally be down-weighted by
  // layer B, which together with the non-holonomic constraint would attenuate twice.
  if (
    params_.covariance_correction.enable_gate &&
    d2 > corrector_->gate_threshold(1, false)) {
    sample.rejected = true;
    ++wheel_speed_reject_count_;
    last_wheel_interval_start_ = interval_start;
    return false;
  }
  sample.rejected = false;

  const Eigen::MatrixXd k = covariance_ * h.transpose() * s.inverse();
  const Vector15 delta = -k * Eigen::VectorXd::Constant(1, innovation);
  state_ = state_.retract(delta.head<9>());
  bias_ = bias_ + gtsam::imuBias::ConstantBias(delta.segment<3>(kBiasIndex), delta.segment<3>(kBiasIndex + 3));
  covariance_ = joseph_update(covariance_, h, k, r);

  // --- non-holonomic: 2-D measurement ---------------------------------------
  if (
    params_.wheel_speed.use_nonholonomic &&
    std::abs(sample.vx) >= params_.wheel_speed.nonholonomic_min_speed_mps) {
    const gtsam::Matrix3 rotation2 = state_.pose().rotation().matrix();
    const gtsam::Vector3 body_velocity2 = rotation2.transpose() * state_.v();

    Eigen::MatrixXd h2 = Eigen::MatrixXd::Zero(2, 15);
    h2.block<2, 3>(0, kPoseIndex) = gtsam::skewSymmetric(body_velocity2).bottomRows<2>();
    h2.block<2, 3>(0, kVelIndex) = rotation2.transpose().bottomRows<2>();

    Eigen::MatrixXd r2 = Eigen::MatrixXd::Zero(2, 2);
    r2(0, 0) = std::pow(params_.wheel_speed.nonholonomic_lat_stddev, 2);
    r2(1, 1) = std::pow(params_.wheel_speed.nonholonomic_vert_stddev, 2);

    const Eigen::MatrixXd s2 = h2 * covariance_ * h2.transpose() + r2;
    const Eigen::MatrixXd k2 = covariance_ * h2.transpose() * s2.inverse();
    const Eigen::Vector2d innovation2(body_velocity2.y(), body_velocity2.z());
    const Vector15 delta2 = -k2 * innovation2;
    state_ = state_.retract(delta2.head<9>());
    bias_ = bias_ + gtsam::imuBias::ConstantBias(
                      delta2.segment<3>(kBiasIndex), delta2.segment<3>(kBiasIndex + 3));
    covariance_ = joseph_update(covariance_, h2, k2, r2);
  }

  last_wheel_speed_ = sample.vx;
  last_wheel_interval_start_ = interval_start;
  publish_snapshot_locked();
  return true;
}

void Graph1Propagator::apply_imu(const ImuSample & sample)
{
  std::lock_guard<std::mutex> lock(graph1_mutex_);
  apply_imu_locked(sample);
}

bool Graph1Propagator::apply_wheel_speed(TwistSample & sample)
{
  std::lock_guard<std::mutex> lock(graph1_mutex_);
  return apply_wheel_speed_locked(sample);
}

void Graph1Propagator::replay_from(
  double t_end, const ImuBuffer & imu_buffer, VehicleTwistBuffer & twist_buffer)
{
  // Lock order: graph1_mutex_ -> buffer mutexes (D36).
  std::lock_guard<std::mutex> lock(graph1_mutex_);

  const std::vector<ImuSample> imu = imu_buffer.since(t_end);
  std::vector<TwistSample> twist = twist_buffer.since(t_end);

  size_t i = 0;
  size_t j = 0;
  while (i < imu.size() || j < twist.size()) {
    const bool take_imu =
      (j >= twist.size()) || (i < imu.size() && imu[i].stamp <= twist[j].stamp);
    if (take_imu) {
      apply_imu_locked(imu[i]);
      ++i;
    } else {
      // The gate decision is re-derived here, which is exactly what makes replay
      // reproduce the layer A rejections (D42 / 試験 18).
      apply_wheel_speed_locked(twist[j]);
      ++j;
    }
  }
  twist_buffer.apply_gate_results(twist);
}

Graph1Propagator::Snapshot Graph1Propagator::snapshot() const
{
  std::lock_guard<std::mutex> lock(graph1_mutex_);
  Snapshot out;
  out.valid = has_anchor_;
  out.stamp = stamp_;
  out.anchor_stamp = anchor_stamp_;
  out.state = state_;
  out.bias = bias_;
  out.covariance = covariance_;
  out.last_gyro = last_gyro_;
  out.last_accel = last_accel_;
  out.last_wheel_speed = last_wheel_speed_;
  return out;
}

std::optional<StateSample> Graph1Propagator::interpolate(double t) const
{
  std::lock_guard<std::mutex> lock(graph1_mutex_);
  return state_history_.interpolate(t);
}

gtsam::Matrix6 Graph1Propagator::propagate_interval(
  double t0, double t1, const ImuBuffer & imu_buffer) const
{
  std::lock_guard<std::mutex> lock(graph1_mutex_);
  gtsam::PreintegratedImuMeasurements interval(preint_params_, bias_);
  const std::vector<ImuSample> samples = imu_buffer.range(t0, t1);
  double previous = t0;
  for (const auto & sample : samples) {
    const double dt = sample.stamp - previous;
    if (dt > 0.0) {
      interval.integrateMeasurement(sample.accel, sample.gyro, dt);
    }
    previous = sample.stamp;
  }
  if (samples.empty() || previous < t1) {
    const double dt = t1 - previous;
    if (dt > 0.0) {
      interval.integrateMeasurement(last_accel_, last_gyro_, dt);
    }
  }
  gtsam::Matrix6 pose_block = interval.preintMeasCov().topLeftCorner<6, 6>();
  return 0.5 * (pose_block + pose_block.transpose());
}

StateHistory Graph1Propagator::state_history_copy() const
{
  std::lock_guard<std::mutex> lock(graph1_mutex_);
  return state_history_;
}

bool Graph1Propagator::has_anchor() const
{
  std::lock_guard<std::mutex> lock(graph1_mutex_);
  return has_anchor_;
}

std::uint64_t Graph1Propagator::wheel_speed_reject_count() const
{
  std::lock_guard<std::mutex> lock(graph1_mutex_);
  return wheel_speed_reject_count_;
}

}  // namespace autoware::pose_twist_fusion::fg
