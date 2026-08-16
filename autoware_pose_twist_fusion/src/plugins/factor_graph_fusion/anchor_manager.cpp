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

#include "plugins/factor_graph_fusion/anchor_manager.hpp"

#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pose_twist_fusion::fg
{

AnchorManager::AnchorManager(
  const FactorGraphParams & params, Graph1Propagator & graph1, ImuBuffer & imu_buffer,
  VehicleTwistBuffer & twist_buffer, PendingMeasurementQueue & pending,
  const gtsam::Point3 & gnss_lever_arm)
: params_(params),
  graph1_(graph1),
  imu_buffer_(imu_buffer),
  twist_buffer_(twist_buffer),
  pending_(pending),
  window_builder_(params),
  factor_builders_(params, gnss_lever_arm),
  optimizer_(params),
  marginalizer_(params),
  measurement_history_(static_cast<size_t>(params.graph2.measurement_history_max))
{
}

AnchorManager::~AnchorManager()
{
  stop();
}

void AnchorManager::start()
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (running_) {
      return;
    }
    running_ = true;
  }
  // D17: the optimization never runs inside a ROS callback.
  worker_ = std::thread(&AnchorManager::worker_loop, this);
}

void AnchorManager::stop()
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (!running_) {
      return;
    }
    running_ = false;
  }
  worker_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void AnchorManager::request_tick()
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (tick_requested_) {
      // Still busy with the previous request: overwrite instead of queueing so that
      // a slow optimization cannot pile up work (design §4.3).
      std::lock_guard<std::mutex> stats_lock(stats_mutex_);
      ++stats_.skipped_ticks;
    }
    tick_requested_ = true;
  }
  worker_cv_.notify_one();
}

void AnchorManager::worker_loop()
{
  while (true) {
    {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      worker_cv_.wait(lock, [this] { return tick_requested_ || !running_; });
      if (!running_) {
        return;
      }
      tick_requested_ = false;
    }
    run_tick();
  }
}

void AnchorManager::set_initial_anchor(const Anchor & anchor)
{
  graph1_.reset_to_anchor(anchor);
  last_anchor_ = anchor;
  t_prior_ = anchor.stamp;
  begin_index_ = 0;
  next_free_index_ = 1;
  boundary_prior_ = gtsam::NonlinearFactorGraph();
  previous_solution_ = gtsam::Values();
  previous_node_stamps_.clear();
  measurement_history_.clear();
  consecutive_failures_ = 0;
  total_failures_since_reset_ = 0;
  window_begin_.store(t_prior_);
  previous_snapshot_.reset();
  generation_.fetch_add(1);

  // The initial anchor is a prior on the first node; without it the very first
  // window would be unconstrained in the absolute frame.
  const gtsam::Key xk = pose_key(begin_index_);
  const gtsam::Key vk = velocity_key(begin_index_);
  const gtsam::Key bk = bias_key(begin_index_);
  boundary_prior_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    xk, anchor.state.pose(),
    gtsam::noiseModel::Gaussian::Covariance(anchor.covariance.block<6, 6>(kPoseIndex, kPoseIndex)));
  boundary_prior_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
    vk, anchor.state.v(),
    gtsam::noiseModel::Gaussian::Covariance(anchor.covariance.block<3, 3>(kVelIndex, kVelIndex)));
  boundary_prior_.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
    bk, anchor.bias,
    gtsam::noiseModel::Gaussian::Covariance(anchor.covariance.block<6, 6>(kBiasIndex, kBiasIndex)));
}

void AnchorManager::reset()
{
  t_prior_ = -1.0;
  begin_index_ = 0;
  next_free_index_ = 1;
  boundary_prior_ = gtsam::NonlinearFactorGraph();
  previous_solution_ = gtsam::Values();
  previous_node_stamps_.clear();
  measurement_history_.clear();
  pending_.clear();
  consecutive_failures_ = 0;
  total_failures_since_reset_ = 0;
  last_anchor_ = Anchor{};
  window_begin_.store(0.0);
  previous_snapshot_.reset();
  needs_reinitialization_.store(false);
}

std::shared_ptr<const Graph1Propagator::Snapshot> AnchorManager::previous_snapshot() const
{
  return std::atomic_load(&previous_snapshot_);
}

AnchorManager::Stats AnchorManager::stats() const
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

void AnchorManager::handle_failure(const std::string & error, double t_anchor)
{
  ++consecutive_failures_;
  ++total_failures_since_reset_;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.optimizer_failures;
    stats_.consecutive_failures = static_cast<std::uint64_t>(consecutive_failures_);
    stats_.last_error = error;
  }

  // §5.3.9: on failure nothing is half-updated. The prior, t_prior_ and the history
  // all stay as they were, so the window simply grows to the right.
  if (consecutive_failures_ >= params_.failure.optimizer_failure_limit) {
    // Forced rebuild from the last anchor with an inflated covariance.
    size_t dropped_not_built = 0;
    measurement_history_.drop_up_to(t_anchor, &dropped_not_built);

    boundary_prior_ = gtsam::NonlinearFactorGraph();
    begin_index_ = next_free_index_++;
    const double inflation = params_.failure.failure_cov_inflation;
    const gtsam::Key xk = pose_key(begin_index_);
    const gtsam::Key vk = velocity_key(begin_index_);
    const gtsam::Key bk = bias_key(begin_index_);
    boundary_prior_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      xk, last_anchor_.state.pose(),
      gtsam::noiseModel::Gaussian::Covariance(
        inflation * last_anchor_.covariance.block<6, 6>(kPoseIndex, kPoseIndex)));
    boundary_prior_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
      vk, last_anchor_.state.v(),
      gtsam::noiseModel::Gaussian::Covariance(
        inflation * last_anchor_.covariance.block<3, 3>(kVelIndex, kVelIndex)));
    boundary_prior_.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
      bk, last_anchor_.bias,
      gtsam::noiseModel::Gaussian::Covariance(
        inflation * last_anchor_.covariance.block<6, 6>(kBiasIndex, kBiasIndex)));

    t_prior_ = t_anchor;
    window_begin_.store(t_prior_);
    previous_solution_ = gtsam::Values();
    previous_node_stamps_.clear();
    consecutive_failures_ = 0;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ++stats_.forced_rebuilds;
      stats_.dropped_not_yet_built += dropped_not_built;
    }
  }

  if (total_failures_since_reset_ >= params_.failure.optimizer_failure_reinit_limit) {
    needs_reinitialization_.store(true);
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.reinit_requests;
  }
}

void AnchorManager::set_gnss_lever_arm(const gtsam::Point3 & lever_arm)
{
  std::lock_guard<std::mutex> lock(lever_arm_mutex_);
  pending_lever_arm_ = lever_arm;
  lever_arm_pending_ = true;
}

void AnchorManager::run_tick()
{
  {
    std::lock_guard<std::mutex> lock(lever_arm_mutex_);
    if (lever_arm_pending_) {
      factor_builders_.set_gnss_lever_arm(pending_lever_arm_);
      lever_arm_pending_ = false;
    }
  }

  if (!graph1_.has_anchor() || t_prior_ < 0.0) {
    return;
  }

  const Graph1Propagator::Snapshot g1 = graph1_.snapshot();
  const double t_end = g1.stamp;
  const double interval_s = params_.graph2_optimize_interval_ms * 1e-3;
  const double lag_s = params_.graph2_lag_ms * 1e-3;
  const double t_next_begin = t_end + interval_s - lag_s;

  if (!(t_prior_ < t_next_begin && t_next_begin < t_end)) {
    // The window has not grown enough yet (start-up, or a long optimizer outage
    // that pushed t_prior_ forward). Skipping keeps the boundary well defined.
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.skipped_ticks;
    return;
  }

  // (a) collect measurements ------------------------------------------------
  for (const auto & measurement : pending_.drain()) {
    measurement_history_.insert(measurement);
  }
  size_t dropped_not_built = 0;
  const size_t dropped = measurement_history_.drop_before(t_prior_, &dropped_not_built);

  std::vector<AcceptedMeasurement *> in_window = measurement_history_.in_window(t_prior_, t_end);

  // (b) node timeline --------------------------------------------------------
  const Graph2Window::BuildResult window = window_builder_.build(
    t_prior_, t_next_begin, t_end, previous_node_stamps_, in_window, begin_index_,
    next_free_index_);
  if (!window.valid) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.skipped_ticks;
    return;
  }

  const auto wheel_samples = twist_buffer_.range(t_prior_, t_end);
  const auto wheel_bindings = window_builder_.bind_wheel_speed(window, wheel_samples);

  // (c)(d) build the graph ---------------------------------------------------
  const StateHistory state_history = graph1_.state_history_copy();
  FactorBuilders::Input build_input;
  build_input.window = &window;
  build_input.measurements = in_window;
  build_input.wheel_bindings = wheel_bindings;
  build_input.imu_buffer = &imu_buffer_;
  build_input.state_history = &state_history;
  build_input.boundary_prior = boundary_prior_;
  build_input.previous_solution = previous_solution_;
  build_input.bias_estimate = g1.bias;
  build_input.begin_state = g1.state;
  build_input.has_begin_state = true;

  const FactorBuilders::Output built = factor_builders_.build(build_input);
  if (!built.valid) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.skipped_ticks;
    return;
  }

  // (f) optimize -------------------------------------------------------------
  const Graph2Optimizer::Result optimization =
    optimizer_.optimize(built.graph, built.initial, window.end_index, t_end);
  if (!optimization.success) {
    handle_failure(optimization.error, last_anchor_.stamp);
    return;
  }

  // (2) divergence check -----------------------------------------------------
  const auto & anchor = optimization.anchor;
  if (
    anchor.state.v().norm() > params_.failure.max_velocity_mps ||
    anchor.bias.accelerometer().norm() > params_.failure.max_accel_bias ||
    anchor.bias.gyroscope().norm() > params_.failure.max_gyro_bias) {
    handle_failure("divergence detected in the optimized anchor", last_anchor_.stamp);
    return;
  }

  consecutive_failures_ = 0;

  // (3) marginalization + history prune --------------------------------------
  const Graph2Marginalizer::Result marginalization =
    marginalizer_.marginalize(built.graph, optimization.values, window.next_begin_index);
  Graph2MeasurementHistory::PruneReport prune_report;
  if (marginalization.success) {
    boundary_prior_ = marginalization.prior;
    t_prior_ = window.nodes.empty() ? t_prior_ : t_next_begin;
    begin_index_ = window.next_begin_index;
    window_begin_.store(t_prior_);
    prune_report = measurement_history_.prune_absorbed(window.next_begin_index);
  }

  next_free_index_ = std::max(window.next_free_index, begin_index_ + 1);
  previous_solution_ = optimization.values;
  previous_node_stamps_.clear();
  for (const auto & node : window.nodes) {
    if (node.stamp >= t_prior_) {
      previous_node_stamps_.push_back(node.stamp);
    }
  }

  // (4)(5) Graph1 reset and replay ------------------------------------------
  // The pre-reset Graph1 state is what OutputStage needs to keep the published pose
  // continuous across the anchor update (D37); it is captured *before* the reset.
  auto previous = std::make_shared<Graph1Propagator::Snapshot>(g1);
  std::atomic_store(&previous_snapshot_, std::shared_ptr<const Graph1Propagator::Snapshot>(previous));

  last_anchor_ = anchor;
  graph1_.reset_to_anchor(anchor);
  graph1_.replay_from(t_end, imu_buffer_, twist_buffer_);

  // (6) publish the new generation ------------------------------------------
  generation_.fetch_add(1);

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.ticks;
    stats_.consecutive_failures = 0;
    stats_.dropped_before_window += dropped;
    stats_.dropped_not_yet_built += dropped_not_built;
    stats_.inv5_blocked += prune_report.not_yet_built_blocked;
    stats_.straddling_factors += marginalization.straddling_factors;
    stats_.history_overflow = measurement_history_.overflow_count();
    stats_.history_size = measurement_history_.size();
    stats_.node_count = window.nodes.size();
    stats_.slam_pairs_split = built.slam_pairs_split;
    stats_.last_optimize_time_ms = optimization.optimize_time_ms;
    stats_.t_begin = t_prior_;
    stats_.t_end = t_end;
    if (!marginalization.success && !marginalization.error.empty()) {
      stats_.last_error = marginalization.error;
    }
  }
}

}  // namespace autoware::pose_twist_fusion::fg
