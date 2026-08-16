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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__ANCHOR_MANAGER_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__ANCHOR_MANAGER_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"
#include "autoware/pose_twist_fusion/types.hpp"
#include "plugins/factor_graph_fusion/factors/factor_builders.hpp"
#include "plugins/factor_graph_fusion/graph1_propagator.hpp"
#include "plugins/factor_graph_fusion/graph2_marginalizer.hpp"
#include "plugins/factor_graph_fusion/graph2_optimizer.hpp"
#include "plugins/factor_graph_fusion/graph2_window.hpp"
#include "plugins/factor_graph_fusion/measurement_history.hpp"
#include "plugins/factor_graph_fusion/measurement_queue.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace autoware::pose_twist_fusion::fg
{

/// Owns the Graph2 worker thread and the anchor hand-over to Graph1
/// (design §5.3.2 tick procedure and §5.3.7 (1)-(6)).
///
/// It deliberately does **not** own any smoothing state: `c` / `u` / `t_left` live
/// in OutputStage and are written only from the output callback group (D37 / D43).
class AnchorManager
{
public:
  struct Stats
  {
    std::uint64_t ticks{0};
    std::uint64_t skipped_ticks{0};
    std::uint64_t optimizer_failures{0};
    std::uint64_t consecutive_failures{0};
    std::uint64_t forced_rebuilds{0};
    std::uint64_t reinit_requests{0};
    std::uint64_t dropped_before_window{0};
    std::uint64_t dropped_not_yet_built{0};
    std::uint64_t inv5_blocked{0};
    std::uint64_t straddling_factors{0};
    std::uint64_t history_overflow{0};
    size_t history_size{0};
    size_t node_count{0};
    size_t slam_pairs_split{0};
    double last_optimize_time_ms{0.0};
    double t_begin{0.0};
    double t_end{0.0};
    std::string last_error;
  };

  AnchorManager(
    const FactorGraphParams & params, Graph1Propagator & graph1, ImuBuffer & imu_buffer,
    VehicleTwistBuffer & twist_buffer, PendingMeasurementQueue & pending,
    const gtsam::Point3 & gnss_lever_arm);
  ~AnchorManager();

  void start();
  void stop();

  /// Called by the 500 ms tick timer. Requests one optimization; a request that
  /// arrives while the worker is busy overwrites the pending one (never queues).
  void request_tick();

  /// Installs the very first anchor coming from the initializer.
  void set_initial_anchor(const Anchor & anchor);

  void reset();

  /// t_begin (= t_prior_), published for the callback side drop check (§5.3.4).
  /// A one tick stale read is harmless: it can only make a measurement be dropped by
  /// step (a-4) instead of by the callback.
  [[nodiscard]] double window_begin() const { return window_begin_.load(); }
  [[nodiscard]] autoware::pose_twist_fusion::EstimateGeneration generation() const
  {
    return generation_.load();
  }
  /// Graph1 state as it was just before the most recent anchor update; OutputStage
  /// needs it to move the jump into the smoothing offset (D37).
  [[nodiscard]] std::shared_ptr<const Graph1Propagator::Snapshot> previous_snapshot() const;
  [[nodiscard]] bool needs_reinitialization() const { return needs_reinitialization_.load(); }
  void clear_reinitialization_request() { needs_reinitialization_.store(false); }
  [[nodiscard]] Stats stats() const;
  [[nodiscard]] const Anchor & last_anchor() const { return last_anchor_; }

  /// Updates the GNSS lever arm once TF becomes available. The value is applied by
  /// the worker at the start of the next tick, so the factor builders are never
  /// modified while a tick is running.
  void set_gnss_lever_arm(const gtsam::Point3 & lever_arm);

  /// Runs one tick synchronously (used by the unit tests).
  void run_tick();

private:
  void worker_loop();
  void handle_failure(const std::string & error, double t_anchor);

  FactorGraphParams params_;
  Graph1Propagator & graph1_;
  ImuBuffer & imu_buffer_;
  VehicleTwistBuffer & twist_buffer_;
  PendingMeasurementQueue & pending_;

  Graph2Window window_builder_;
  FactorBuilders factor_builders_;
  Graph2Optimizer optimizer_;
  Graph2Marginalizer marginalizer_;
  Graph2MeasurementHistory measurement_history_;

  // --- worker owned state (no locking needed) ------------------------------
  double t_prior_{-1.0};
  std::uint64_t begin_index_{0};
  std::uint64_t next_free_index_{1};
  gtsam::NonlinearFactorGraph boundary_prior_;
  gtsam::Values previous_solution_;
  std::vector<double> previous_node_stamps_;
  Anchor last_anchor_;
  int consecutive_failures_{0};
  int total_failures_since_reset_{0};

  mutable std::mutex stats_mutex_;
  Stats stats_;

  std::mutex lever_arm_mutex_;
  gtsam::Point3 pending_lever_arm_;
  bool lever_arm_pending_{false};

  std::atomic<double> window_begin_{0.0};
  std::atomic<autoware::pose_twist_fusion::EstimateGeneration> generation_{0};
  std::atomic<bool> needs_reinitialization_{false};
  std::shared_ptr<const Graph1Propagator::Snapshot> previous_snapshot_;

  std::thread worker_;
  std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
  bool tick_requested_{false};
  bool running_{false};
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__ANCHOR_MANAGER_HPP_
