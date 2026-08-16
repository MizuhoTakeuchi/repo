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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__GRAPH1_PROPAGATOR_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__GRAPH1_PROPAGATOR_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"
#include "plugins/factor_graph_fusion/covariance_corrector.hpp"
#include "plugins/factor_graph_fusion/fg_types.hpp"
#include "plugins/factor_graph_fusion/imu_buffer.hpp"
#include "plugins/factor_graph_fusion/state_history.hpp"
#include "plugins/factor_graph_fusion/vehicle_twist_buffer.hpp"

#include <gtsam/navigation/ImuFactor.h>

#include <memory>
#include <mutex>
#include <optional>

namespace autoware::pose_twist_fusion::fg
{

/// "Factor graph 1" of requirement R6/R7, implemented as an analytic propagator
/// plus a one-step EKF update instead of a 50 Hz graph optimization (D9b).
///
/// The graph it stands for is a chain (IMU preintegration between consecutive nodes,
/// unary wheel speed / non-holonomic factors on each node), and for a chain the MAP
/// marginal of the newest node equals the forward recursion, so this is the same
/// quantity at a fraction of the cost (design §0.1 (1)).
///
/// Threading (D36): **every** public method takes `graph1_mutex_` on entry; the
/// internal `*_locked` variants avoid double locking. Lock order is
/// `graph1_mutex_` -> buffer mutexes, never the other way around.
class Graph1Propagator
{
public:
  struct Snapshot
  {
    bool valid{false};
    double stamp{0.0};
    double anchor_stamp{0.0};
    gtsam::NavState state;
    gtsam::imuBias::ConstantBias bias;
    Matrix15 covariance{Matrix15::Identity()};
    gtsam::Vector3 last_gyro{gtsam::Vector3::Zero()};
    gtsam::Vector3 last_accel{gtsam::Vector3::Zero()};
    double last_wheel_speed{0.0};
  };

  Graph1Propagator(const FactorGraphParams & params, std::shared_ptr<CovarianceCorrector> corrector);

  /// Graph1 reset of requirement R8 / design §5.3.7 (4).
  void reset_to_anchor(const Anchor & anchor);

  /// The one and only IMU entry point; normal reception and replay share it (P2 / D15).
  void apply_imu(const ImuSample & sample);

  /// The one and only wheel speed entry point. Applies the layer A chi-squared gate
  /// (dof = 1) right before the EKF update and records the decision in `sample`
  /// (D42). Returns true when the update was applied.
  bool apply_wheel_speed(TwistSample & sample);

  /// Replays IMU and wheel speed after an anchor update, merged by stamp with IMU
  /// first on ties (design §5.3.7 (5)). Holds the lock for the whole replay and
  /// calls the same code path as normal reception.
  void replay_from(
    double t_end, const ImuBuffer & imu_buffer, VehicleTwistBuffer & twist_buffer);

  [[nodiscard]] Snapshot snapshot() const;

  /// Lock free snapshot for the output path (D17: the output timer must never take
  /// `graph1_mutex_`). It is refreshed by every state change through an atomic
  /// shared_ptr swap, so the reader always sees one consistent state.
  [[nodiscard]] std::shared_ptr<const Snapshot> latest_snapshot() const;

  /// mu_1(t), Sigma_1(t) for a measurement stamp (design §5.3.4).
  [[nodiscard]] std::optional<StateSample> interpolate(double t) const;

  /// Relative pose covariance of the interval (t0, t1], used by the SLAM pair gate
  /// (D38). This is the interval preintegration covariance, **not** the difference
  /// of two absolute covariances.
  [[nodiscard]] gtsam::Matrix6 propagate_interval(
    double t0, double t1, const ImuBuffer & imu_buffer) const;

  [[nodiscard]] StateHistory state_history_copy() const;
  [[nodiscard]] bool has_anchor() const;
  [[nodiscard]] std::uint64_t wheel_speed_reject_count() const;

private:
  void apply_imu_locked(const ImuSample & sample);
  bool apply_wheel_speed_locked(TwistSample & sample);
  void write_grid_point_locked();
  void publish_snapshot_locked();
  [[nodiscard]] StateSample current_sample_locked() const;

  FactorGraphParams params_;
  std::shared_ptr<CovarianceCorrector> corrector_;
  boost::shared_ptr<gtsam::PreintegrationParams> preint_params_;

  mutable std::mutex graph1_mutex_;
  bool has_anchor_{false};
  double anchor_stamp_{0.0};
  double stamp_{0.0};
  gtsam::NavState state_;
  gtsam::imuBias::ConstantBias bias_;
  Matrix15 covariance_{Matrix15::Identity()};
  gtsam::Vector3 last_gyro_{gtsam::Vector3::Zero()};
  gtsam::Vector3 last_accel_{gtsam::Vector3::Zero()};
  double last_wheel_speed_{0.0};
  double next_grid_stamp_{0.0};
  double last_wheel_interval_start_{-1.0};
  StateHistory state_history_;
  std::uint64_t wheel_speed_reject_count_{0};
  std::shared_ptr<const Snapshot> latest_snapshot_;
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__GRAPH1_PROPAGATOR_HPP_
