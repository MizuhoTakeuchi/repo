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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__INITIALIZER_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__INITIALIZER_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"
#include "plugins/factor_graph_fusion/fg_types.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::pose_twist_fusion::fg
{

/// UNINITIALIZED -> BIAS_INIT -> RUNNING state machine of design §5.3.8
/// (D16 / D26 / D41).
///
/// Two rules are load bearing:
///  * b_a = mean(accel) - R_wb^T (0,0,g)   - subtracting gravity is what keeps the
///    sign convention consistent with GTSAM's MakeSharedU (a mistake here shows up
///    as a 19.6 m/s^2 error).
///  * with `pose_source: gnss` the yaw must come from an explicit source; guessing
///    yaw = 0 with a large variance is not a default, because a plausible but wrong
///    heading is more dangerous downstream than refusing to start (D41).
class Initializer
{
public:
  enum class State : std::uint8_t { kUninitialized, kBiasInit, kRunning };

  explicit Initializer(const FactorGraphParams & params);

  void on_imu(const ImuSample & sample);
  void on_wheel_speed(double stamp, double vx);
  void on_initial_pose(double stamp, const gtsam::Pose3 & pose, const gtsam::Matrix6 & covariance);
  void on_ndt(double stamp, const gtsam::Pose3 & pose, const gtsam::Matrix6 & covariance);
  /// @param yaw       heading from a dual antenna receiver, if available
  /// @param yaw_var   its variance (used to decide whether it can be trusted)
  void on_gnss(
    double stamp, const gtsam::Point3 & position, const gtsam::Matrix3 & covariance,
    std::optional<double> yaw, double yaw_var);

  /// Tries to produce the initial anchor. Returns nullopt while the state machine is
  /// still waiting; `message()` explains what for.
  std::optional<Anchor> try_initialize(double now);

  void reset();

  [[nodiscard]] State state() const { return state_; }
  [[nodiscard]] const std::string & message() const { return message_; }
  /// Which source produced the yaw when pose_source = gnss (diagnostics of 試験 24).
  [[nodiscard]] const std::string & gnss_yaw_source_used() const { return gnss_yaw_source_used_; }

private:
  struct GnssSample
  {
    double stamp{0.0};
    gtsam::Point3 position{gtsam::Point3(0, 0, 0)};
    gtsam::Matrix3 covariance{gtsam::Matrix3::Identity()};
    std::optional<double> yaw;
    double yaw_var{1.0};
  };

  [[nodiscard]] bool is_static(double now) const;
  [[nodiscard]] std::optional<double> resolve_gnss_yaw(double * yaw_stddev);

  FactorGraphParams params_;
  State state_{State::kUninitialized};
  std::string message_{"waiting for the initial pose"};
  std::string gnss_yaw_source_used_{"none"};

  std::vector<ImuSample> static_window_;
  double first_imu_stamp_{0.0};
  double latest_imu_stamp_{0.0};
  size_t imu_count_{0};
  double static_since_{-1.0};
  double bias_init_started_{-1.0};

  double latest_wheel_stamp_{0.0};
  double latest_vx_{0.0};
  bool has_wheel_{false};
  double moving_since_{-1.0};

  std::optional<gtsam::Pose3> initial_pose_;
  gtsam::Matrix6 initial_pose_cov_{gtsam::Matrix6::Identity()};
  std::optional<gtsam::Pose3> ndt_pose_;
  gtsam::Matrix6 ndt_pose_cov_{gtsam::Matrix6::Identity()};
  std::vector<GnssSample> gnss_samples_;
  double first_seen_{-1.0};
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__INITIALIZER_HPP_
