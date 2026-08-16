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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__SLAM_PAIR_GATE_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__SLAM_PAIR_GATE_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"
#include "plugins/factor_graph_fusion/covariance_corrector.hpp"

#include <gtsam/geometry/Pose3.h>

#include <cstdint>
#include <memory>

namespace autoware::pose_twist_fusion::fg
{

/// Gate for SLAM relative constraints, evaluated on **consecutive sample pairs**
/// and frozen when the pair forms (design §5.3.3c / D38).
///
/// Jump detection is (i) a chi-squared test on the dead-reckoning residual (main)
/// and (ii) a dynamic upper bound (backup). A fixed distance threshold is not used:
/// at 15 m/s and 100 ms the normal increment is already 1.5 m.
class SlamPairGate
{
public:
  struct PairResult
  {
    bool has_pair{false};
    bool accepted{false};
    double previous_stamp{0.0};
    gtsam::Pose3 relative;
    Eigen::MatrixXd covariance;  //< Sigma_Delta' (already weighted)
    double d2{0.0};
    double weight{1.0};
    bool rejected_chi2{false};
    bool rejected_dynamic{false};
  };

  SlamPairGate(const SlamParams & params, std::shared_ptr<CovarianceCorrector> corrector);

  /// @param stamp            current SLAM sample stamp
  /// @param odom_pose        current SLAM odometry pose
  /// @param odom_covariance  its covariance (GTSAM body tangent)
  /// @param delta_graph1     Graph1 relative motion over the same interval
  /// @param sigma_relative   interval preintegration covariance of that motion
  PairResult evaluate(
    double stamp, const gtsam::Pose3 & odom_pose, const gtsam::Matrix6 & odom_covariance,
    const gtsam::Pose3 & delta_graph1, const gtsam::Matrix6 & sigma_relative);

  void reset();

  /// Stamp of the sample that will start the next pair (0 when there is none).
  [[nodiscard]] double previous_stamp() const { return has_previous_ ? previous_stamp_ : 0.0; }
  [[nodiscard]] std::uint64_t chi2_reject_count() const { return chi2_reject_count_; }
  [[nodiscard]] std::uint64_t dynamic_reject_count() const { return dynamic_reject_count_; }

private:
  SlamParams params_;
  std::shared_ptr<CovarianceCorrector> corrector_;
  bool has_previous_{false};
  double previous_stamp_{0.0};
  gtsam::Pose3 previous_pose_;
  gtsam::Matrix6 previous_covariance_{gtsam::Matrix6::Identity()};
  std::uint64_t chi2_reject_count_{0};
  std::uint64_t dynamic_reject_count_{0};
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__SLAM_PAIR_GATE_HPP_
