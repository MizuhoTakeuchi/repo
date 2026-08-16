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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__FG_TYPES_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__FG_TYPES_HPP_

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include <cstdint>

namespace autoware::pose_twist_fusion::fg
{

using Matrix15 = Eigen::Matrix<double, 15, 15>;
using Vector15 = Eigen::Matrix<double, 15, 1>;

/// Index layout of the joint state covariance:
///   [ xi_X (6: omega, nu) ; delta_v (3, world) ; delta_b (6: accel, gyro) ]
/// which is exactly [NavState tangent (9) ; bias (6)] in GTSAM's ordering.
constexpr int kPoseIndex = 0;
constexpr int kVelIndex = 6;
constexpr int kBiasIndex = 9;

struct ImuSample
{
  double stamp{0.0};
  gtsam::Vector3 accel{gtsam::Vector3::Zero()};  // base_link, specific force
  gtsam::Vector3 gyro{gtsam::Vector3::Zero()};   // base_link
};

struct TwistSample
{
  double stamp{0.0};
  double vx{0.0};
  double variance{0.04};
  /// Set by the layer A chi-squared gate inside Graph1 (D42). A rejected sample is
  /// used neither by Graph1 nor by Graph2, and replay reproduces the same decision.
  bool rejected{false};
  bool gate_evaluated{false};
  double d2{0.0};
};

/// One 20 ms grid point of Graph1 (design §5.3.4).
struct StateSample
{
  double stamp{0.0};
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity{gtsam::Vector3::Zero()};
  gtsam::Matrix6 pose_cov{gtsam::Matrix6::Identity()};
  gtsam::Matrix3 vel_cov{gtsam::Matrix3::Identity()};
};

/// Anchor handed from Graph2 to Graph1 (design §5.3.7 (1)/(4)).
struct Anchor
{
  double stamp{0.0};
  gtsam::NavState state;
  gtsam::imuBias::ConstantBias bias;
  Matrix15 covariance{Matrix15::Identity()};
  bool valid{false};
};

enum class MeasurementSource : std::uint8_t { kGnss, kNdt, kSlamAbsolute, kSlamRelative };

/// Whether a stored measurement has ever been turned into a factor (D34 / INV-5).
enum class Contribution : std::uint8_t {
  kNotYetBuilt,           //< never went through graph construction
  kFactorAdded,           //< added as a factor in the most recent tick
  kIntentionallyExcluded  //< SLAM jump detection / SLAM interpolation anchor
};

struct MeasurementValue
{
  /// GNSS: antenna position in map. NDT / SLAM absolute: full pose.
  gtsam::Point3 position{gtsam::Point3(0, 0, 0)};
  gtsam::Pose3 pose;
  /// SLAM relative: frozen relative transform of the consecutive sample pair.
  gtsam::Pose3 relative;
  /// Stamp of the pair's first sample (SLAM relative only).
  double previous_stamp{0.0};
};

/// An accepted measurement kept until it is absorbed by the marginal prior (D28).
struct AcceptedMeasurement
{
  double stamp{0.0};
  MeasurementSource source{MeasurementSource::kNdt};
  MeasurementValue value;
  /// Corrected at arrival and never recomputed (INV-3).
  gtsam::SharedNoiseModel noise;
  /// The same covariance in matrix form (needed to split a SLAM pair at the
  /// marginalization boundary and to report it in diagnostics). Frozen as well.
  Eigen::MatrixXd corrected_covariance;
  double nis{0.0};
  double weight{1.0};
  /// Index of the node this measurement was attached to in the most recent tick.
  /// Pruning is decided on this index, never on the stamp (D29).
  std::uint64_t assigned_node{0};
  bool has_assigned_node{false};
  /// node time - measurement stamp of the most recent assignment. Transient (it is
  /// re-derived every tick) and used only to inflate the factor noise (D34);
  /// the frozen `noise` itself is never modified (INV-3).
  double node_time_offset{0.0};
  Contribution contribution{Contribution::kNotYetBuilt};
  /// SLAM relative only: this sample is kept as the start point of a future pair.
  bool is_interpolation_anchor{false};
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__FG_TYPES_HPP_
