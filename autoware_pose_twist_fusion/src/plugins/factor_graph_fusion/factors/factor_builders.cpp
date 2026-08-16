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

#include "plugins/factor_graph_fusion/factors/factor_builders.hpp"

#include "plugins/factor_graph_fusion/factors/gnss_lever_arm_factor.hpp"
#include "plugins/factor_graph_fusion/factors/nonholonomic_factor.hpp"
#include "plugins/factor_graph_fusion/factors/wheel_speed_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace autoware::pose_twist_fusion::fg
{

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

gtsam::Key pose_key(std::uint64_t index)
{
  return X(index);
}
gtsam::Key velocity_key(std::uint64_t index)
{
  return V(index);
}
gtsam::Key bias_key(std::uint64_t index)
{
  return B(index);
}

namespace
{
/// D34: shifting a measurement off the boundary node (and any nearest-node
/// assignment) introduces a time offset; convert it into extra noise with the
/// configured velocity / yaw-rate bounds.
Eigen::MatrixXd inflate_for_time_offset(
  const Eigen::MatrixXd & covariance, double offset, const Graph2Params & params, bool pose_6dof)
{
  const double dt = std::abs(offset);
  if (dt <= 0.0 || covariance.size() == 0) {
    return covariance;
  }
  Eigen::MatrixXd inflated = covariance;
  const double position_var = std::pow(params.boundary_merge_velocity_mps * dt, 2);
  const double rotation_var = std::pow(params.boundary_merge_yaw_rate_rps * dt, 2);
  if (pose_6dof) {
    // GTSAM tangent order [omega; nu].
    for (int i = 0; i < 3; ++i) {
      inflated(i, i) += rotation_var;
      inflated(i + 3, i + 3) += position_var;
    }
  } else {
    for (int i = 0; i < inflated.rows(); ++i) {
      inflated(i, i) += position_var;
    }
  }
  return inflated;
}

gtsam::SharedNoiseModel noise_from(const Eigen::MatrixXd & covariance)
{
  return gtsam::noiseModel::Gaussian::Covariance(covariance);
}
}  // namespace

FactorBuilders::FactorBuilders(const FactorGraphParams & params, gtsam::Point3 gnss_lever_arm)
: params_(params), gnss_lever_arm_(std::move(gnss_lever_arm))
{
  preint_params_ = gtsam::PreintegrationParams::MakeSharedU(params.imu.gravity);
  preint_params_->accelerometerCovariance =
    gtsam::Matrix33::Identity() * std::pow(params.imu.accel_noise_stddev, 2);
  preint_params_->gyroscopeCovariance =
    gtsam::Matrix33::Identity() * std::pow(params.imu.gyro_noise_stddev, 2);
  preint_params_->integrationCovariance = gtsam::Matrix33::Identity() * params.imu.integration_cov;
}

FactorBuilders::Output FactorBuilders::build(const Input & input) const
{
  Output out;
  if (input.window == nullptr || !input.window->valid || input.imu_buffer == nullptr) {
    return out;
  }
  const auto & nodes = input.window->nodes;

  out.graph.push_back(input.boundary_prior);

  // --- initial values -------------------------------------------------------
  for (const auto & node : nodes) {
    const gtsam::Key xk = pose_key(node.index);
    const gtsam::Key vk = velocity_key(node.index);
    const gtsam::Key bk = bias_key(node.index);

    gtsam::Pose3 pose;
    gtsam::Vector3 velocity = gtsam::Vector3::Zero();
    bool have_value = false;
    if (input.previous_solution.exists(xk) && input.previous_solution.exists(vk)) {
      pose = input.previous_solution.at<gtsam::Pose3>(xk);
      velocity = input.previous_solution.at<gtsam::Vector3>(vk);
      have_value = true;
    }
    if (!have_value && input.state_history != nullptr) {
      if (const auto sample = input.state_history->interpolate(node.stamp)) {
        pose = sample->pose;
        velocity = sample->velocity;
        have_value = true;
      }
    }
    if (!have_value && input.has_begin_state) {
      pose = input.begin_state.pose();
      velocity = input.begin_state.v();
      have_value = true;
    }
    if (!have_value) {
      return out;  // nothing to linearize around
    }
    out.initial.insert(xk, pose);
    out.initial.insert(vk, velocity);
    out.initial.insert(
      bk, input.previous_solution.exists(bk)
            ? input.previous_solution.at<gtsam::imuBias::ConstantBias>(bk)
            : input.bias_estimate);
  }

  // --- IMU preintegration per interval (design §5.3.2 (c)) ------------------
  for (size_t i = 0; i + 1 < nodes.size(); ++i) {
    const double t0 = nodes[i].stamp;
    const double t1 = nodes[i + 1].stamp;
    gtsam::PreintegratedImuMeasurements pim(preint_params_, input.bias_estimate);
    const auto samples = input.imu_buffer->range(t0, t1);
    double previous = t0;
    for (const auto & sample : samples) {
      const double dt = sample.stamp - previous;
      if (dt > 0.0) {
        pim.integrateMeasurement(sample.accel, sample.gyro, dt);
      }
      previous = sample.stamp;
    }
    if (previous < t1 && !samples.empty()) {
      pim.integrateMeasurement(samples.back().accel, samples.back().gyro, t1 - previous);
    }
    if (pim.deltaTij() <= 0.0) {
      continue;
    }
    out.graph.emplace_shared<gtsam::ImuFactor>(
      pose_key(nodes[i].index), velocity_key(nodes[i].index), pose_key(nodes[i + 1].index),
      velocity_key(nodes[i + 1].index), bias_key(nodes[i].index), pim);

    const gtsam::Vector6 bias_sigmas =
      (gtsam::Vector(6) << params_.imu.accel_bias_rw_stddev, params_.imu.accel_bias_rw_stddev,
       params_.imu.accel_bias_rw_stddev, params_.imu.gyro_bias_rw_stddev,
       params_.imu.gyro_bias_rw_stddev, params_.imu.gyro_bias_rw_stddev)
        .finished();
    out.graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
      bias_key(nodes[i].index), bias_key(nodes[i + 1].index), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Diagonal::Sigmas(std::sqrt(pim.deltaTij()) * bias_sigmas));
    ++out.imu_factors;
  }

  // --- measurement factors --------------------------------------------------
  // stamp -> node index, used to attach a SLAM pair to both of its endpoints.
  std::map<double, std::uint64_t> stamp_to_node;
  for (auto * measurement : input.measurements) {
    if (measurement->has_assigned_node) {
      stamp_to_node[measurement->stamp] = measurement->assigned_node;
    }
  }

  for (auto * measurement : input.measurements) {
    measurement->contribution = Contribution::kNotYetBuilt;
    if (!measurement->has_assigned_node) {
      continue;
    }
    const std::uint64_t node_index = measurement->assigned_node;
    if (node_index == input.window->begin_index) {
      // Should not happen (the boundary node is excluded during assignment); the
      // information is already in the prior, so adding it would double count.
      measurement->contribution = Contribution::kIntentionallyExcluded;
      continue;
    }

    switch (measurement->source) {
      case MeasurementSource::kGnss: {
        const Eigen::MatrixXd covariance = inflate_for_time_offset(
          measurement->corrected_covariance, measurement->node_time_offset, params_.graph2, false);
        out.graph.emplace_shared<GnssLeverArmFactor>(
          pose_key(node_index), measurement->value.position, gnss_lever_arm_,
          noise_from(covariance));
        measurement->contribution = Contribution::kFactorAdded;
        ++out.measurement_factors;
        break;
      }
      case MeasurementSource::kNdt:
      case MeasurementSource::kSlamAbsolute: {
        const Eigen::MatrixXd covariance = inflate_for_time_offset(
          measurement->corrected_covariance, measurement->node_time_offset, params_.graph2, true);
        out.graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          pose_key(node_index), measurement->value.pose, noise_from(covariance));
        measurement->contribution = Contribution::kFactorAdded;
        ++out.measurement_factors;
        break;
      }
      case MeasurementSource::kSlamRelative: {
        if (measurement->is_interpolation_anchor) {
          measurement->contribution = Contribution::kIntentionallyExcluded;
          break;
        }
        const auto previous = stamp_to_node.find(measurement->value.previous_stamp);
        if (previous == stamp_to_node.end() || previous->second == node_index) {
          // The pair start is outside the window, or both samples collapsed into one
          // node (SLAM faster than node_merge_tolerance_ms): cannot build a relative
          // factor, and saying so explicitly keeps INV-5 meaningful.
          measurement->contribution = Contribution::kIntentionallyExcluded;
          break;
        }
        const std::uint64_t start_index = previous->second;
        const Eigen::MatrixXd covariance = measurement->corrected_covariance;
        const std::uint64_t boundary = input.window->next_begin_index;

        if (start_index < boundary && node_index > boundary) {
          // INV-2: a pair may not straddle the next marginalization boundary. Split
          // it at j*, interpolating the relative transform and apportioning the
          // covariance by interval length so that the two halves carry the same
          // information as the original (design §5.3.3c).
          const double t_start = nodes[start_index].stamp;
          const double t_end_pair = nodes[node_index].stamp;
          const double t_boundary = nodes[boundary].stamp;
          const double span = t_end_pair - t_start;
          const double alpha = (span > 1e-9) ? (t_boundary - t_start) / span : 0.5;

          const gtsam::Pose3 first =
            gtsam::interpolate(gtsam::Pose3(), measurement->value.relative, alpha);
          const gtsam::Pose3 second = first.inverse() * measurement->value.relative;

          Eigen::MatrixXd cov_first = alpha * covariance;
          Eigen::MatrixXd cov_second = (1.0 - alpha) * covariance;
          const double interp_rot =
            std::pow(params_.slam.interp_sigma_rot_per_s * alpha * span, 2);
          const double interp_pos =
            std::pow(params_.slam.interp_sigma_pos_per_s * alpha * span, 2);
          for (int i = 0; i < 3; ++i) {
            cov_first(i, i) += interp_rot;
            cov_second(i, i) += interp_rot;
            cov_first(i + 3, i + 3) += interp_pos;
            cov_second(i + 3, i + 3) += interp_pos;
          }

          out.graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            pose_key(start_index), pose_key(boundary), first, noise_from(cov_first));
          out.graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            pose_key(boundary), pose_key(node_index), second, noise_from(cov_second));
          ++out.slam_pairs_split;
          out.measurement_factors += 2;
        } else {
          out.graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            pose_key(start_index), pose_key(node_index), measurement->value.relative,
            noise_from(covariance));
          ++out.measurement_factors;
        }
        measurement->contribution = Contribution::kFactorAdded;
        break;
      }
    }
  }

  // --- wheel speed ----------------------------------------------------------
  for (const auto & binding : input.wheel_bindings) {
    out.graph.emplace_shared<WheelSpeedFactor>(
      pose_key(binding.node_index), velocity_key(binding.node_index), binding.vx,
      gtsam::noiseModel::Isotropic::Sigma(1, std::sqrt(binding.variance)));
    ++out.wheel_factors;
    if (
      params_.wheel_speed.use_nonholonomic &&
      std::abs(binding.vx) >= params_.wheel_speed.nonholonomic_min_speed_mps) {
      out.graph.emplace_shared<NonHolonomicFactor>(
        pose_key(binding.node_index), velocity_key(binding.node_index),
        gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(2) << params_.wheel_speed.nonholonomic_lat_stddev,
           params_.wheel_speed.nonholonomic_vert_stddev)
            .finished()));
      ++out.wheel_factors;
    }
  }

  out.valid = true;
  return out;
}

}  // namespace autoware::pose_twist_fusion::fg
