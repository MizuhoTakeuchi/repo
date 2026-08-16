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

#include "plugins/factor_graph_fusion/graph2_optimizer.hpp"

#include "plugins/factor_graph_fusion/factors/factor_builders.hpp"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <chrono>
#include <exception>

namespace autoware::pose_twist_fusion::fg
{

Graph2Optimizer::Graph2Optimizer(const FactorGraphParams & params) : params_(params)
{
}

Graph2Optimizer::Result Graph2Optimizer::optimize(
  const gtsam::NonlinearFactorGraph & graph, const gtsam::Values & initial,
  std::uint64_t anchor_node_index, double anchor_stamp) const
{
  Result result;
  const auto start = std::chrono::steady_clock::now();

  try {
    if (params_.graph2.optimizer == "isam2") {
      // Even in isam2 mode the window is rebuilt every tick: incremental reuse is
      // incompatible with out-of-order measurements (D24 / §9.2).
      gtsam::ISAM2Params isam_params;
      isam_params.relinearizeThreshold = 0.1;
      isam_params.relinearizeSkip = 1;
      gtsam::ISAM2 isam(isam_params);
      isam.update(graph, initial);
      isam.update();
      result.values = isam.calculateEstimate();
    } else {
      gtsam::LevenbergMarquardtParams lm_params;
      lm_params.setVerbosityLM("SILENT");
      gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, lm_params);
      result.values = optimizer.optimize();
    }

    const gtsam::Key xk = pose_key(anchor_node_index);
    const gtsam::Key vk = velocity_key(anchor_node_index);
    const gtsam::Key bk = bias_key(anchor_node_index);

    result.anchor.stamp = anchor_stamp;
    result.anchor.state = gtsam::NavState(
      result.values.at<gtsam::Pose3>(xk), result.values.at<gtsam::Vector3>(vk));
    result.anchor.bias = result.values.at<gtsam::imuBias::ConstantBias>(bk);

    // D10: hand over the exact joint 15x15 covariance. Dropping the cross terms is
    // not a conservative approximation, so it is not the default anywhere.
    gtsam::Marginals marginals(graph, result.values);
    if (params_.reset.carry_cross_covariance) {
      gtsam::JointMarginal joint =
        marginals.jointMarginalCovariance(gtsam::KeyVector{xk, vk, bk});
      result.anchor.covariance.block<6, 6>(0, 0) = joint.at(xk, xk);
      result.anchor.covariance.block<6, 3>(0, 6) = joint.at(xk, vk);
      result.anchor.covariance.block<6, 6>(0, 9) = joint.at(xk, bk);
      result.anchor.covariance.block<3, 6>(6, 0) = joint.at(vk, xk);
      result.anchor.covariance.block<3, 3>(6, 6) = joint.at(vk, vk);
      result.anchor.covariance.block<3, 6>(6, 9) = joint.at(vk, bk);
      result.anchor.covariance.block<6, 6>(9, 0) = joint.at(bk, xk);
      result.anchor.covariance.block<6, 3>(9, 6) = joint.at(bk, vk);
      result.anchor.covariance.block<6, 6>(9, 9) = joint.at(bk, bk);
    } else {
      // Explicit fallback only: n * blkdiag(P) >= P holds for n = 3 blocks
      // (Cauchy-Schwarz), which is why the inflation is applied here.
      result.anchor.covariance.setZero();
      const double inflation = params_.reset.cross_covariance_inflation;
      result.anchor.covariance.block<6, 6>(0, 0) = inflation * marginals.marginalCovariance(xk);
      result.anchor.covariance.block<3, 3>(6, 6) = inflation * marginals.marginalCovariance(vk);
      result.anchor.covariance.block<6, 6>(9, 9) = inflation * marginals.marginalCovariance(bk);
    }
    result.anchor.covariance = 0.5 * (result.anchor.covariance + result.anchor.covariance.transpose());
    result.anchor.valid = true;
    result.success = true;
  } catch (const gtsam::IndeterminantLinearSystemException & e) {
    result.success = false;
    result.error = std::string("IndeterminantLinearSystemException: ") + e.what();
  } catch (const std::exception & e) {
    result.success = false;
    result.error = std::string("optimization failed: ") + e.what();
  }

  const auto end = std::chrono::steady_clock::now();
  result.optimize_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
  return result;
}

}  // namespace autoware::pose_twist_fusion::fg
