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

#include "plugins/lio_sam/lio_sam_covariance.hpp"

#include "autoware/pose_twist_fusion/covariance_convert.hpp"

#include <gtsam/base/numericalDerivative.h>

namespace autoware::pose_twist_fusion::lio_sam
{

PropagatedCovariance propagate_joint(
  const gtsam::PreintegratedImuMeasurements & pim, const gtsam::NavState & anchor_state,
  const gtsam::imuBias::ConstantBias & anchor_bias, const Matrix15 & anchor_covariance, double dt,
  double accel_bias_rw_stddev, double gyro_bias_rw_stddev)
{
  gtsam::Matrix99 h_state = gtsam::Matrix99::Identity();
  gtsam::Matrix96 h_bias = gtsam::Matrix96::Zero();

#if defined(PTF_LIO_SAM_PREDICT_JACOBIAN)
  pim.predict(anchor_state, anchor_bias, h_state, h_bias);
#else
  // Fallback for GTSAM builds whose predict() does not expose the Jacobians:
  // differentiate it numerically (design §5.2.1 explicitly allows this and requires
  // that test 9 passes either way).
  const auto predict_fn = [&pim](
                            const gtsam::NavState & state,
                            const gtsam::imuBias::ConstantBias & bias) {
    return pim.predict(state, bias);
  };
  h_state = gtsam::numericalDerivative21<gtsam::NavState, gtsam::NavState,
                                         gtsam::imuBias::ConstantBias>(
    predict_fn, anchor_state, anchor_bias);
  h_bias = gtsam::numericalDerivative22<gtsam::NavState, gtsam::NavState,
                                        gtsam::imuBias::ConstantBias>(
    predict_fn, anchor_state, anchor_bias);
#endif

  Eigen::Matrix<double, 9, 15> h;
  h.leftCols<9>() = h_state;
  h.rightCols<6>() = h_bias;

  PropagatedCovariance out;
  out.nav = h * anchor_covariance * h.transpose() + pim.preintMeasCov();
  out.nav = 0.5 * (out.nav + out.nav.transpose());

  out.pose = out.nav.topLeftCorner<6, 6>();
  out.velocity = out.nav.bottomRightCorner<3, 3>();

  gtsam::Matrix6 bias_random_walk = gtsam::Matrix6::Zero();
  bias_random_walk.topLeftCorner<3, 3>() =
    gtsam::Matrix3::Identity() * accel_bias_rw_stddev * accel_bias_rw_stddev * dt;
  bias_random_walk.bottomRightCorner<3, 3>() =
    gtsam::Matrix3::Identity() * gyro_bias_rw_stddev * gyro_bias_rw_stddev * dt;
  out.bias = anchor_covariance.bottomRightCorner<6, 6>() + bias_random_walk;

  return out;
}

}  // namespace autoware::pose_twist_fusion::lio_sam
