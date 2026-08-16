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
//
// Test 12 of design_last.md §11: ROS <-> GTSAM covariance conversion.

#include "autoware/pose_twist_fusion/covariance_convert.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

using autoware::pose_twist_fusion::covariance_convert::RosCovariance;
namespace cc = autoware::pose_twist_fusion::covariance_convert;

namespace
{
RosCovariance make_ros_covariance()
{
  // Distinct diagonal entries so that any mix-up of the ordering is visible.
  RosCovariance cov{};
  const double diag[6] = {0.01, 0.04, 0.09, 0.0001, 0.0004, 0.0009};
  for (int i = 0; i < 6; ++i) {
    cov[static_cast<size_t>(i * 6 + i)] = diag[i];
  }
  // one off-diagonal correlation between x and yaw
  cov[0 * 6 + 5] = 0.0005;
  cov[5 * 6 + 0] = 0.0005;
  return cov;
}
}  // namespace

TEST(CovarianceConvert, PermutationMapsRosOrderToGtsamOrder)
{
  const gtsam::Matrix6 pi = cc::permutation();
  gtsam::Vector6 ros_vector;
  ros_vector << 1, 2, 3, 4, 5, 6;  // [x y z rx ry rz]
  const gtsam::Vector6 gtsam_vector = pi * ros_vector;
  EXPECT_DOUBLE_EQ(gtsam_vector(0), 4);  // rx
  EXPECT_DOUBLE_EQ(gtsam_vector(1), 5);
  EXPECT_DOUBLE_EQ(gtsam_vector(2), 6);
  EXPECT_DOUBLE_EQ(gtsam_vector(3), 1);  // x
  EXPECT_DOUBLE_EQ(gtsam_vector(4), 2);
  EXPECT_DOUBLE_EQ(gtsam_vector(5), 3);
  // Pi is an involution, which is why Pi^T is used for the inverse direction.
  EXPECT_TRUE((pi * pi).isApprox(gtsam::Matrix6::Identity()));
}

TEST(CovarianceConvert, PoseRoundTripIsIdentity)
{
  const RosCovariance original = make_ros_covariance();
  const std::vector<gtsam::Rot3> rotations = {
    gtsam::Rot3(), gtsam::Rot3::Yaw(0.7), gtsam::Rot3::RzRyRx(0.1, -0.2, 1.3)};
  for (const auto & rotation : rotations) {
    const gtsam::Matrix6 in_gtsam = cc::ros_pose_to_gtsam(original, rotation);
    const RosCovariance back = cc::gtsam_pose_to_ros(in_gtsam, rotation);
    for (size_t i = 0; i < 36; ++i) {
      EXPECT_NEAR(back[i], original[i], 1e-12) << "element " << i;
    }
  }
}

TEST(CovarianceConvert, PoseConversionMatchesTheAnalyticSolution)
{
  // Pure translation covariance under a 90 degree yaw: the x and y variances swap.
  RosCovariance cov{};
  cov[0] = 4.0;   // var(x)
  cov[7] = 1.0;   // var(y)
  cov[14] = 0.25;  // var(z)
  const gtsam::Rot3 rotation = gtsam::Rot3::Yaw(M_PI / 2.0);
  const gtsam::Matrix6 in_gtsam = cc::ros_pose_to_gtsam(cov, rotation);
  // GTSAM order is [omega; nu], so the translation block is the bottom right one.
  EXPECT_NEAR(in_gtsam(3, 3), 1.0, 1e-12);
  EXPECT_NEAR(in_gtsam(4, 4), 4.0, 1e-12);
  EXPECT_NEAR(in_gtsam(5, 5), 0.25, 1e-12);
}

TEST(CovarianceConvert, TwistConversionOnlyReordersComponents)
{
  // GTSAM order [omega; nu] with distinct diagonal entries.
  gtsam::Matrix6 gtsam_twist = gtsam::Matrix6::Zero();
  for (int i = 0; i < 6; ++i) {
    gtsam_twist(i, i) = static_cast<double>(i + 1);
  }
  gtsam_twist(0, 3) = 0.5;  // correlation between omega_x and nu_x
  gtsam_twist(3, 0) = 0.5;

  const RosCovariance ros_twist = cc::gtsam_twist_to_ros(gtsam_twist);
  // ROS order is [nu; omega]: the diagonal must be rotated by three positions.
  EXPECT_DOUBLE_EQ(ros_twist[0 * 6 + 0], 4.0);
  EXPECT_DOUBLE_EQ(ros_twist[1 * 6 + 1], 5.0);
  EXPECT_DOUBLE_EQ(ros_twist[2 * 6 + 2], 6.0);
  EXPECT_DOUBLE_EQ(ros_twist[3 * 6 + 3], 1.0);
  EXPECT_DOUBLE_EQ(ros_twist[4 * 6 + 4], 2.0);
  EXPECT_DOUBLE_EQ(ros_twist[5 * 6 + 5], 3.0);
  // The correlation moves to (linear.x, angular.x).
  EXPECT_DOUBLE_EQ(ros_twist[0 * 6 + 3], 0.5);
  EXPECT_DOUBLE_EQ(ros_twist[3 * 6 + 0], 0.5);

  const gtsam::Matrix6 back = cc::ros_twist_to_gtsam(ros_twist);
  EXPECT_TRUE(back.isApprox(gtsam_twist, 1e-12));
}

TEST(CovarianceConvert, SanitizeNeverReturnsZeroOrNaN)
{
  gtsam::Matrix6 broken = gtsam::Matrix6::Zero();
  bool repaired = false;
  const gtsam::Matrix6 healthy = cc::sanitize(broken, 1.0e4, &repaired);
  EXPECT_TRUE(repaired);
  for (int i = 0; i < 6; ++i) {
    EXPECT_GT(healthy(i, i), 0.0);
  }

  broken.setConstant(std::numeric_limits<double>::quiet_NaN());
  const gtsam::Matrix6 from_nan = cc::sanitize(broken, 1.0e4, &repaired);
  EXPECT_TRUE(from_nan.allFinite());
  EXPECT_TRUE(repaired);

  gtsam::Matrix6 indefinite = gtsam::Matrix6::Identity();
  indefinite(0, 0) = -1.0;
  const gtsam::Matrix6 fixed = cc::sanitize(indefinite, 1.0e4, &repaired);
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> solver(fixed);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-12);
}

TEST(CovarianceConvert, TransportOperatorsAreIdentityForZeroOffset)
{
  const gtsam::Vector6 zero = gtsam::Vector6::Zero();
  EXPECT_TRUE(cc::right_jacobian_inverse(zero).isApprox(gtsam::Matrix6::Identity(), 1e-12));
  EXPECT_TRUE(cc::adjoint_of_inverse(zero).isApprox(gtsam::Matrix6::Identity(), 1e-12));
}

TEST(CovarianceConvert, RightJacobianInverseDiffersFromTheAdjoint)
{
  // D40: J_r^-1(-c) and Ad_{C^-1} are both I + O(|c|) but differ at first order.
  // Mixing them up is exactly the defect test 21 (11)/(12) guards against.
  gtsam::Vector6 c;
  c << 0.05, -0.02, 0.1, 1.0, 0.3, -0.4;
  const gtsam::Matrix6 j = cc::right_jacobian_inverse(-c);
  const gtsam::Matrix6 ad = cc::adjoint_of_inverse(c);
  EXPECT_FALSE(j.isApprox(ad, 1e-3));
  EXPECT_TRUE(j.isApprox(gtsam::Matrix6::Identity(), 1.0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
