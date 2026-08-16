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
// Test 21 of design_last.md §11 (unit part, Phase 0): anchor jump smoothing.
// Injection sizes 0.01 / 0.1 / 1 / 5 m and driving conditions (A) standing,
// (B) straight 10 m/s, (C) 15 m/s with 0.3 rad/s yaw rate.

#include "autoware/pose_twist_fusion/covariance_convert.hpp"
#include "autoware/pose_twist_fusion/output_stage.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <random>
#include <vector>

using autoware::pose_twist_fusion::InternalEstimate;
using autoware::pose_twist_fusion::OutputStage;
using autoware::pose_twist_fusion::PublishedEstimate;
namespace cc = autoware::pose_twist_fusion::covariance_convert;

namespace
{
constexpr double kDt = 0.02;  // 50 Hz

OutputStage::Config default_config()
{
  OutputStage::Config config;
  config.output_rate_hz = 50.0;
  config.twist_smoothing_consistency = true;
  config.exact_smoothing_adjoint = false;
  return config;
}

/// Motion model of the fake internal estimate.
struct Motion
{
  double speed{0.0};
  double yaw_rate{0.0};
};

/// Internal estimate of a vehicle driving with `motion` since t = 0, optionally
/// displaced by `offset` (which is what an anchor update looks like).
InternalEstimate make_internal(double t, const Motion & motion, const gtsam::Vector6 & offset)
{
  const double yaw = motion.yaw_rate * t;
  gtsam::Point3 position(0.0, 0.0, 0.0);
  if (std::abs(motion.yaw_rate) < 1e-9) {
    position = gtsam::Point3(motion.speed * t, 0.0, 0.0);
  } else {
    const double radius = motion.speed / motion.yaw_rate;
    position = gtsam::Point3(radius * std::sin(yaw), radius * (1.0 - std::cos(yaw)), 0.0);
  }

  InternalEstimate estimate;
  estimate.valid = true;
  estimate.stamp = t;
  estimate.pose = gtsam::Pose3(gtsam::Rot3::Yaw(yaw), position) * gtsam::Pose3::Expmap(offset);
  estimate.biased_pose = estimate.pose;
  estimate.velocity_world =
    estimate.pose.rotation() * gtsam::Vector3(motion.speed, 0.0, 0.0);
  estimate.angular_velocity_body = gtsam::Vector3(0.0, 0.0, motion.yaw_rate);
  estimate.pose_cov = gtsam::Matrix6::Identity() * 1e-4;
  estimate.twist_cov = gtsam::Matrix6::Identity() * 1e-4;
  return estimate;
}

gtsam::Vector6 translation_offset(double meters)
{
  gtsam::Vector6 offset = gtsam::Vector6::Zero();
  offset(3) = meters;
  return offset;
}
}  // namespace

TEST(OutputStage, SmallJumpIsAppliedImmediately)
{
  // (1) 0.01 m < max_anchor_jump_m (0.03) -> no smoothing at all.
  OutputStage stage(default_config());
  const Motion motion{0.0, 0.0};
  double t = 0.0;

  OutputStage::Event event;
  stage.step(t, make_internal(t, motion, gtsam::Vector6::Zero()), {}, 1, &event);
  t += kDt;

  const auto previous = [&](double at) -> std::optional<InternalEstimate> {
    return make_internal(at, motion, gtsam::Vector6::Zero());
  };
  const InternalEstimate jumped = make_internal(t, motion, translation_offset(0.01));
  const PublishedEstimate published = stage.step(t, jumped, previous, 2, &event);

  EXPECT_TRUE(event.anchor_update);
  EXPECT_FALSE(event.smoothing_triggered);
  EXPECT_FALSE(published.smoothing_active());
  EXPECT_NEAR(published.smoothing_offset_lin(), 0.0, 1e-12);
  // The published pose equals the internal one immediately.
  EXPECT_NEAR((published.pose().translation() - jumped.pose.translation()).norm(), 0.0, 1e-12);
}

TEST(OutputStage, LargeJumpsAlwaysGoThroughSmoothing)
{
  // (2) 0.1 / 1 / 5 m must never be applied immediately, and must be absorbed
  // within smoothing_max_duration_ms.
  for (const double jump : {0.1, 1.0, 5.0}) {
    OutputStage stage(default_config());
    const Motion motion{0.0, 0.0};
    double t = 0.0;
    stage.step(t, make_internal(t, motion, gtsam::Vector6::Zero()), {}, 1);

    const auto previous = [&](double at) -> std::optional<InternalEstimate> {
      return make_internal(at, motion, gtsam::Vector6::Zero());
    };
    t += kDt;
    OutputStage::Event event;
    PublishedEstimate published =
      stage.step(t, make_internal(t, motion, translation_offset(jump)), previous, 2, &event);

    EXPECT_TRUE(event.smoothing_triggered) << "jump " << jump;
    EXPECT_TRUE(published.smoothing_active()) << "jump " << jump;
    // The published pose barely moved even though the internal one jumped.
    EXPECT_LT(published.smoothing_offset_lin(), jump + 1e-9);
    EXPECT_GT(published.smoothing_offset_lin(), jump * 0.5);

    int steps = 0;
    while (published.smoothing_active() && steps < 200) {
      t += kDt;
      published = stage.step(t, make_internal(t, motion, translation_offset(jump)), previous, 2);
      ++steps;
    }
    EXPECT_FALSE(published.smoothing_active()) << "jump " << jump;
    EXPECT_LE(steps * kDt, 1.0 + 1e-9) << "jump " << jump;  // smoothing_max_duration_ms
    // (8) the residual at the end is below smoothing_done_m
    EXPECT_LE(published.smoothing_offset_lin(), 0.005 + 1e-9);
  }
}

TEST(OutputStage, PublishedPoseKeepsTheVehicleMotionWhileSmoothing)
{
  // (7) The regression this guards against: a published pose that moves only with
  // the correction rate and stops following the vehicle. Conditions (B) and (C).
  const std::vector<Motion> motions = {{10.0, 0.0}, {15.0, 0.3}};
  for (const auto & motion : motions) {
    OutputStage stage(default_config());
    double t = 0.0;
    stage.step(t, make_internal(t, motion, gtsam::Vector6::Zero()), {}, 1);
    const auto previous = [&](double at) -> std::optional<InternalEstimate> {
      return make_internal(at, motion, gtsam::Vector6::Zero());
    };

    t += kDt;
    PublishedEstimate published =
      stage.step(t, make_internal(t, motion, translation_offset(1.0)), previous, 2);
    gtsam::Point3 last_position = published.pose().translation();

    for (int i = 0; i < 30; ++i) {
      t += kDt;
      published = stage.step(t, make_internal(t, motion, translation_offset(1.0)), previous, 2);
      const double measured_speed =
        (published.pose().translation() - last_position).norm() / kDt;
      last_position = published.pose().translation();
      // vehicle speed +/- the correction rate limit (1 m/s)
      EXPECT_GT(measured_speed, motion.speed - 1.2);
      EXPECT_LT(measured_speed, motion.speed + 1.2);

      // (3) d(pose_published)/dt must agree with twist_published
      const double published_speed = published.linear_velocity_body().norm();
      EXPECT_NEAR(measured_speed, published_speed, 0.05);
    }
  }
}

TEST(OutputStage, CovarianceInflationHappensInTheGtsamTangentSpace)
{
  // (4) With c non-zero only in the rotation block, the ROS output must grow in
  // rx/ry/rz and not in x/y/z. Adding c c^T after the conversion would show up in
  // the wrong block.
  OutputStage stage(default_config());
  const Motion motion{0.0, 0.0};
  double t = 0.0;
  stage.step(t, make_internal(t, motion, gtsam::Vector6::Zero()), {}, 1);

  gtsam::Vector6 rotation_only = gtsam::Vector6::Zero();
  rotation_only(2) = 0.2;  // yaw only
  const auto previous = [&](double at) -> std::optional<InternalEstimate> {
    return make_internal(at, motion, gtsam::Vector6::Zero());
  };
  t += kDt;
  const PublishedEstimate published =
    stage.step(t, make_internal(t, motion, rotation_only), previous, 2);
  ASSERT_TRUE(published.smoothing_active());

  const InternalEstimate reference = make_internal(t, motion, rotation_only);
  const auto ros_published = cc::gtsam_pose_to_ros(published.pose_mse(), published.pose().rotation());
  const auto ros_internal =
    cc::gtsam_pose_to_ros(reference.pose_cov, reference.pose.rotation());

  EXPECT_GT(ros_published[5 * 6 + 5], ros_internal[5 * 6 + 5] * 10.0);  // rz grows
  EXPECT_NEAR(ros_published[0], ros_internal[0], 1e-6);                 // x unchanged
  EXPECT_NEAR(ros_published[7], ros_internal[7], 1e-6);                 // y unchanged
}

TEST(OutputStage, CovarianceReturnsToTheInternalValueWhenSmoothingEnds)
{
  // (5) continuity of the output covariance at the end of the smoothing.
  OutputStage stage(default_config());
  const Motion motion{0.0, 0.0};
  double t = 0.0;
  stage.step(t, make_internal(t, motion, gtsam::Vector6::Zero()), {}, 1);
  const auto previous = [&](double at) -> std::optional<InternalEstimate> {
    return make_internal(at, motion, gtsam::Vector6::Zero());
  };

  t += kDt;
  PublishedEstimate published =
    stage.step(t, make_internal(t, motion, translation_offset(0.5)), previous, 2);
  int steps = 0;
  while (published.smoothing_active() && steps < 200) {
    t += kDt;
    published = stage.step(t, make_internal(t, motion, translation_offset(0.5)), previous, 2);
    ++steps;
  }
  const InternalEstimate reference = make_internal(t, motion, translation_offset(0.5));
  EXPECT_TRUE(published.pose_mse().isApprox(reference.pose_cov, 1e-9));
  EXPECT_TRUE(published.twist_mse().isApprox(reference.twist_cov, 1e-9));
}

TEST(OutputStage, RepeatedAnchorUpdatesConvergeMonotonically)
{
  // (9) A second 0.5 m update during smoothing must not restart the absorption from
  // scratch; |c| has to keep shrinking and reach IDLE within 2 x max duration.
  OutputStage stage(default_config());
  const Motion motion{10.0, 0.0};
  double t = 0.0;
  stage.step(t, make_internal(t, motion, gtsam::Vector6::Zero()), {}, 1);

  gtsam::Vector6 offset = translation_offset(1.0);
  gtsam::Vector6 previous_offset = gtsam::Vector6::Zero();
  const OutputStage::PreviousEstimateProvider previous =
    [&](double at) -> std::optional<InternalEstimate> {
    return make_internal(at, motion, previous_offset);
  };
  t += kDt;
  PublishedEstimate published = stage.step(t, make_internal(t, motion, offset), previous, 2);

  // second anchor update after 200 ms
  for (int i = 0; i < 10; ++i) {
    t += kDt;
    published = stage.step(t, make_internal(t, motion, offset), previous, 2);
  }
  previous_offset = offset;
  offset = translation_offset(1.5);
  t += kDt;
  published = stage.step(t, make_internal(t, motion, offset), previous, 3);

  double last_norm = published.smoothing_offset_lin();
  int steps = 0;
  while (published.smoothing_active() && steps < 200) {
    t += kDt;
    published = stage.step(t, make_internal(t, motion, offset), previous, 3);
    EXPECT_LE(published.smoothing_offset_lin(), last_norm + 1e-9);
    last_norm = published.smoothing_offset_lin();
    ++steps;
  }
  EXPECT_FALSE(published.smoothing_active());
  EXPECT_LE(steps * kDt, 2.0);
}

TEST(OutputStage, PoseCovarianceTransportMatchesMonteCarlo)
{
  // (11) The pose transport operator must be J_r^-1(-c). Replacing it by I or by
  // Ad_{C^-1} has to fail this comparison, which is checked explicitly below.
  gtsam::Vector6 c;
  c << 0.0, 0.0, 0.1, 1.0, 0.0, 0.0;  // 0.1 rad yaw + 1 m
  gtsam::Matrix6 sigma_internal = gtsam::Matrix6::Zero();
  sigma_internal.diagonal() << 1e-4, 1e-4, 4e-4, 0.01, 0.01, 0.0025;

  const gtsam::Pose3 x_internal(gtsam::Rot3::Yaw(0.4), gtsam::Point3(5.0, -2.0, 0.3));
  const gtsam::Pose3 x_published = x_internal * gtsam::Pose3::Expmap(c);

  const gtsam::Matrix6 j = cc::right_jacobian_inverse(-c);
  const gtsam::Matrix6 analytic = j * sigma_internal * j.transpose() + c * c.transpose();

  std::mt19937 generator(42);
  std::normal_distribution<double> normal(0.0, 1.0);
  const Eigen::LLT<gtsam::Matrix6> llt(sigma_internal);
  const gtsam::Matrix6 sqrt_sigma = llt.matrixL();

  gtsam::Matrix6 empirical = gtsam::Matrix6::Zero();
  const int samples = 40000;
  for (int i = 0; i < samples; ++i) {
    gtsam::Vector6 unit;
    for (int k = 0; k < 6; ++k) {
      unit(k) = normal(generator);
    }
    const gtsam::Vector6 xi = sqrt_sigma * unit;
    const gtsam::Pose3 x_true = x_internal * gtsam::Pose3::Expmap(xi);
    const gtsam::Vector6 error = gtsam::Pose3::Logmap(x_published.inverse() * x_true);
    empirical += error * error.transpose();
  }
  empirical /= static_cast<double>(samples);

  const double relative = (empirical - analytic).norm() / analytic.norm();
  EXPECT_LT(relative, 0.05) << "analytic:\n" << analytic << "\nempirical:\n" << empirical;

  // Wrong operators must be detectably worse.
  const gtsam::Matrix6 with_identity = sigma_internal + c * c.transpose();
  const gtsam::Matrix6 ad = cc::adjoint_of_inverse(c);
  const gtsam::Matrix6 with_adjoint = ad * sigma_internal * ad.transpose() + c * c.transpose();
  EXPECT_LT(
    (empirical - analytic).norm(), (empirical - with_identity).norm());
  EXPECT_LT((empirical - analytic).norm(), (empirical - with_adjoint).norm());
}

TEST(OutputStage, TwistCovarianceUsesTheAdjoint)
{
  // (12) For the twist the correct operator is Ad_{C^-1}: it maps a tangent vector
  // at the same instant to another reference, which is a different map from the
  // group composition Jacobian used for the pose.
  gtsam::Vector6 c;
  c << 0.0, 0.0, 0.1, 1.0, 0.0, 0.0;
  const gtsam::Matrix6 ad = cc::adjoint_of_inverse(c);
  const gtsam::Matrix6 sigma_twist = gtsam::Matrix6::Identity() * 0.01;
  const gtsam::Matrix6 transported = ad * sigma_twist * ad.transpose();

  // The internal twist expressed in the published frame is exactly Ad_{C^-1} xi.
  std::mt19937 generator(7);
  std::normal_distribution<double> normal(0.0, 0.1);
  gtsam::Matrix6 empirical = gtsam::Matrix6::Zero();
  const int samples = 20000;
  for (int i = 0; i < samples; ++i) {
    gtsam::Vector6 xi;
    for (int k = 0; k < 6; ++k) {
      xi(k) = normal(generator);
    }
    const gtsam::Vector6 mapped = ad * xi;
    empirical += mapped * mapped.transpose();
  }
  empirical /= static_cast<double>(samples);
  EXPECT_LT((empirical - transported).norm() / transported.norm(), 0.1);
}

TEST(OutputStage, InternalEstimateIsNeverSmoothed)
{
  // (13) OutputStage owns the smoothing: the internal estimate that a plugin
  // returns is unaffected, and PublishedEstimate cannot be constructed from
  // anything else (enforced by the type, see types.hpp).
  OutputStage stage(default_config());
  const Motion motion{0.0, 0.0};
  double t = 0.0;
  stage.step(t, make_internal(t, motion, gtsam::Vector6::Zero()), {}, 1);
  const auto previous = [&](double at) -> std::optional<InternalEstimate> {
    return make_internal(at, motion, gtsam::Vector6::Zero());
  };
  t += kDt;
  const InternalEstimate internal = make_internal(t, motion, translation_offset(1.0));
  const PublishedEstimate published = stage.step(t, internal, previous, 2);

  EXPECT_NEAR(internal.pose.translation().x(), 1.0, 1e-9);
  EXPECT_LT(published.pose().translation().x(), 0.1);
  EXPECT_TRUE(published.smoothing_active());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
