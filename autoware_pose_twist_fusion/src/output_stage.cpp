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

#include "autoware/pose_twist_fusion/output_stage.hpp"

#include "autoware/pose_twist_fusion/covariance_convert.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::pose_twist_fusion
{

namespace
{
gtsam::Vector3 rotation_part(const gtsam::Vector6 & xi)
{
  return xi.head<3>();
}
gtsam::Vector3 translation_part(const gtsam::Vector6 & xi)
{
  return xi.tail<3>();
}
}  // namespace

OutputStage::OutputStage(const Config & config) : config_(config)
{
}

void OutputStage::apply_immediately()
{
  smoothing_.clear();
}

void OutputStage::latch(double dt, Event * event)
{
  const double c_lin = translation_part(smoothing_.c).norm();
  const double c_rot = rotation_part(smoothing_.c).norm();
  if (event != nullptr) {
    event->jump_lin = c_lin;
    event->jump_rot = c_rot;
  }

  if (c_lin <= config_.reset.max_anchor_jump_m && c_rot <= config_.reset.max_anchor_jump_rad) {
    // Small enough to apply in a single step (design §5.3.7 発動判定).
    smoothing_.clear();
    return;
  }

  const double max_duration_s = static_cast<double>(config_.reset.smoothing_max_duration_ms) * 1e-3;
  const double t_apply = std::clamp(
    std::max(
      c_lin / std::max(config_.reset.smoothing_max_lin_rate_mps, 1e-9),
      c_rot / std::max(config_.reset.smoothing_max_ang_rate_rps, 1e-9)),
    dt, max_duration_s);

  const double c_norm = smoothing_.c.norm();
  const bool continue_current =
    smoothing_.mode == detail::SmoothingState::Mode::kSmoothing && smoothing_.t_left >= dt &&
    c_norm <= config_.reset.smoothing_relatch_ratio * smoothing_.latched_norm;

  if (continue_current) {
    // Keep the total absorption time, only re-aim the direction (design §5.3.7).
    smoothing_.u = -smoothing_.c / smoothing_.t_left;
    if (event != nullptr) {
      event->relatched = false;
    }
  } else {
    smoothing_.t_left = t_apply;
    smoothing_.u = -smoothing_.c / t_apply;
    smoothing_.latched_norm = c_norm;
    if (event != nullptr) {
      event->relatched = smoothing_.mode == detail::SmoothingState::Mode::kSmoothing;
    }
  }

  smoothing_.mode = detail::SmoothingState::Mode::kSmoothing;
  ++smoothing_count_;
  if (event != nullptr) {
    event->smoothing_triggered = true;
  }

  const bool rate_exceeded =
    translation_part(smoothing_.u).norm() > config_.reset.smoothing_max_lin_rate_mps * 1.001 ||
    rotation_part(smoothing_.u).norm() > config_.reset.smoothing_max_ang_rate_rps * 1.001;
  if (rate_exceeded) {
    ++rate_exceeded_count_;
    if (event != nullptr) {
      event->rate_limit_exceeded = true;
    }
  }
}

PublishedEstimate OutputStage::step(
  double t_now, const InternalEstimate & current, const PreviousEstimateProvider & previous,
  EstimateGeneration generation, Event * event)
{
  Event local_event;
  Event & ev = (event != nullptr) ? *event : local_event;
  ev = Event{};

  PublishedEstimate published;
  if (!current.valid) {
    has_last_time_ = false;
    return published;
  }

  const double nominal_dt = 1.0 / std::max(config_.output_rate_hz, 1e-6);
  double dt = nominal_dt;
  if (has_last_time_) {
    const double measured = t_now - last_time_;
    if (measured > 0.0 && measured < 5.0 * nominal_dt) {
      dt = measured;
    }
  }
  last_time_ = t_now;
  has_last_time_ = true;

  if (!has_generation_) {
    has_generation_ = true;
    generation_ = generation;
    smoothing_.clear();
  } else if (generation != generation_) {
    ev.anchor_update = true;
    generation_ = generation;
    const std::optional<InternalEstimate> previous_estimate =
      previous ? previous(t_now) : std::nullopt;
    if (previous_estimate.has_value() && previous_estimate->valid) {
      // Move the jump into the offset without moving the published pose:
      //   X_pub_hold = X_internal_old(t_now) . Exp(c)
      //   c <- Log( X_internal_new(t_now)^-1 . X_pub_hold )
      const gtsam::Pose3 x_pub_hold =
        previous_estimate->pose * gtsam::Pose3::Expmap(smoothing_.c);
      smoothing_.c = gtsam::Pose3::Logmap(current.pose.inverse() * x_pub_hold);
    } else {
      // No previous snapshot: nothing to hold, apply the new anchor immediately.
      smoothing_.c.setZero();
    }
    latch(dt, &ev);
  }

  if (smoothing_.mode == detail::SmoothingState::Mode::kSmoothing) {
    // u is parallel to c, so this addition is exact on SE(3) (D32).
    smoothing_.c += smoothing_.u * dt;
    smoothing_.t_left -= dt;
    const bool done_by_time = smoothing_.t_left <= 0.0;
    const bool done_by_size =
      translation_part(smoothing_.c).norm() <= config_.reset.smoothing_done_m &&
      rotation_part(smoothing_.c).norm() <= config_.reset.smoothing_done_rad;
    if (done_by_time || done_by_size) {
      smoothing_.clear();
      ev.smoothing_finished = true;
    }
  }

  const gtsam::Vector6 & c = smoothing_.c;
  const gtsam::Vector6 & u = smoothing_.u;
  const gtsam::Pose3 offset = gtsam::Pose3::Expmap(c);

  published.valid_ = true;
  published.stamp_ = t_now;
  published.pose_ = current.pose * offset;
  published.biased_pose_ = current.biased_pose * offset;
  published.yaw_bias_ = current.yaw_bias;
  published.has_flat_reference_ = current.has_flat_reference;
  published.flat_z_ = current.flat_z;
  published.flat_roll_ = current.flat_roll;
  published.flat_pitch_ = current.flat_pitch;
  published.smoothing_active_ = smoothing_.mode == detail::SmoothingState::Mode::kSmoothing;
  published.smoothing_offset_lin_ = translation_part(c).norm();
  published.smoothing_offset_rot_ = rotation_part(c).norm();

  // --- twist ------------------------------------------------------------
  // internal body twist, GTSAM order [omega; nu]
  gtsam::Vector6 xi_internal;
  xi_internal.head<3>() = current.angular_velocity_body;
  xi_internal.tail<3>() = current.pose.rotation().transpose() * current.velocity_world;

  const gtsam::Matrix6 ad_c_inv = covariance_convert::adjoint_of_inverse(c);

  gtsam::Vector6 xi_published = xi_internal;
  if (config_.twist_smoothing_consistency) {
    // xi_pub = Ad_{C^-1} xi_int + u  (exact) ~= xi_int + u  (|c| << 1)
    xi_published = config_.exact_smoothing_adjoint
                     ? gtsam::Vector6(ad_c_inv * xi_internal + u)
                     : gtsam::Vector6(xi_internal + u);
  }
  published.angular_body_ = xi_published.head<3>();
  published.linear_body_ = xi_published.tail<3>();

  // --- covariance (MSE matrices, D40) -----------------------------------
  // Transport the internal covariance to the published reference *before* adding
  // the known-bias terms; pose uses J_r^-1(-c), twist uses Ad_{C^-1}.
  const gtsam::Matrix6 j = covariance_convert::right_jacobian_inverse(-c);
  gtsam::Matrix6 pose_mse = j * current.pose_cov * j.transpose() + c * c.transpose();
  gtsam::Matrix6 twist_mse = ad_c_inv * current.twist_cov * ad_c_inv.transpose();
  if (config_.twist_smoothing_consistency) {
    twist_mse += u * u.transpose();
  }

  published.pose_mse_ = covariance_convert::symmetrize(pose_mse);
  published.twist_mse_ = covariance_convert::symmetrize(twist_mse);
  return published;
}

}  // namespace autoware::pose_twist_fusion
