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

#include "plugins/factor_graph_fusion/state_history.hpp"

#include <gtsam/geometry/Pose3.h>

#include <algorithm>

namespace autoware::pose_twist_fusion::fg
{

StateHistory::StateHistory(double length_s) : length_s_(length_s)
{
}

void StateHistory::push(const StateSample & sample)
{
  if (!samples_.empty() && sample.stamp <= samples_.back().stamp) {
    // Replay re-creates grid points; overwrite instead of duplicating them.
    while (!samples_.empty() && samples_.back().stamp >= sample.stamp - 1e-9) {
      samples_.pop_back();
    }
  }
  samples_.push_back(sample);
  const double cutoff = samples_.back().stamp - length_s_;
  while (!samples_.empty() && samples_.front().stamp < cutoff) {
    samples_.pop_front();
  }
}

void StateHistory::erase_after(double t)
{
  while (!samples_.empty() && samples_.back().stamp > t + 1e-9) {
    samples_.pop_back();
  }
}

void StateHistory::clear()
{
  samples_.clear();
}

double StateHistory::oldest_stamp() const
{
  return samples_.empty() ? 0.0 : samples_.front().stamp;
}

double StateHistory::latest_stamp() const
{
  return samples_.empty() ? 0.0 : samples_.back().stamp;
}

std::optional<StateSample> StateHistory::interpolate(double t) const
{
  if (samples_.empty()) {
    return std::nullopt;
  }
  if (t < samples_.front().stamp - 1e-9 || t > samples_.back().stamp + 1e-9) {
    return std::nullopt;
  }

  const auto it = std::lower_bound(
    samples_.begin(), samples_.end(), t,
    [](const StateSample & s, double stamp) { return s.stamp < stamp; });
  if (it == samples_.begin()) {
    return samples_.front();
  }
  const auto & next = *it;
  const auto & prev = *(it - 1);
  const double span = next.stamp - prev.stamp;
  if (span <= 1e-9) {
    return next;
  }
  const double alpha = (t - prev.stamp) / span;

  StateSample out;
  out.stamp = t;
  out.pose = gtsam::interpolate(prev.pose, next.pose, alpha);
  out.velocity = (1.0 - alpha) * prev.velocity + alpha * next.velocity;
  // First order approximation; PSD is preserved because a convex combination of
  // PSD matrices is PSD (design §5.3.4).
  out.pose_cov = (1.0 - alpha) * prev.pose_cov + alpha * next.pose_cov;
  out.vel_cov = (1.0 - alpha) * prev.vel_cov + alpha * next.vel_cov;
  return out;
}

}  // namespace autoware::pose_twist_fusion::fg
