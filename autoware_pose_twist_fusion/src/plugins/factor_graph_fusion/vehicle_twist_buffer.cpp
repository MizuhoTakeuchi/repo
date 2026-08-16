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

#include "plugins/factor_graph_fusion/vehicle_twist_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::pose_twist_fusion::fg
{

VehicleTwistBuffer::VehicleTwistBuffer(double retention_s) : retention_s_(retention_s)
{
}

void VehicleTwistBuffer::push(const TwistSample & sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty() || samples_.back().stamp <= sample.stamp) {
    samples_.push_back(sample);
  } else {
    const auto it = std::upper_bound(
      samples_.begin(), samples_.end(), sample.stamp,
      [](double stamp, const TwistSample & s) { return stamp < s.stamp; });
    samples_.insert(it, sample);
  }
  const double cutoff = samples_.back().stamp - retention_s_;
  while (!samples_.empty() && samples_.front().stamp < cutoff) {
    samples_.pop_front();
  }
}

std::vector<TwistSample> VehicleTwistBuffer::since(double t) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TwistSample> out;
  for (const auto & s : samples_) {
    if (s.stamp > t) {
      out.push_back(s);
    }
  }
  return out;
}

std::vector<TwistSample> VehicleTwistBuffer::range(double t0, double t1) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TwistSample> out;
  for (const auto & s : samples_) {
    if (s.stamp > t0 && s.stamp <= t1) {
      out.push_back(s);
    }
  }
  return out;
}

void VehicleTwistBuffer::apply_gate_results(const std::vector<TwistSample> & evaluated)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = samples_.begin();
  for (const auto & e : evaluated) {
    while (it != samples_.end() && it->stamp < e.stamp - 1e-9) {
      ++it;
    }
    if (it == samples_.end()) {
      break;
    }
    if (std::abs(it->stamp - e.stamp) <= 1e-9) {
      it->rejected = e.rejected;
      it->gate_evaluated = e.gate_evaluated;
      it->d2 = e.d2;
    }
  }
}

bool VehicleTwistBuffer::empty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.empty();
}

size_t VehicleTwistBuffer::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.size();
}

double VehicleTwistBuffer::latest_stamp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.empty() ? 0.0 : samples_.back().stamp;
}

void VehicleTwistBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
}

}  // namespace autoware::pose_twist_fusion::fg
