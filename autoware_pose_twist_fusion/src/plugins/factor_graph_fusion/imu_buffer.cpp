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

#include "plugins/factor_graph_fusion/imu_buffer.hpp"

#include <algorithm>

namespace autoware::pose_twist_fusion::fg
{

ImuBuffer::ImuBuffer(double retention_s) : retention_s_(retention_s)
{
}

void ImuBuffer::push(const ImuSample & sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Out-of-order IMU is pathological but must not corrupt the ordering invariant
  // the replay relies on, so insert at the right place instead of appending.
  if (samples_.empty() || samples_.back().stamp <= sample.stamp) {
    samples_.push_back(sample);
  } else {
    const auto it = std::upper_bound(
      samples_.begin(), samples_.end(), sample.stamp,
      [](double stamp, const ImuSample & s) { return stamp < s.stamp; });
    samples_.insert(it, sample);
  }
  const double cutoff = samples_.back().stamp - retention_s_;
  while (!samples_.empty() && samples_.front().stamp < cutoff) {
    samples_.pop_front();
  }
}

std::vector<ImuSample> ImuBuffer::since(double t) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ImuSample> out;
  for (const auto & s : samples_) {
    if (s.stamp > t) {
      out.push_back(s);
    }
  }
  return out;
}

std::vector<ImuSample> ImuBuffer::range(double t0, double t1) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ImuSample> out;
  for (const auto & s : samples_) {
    if (s.stamp > t0 && s.stamp <= t1) {
      out.push_back(s);
    }
  }
  return out;
}

bool ImuBuffer::empty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.empty();
}

size_t ImuBuffer::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.size();
}

double ImuBuffer::latest_stamp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.empty() ? 0.0 : samples_.back().stamp;
}

double ImuBuffer::oldest_stamp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.empty() ? 0.0 : samples_.front().stamp;
}

void ImuBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
}

}  // namespace autoware::pose_twist_fusion::fg
