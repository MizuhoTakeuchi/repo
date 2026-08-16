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

#include "plugins/factor_graph_fusion/measurement_queue.hpp"

#include <utility>

namespace autoware::pose_twist_fusion::fg
{

void PendingMeasurementQueue::push(const AcceptedMeasurement & measurement)
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_.push_back(measurement);
}

std::vector<AcceptedMeasurement> PendingMeasurementQueue::drain()
{
  std::vector<AcceptedMeasurement> out;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    out.swap(pending_);
  }
  return out;
}

size_t PendingMeasurementQueue::size() const
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  return pending_.size();
}

void PendingMeasurementQueue::clear()
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_.clear();
}

}  // namespace autoware::pose_twist_fusion::fg
