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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__MEASUREMENT_QUEUE_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__MEASUREMENT_QUEUE_HPP_

#include "plugins/factor_graph_fusion/fg_types.hpp"

#include <mutex>
#include <vector>

namespace autoware::pose_twist_fusion::fg
{

/// Hand-off between the measurement callbacks and the Graph2 worker (design §5.3.2a).
///
/// It is *only* a hand-off: the durable store is Graph2MeasurementHistory. The queue
/// holds fully corrected entries (noise model already frozen), so the callback stays
/// O(1) and never touches Graph2.
class PendingMeasurementQueue
{
public:
  void push(const AcceptedMeasurement & measurement);
  [[nodiscard]] std::vector<AcceptedMeasurement> drain();
  [[nodiscard]] size_t size() const;
  void clear();

private:
  mutable std::mutex pending_mutex_;
  std::vector<AcceptedMeasurement> pending_;
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__MEASUREMENT_QUEUE_HPP_
