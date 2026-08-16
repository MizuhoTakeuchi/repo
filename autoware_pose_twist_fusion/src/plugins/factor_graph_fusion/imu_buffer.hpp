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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__IMU_BUFFER_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__IMU_BUFFER_HPP_

#include "plugins/factor_graph_fusion/fg_types.hpp"

#include <deque>
#include <mutex>
#include <vector>

namespace autoware::pose_twist_fusion::fg
{

/// Ring buffer of base_link IMU samples used for the anchor replay (D15) and for
/// the per-interval preintegration of Graph2 (design §5.3.2 (c)).
/// Retention is validated in §6.2 against the worst case window growth.
class ImuBuffer
{
public:
  explicit ImuBuffer(double retention_s);

  void push(const ImuSample & sample);
  /// Samples with stamp > t, in ascending stamp order.
  [[nodiscard]] std::vector<ImuSample> since(double t) const;
  /// Samples with t0 < stamp <= t1, in ascending stamp order.
  [[nodiscard]] std::vector<ImuSample> range(double t0, double t1) const;
  [[nodiscard]] bool empty() const;
  [[nodiscard]] size_t size() const;
  [[nodiscard]] double latest_stamp() const;
  [[nodiscard]] double oldest_stamp() const;
  void clear();

private:
  mutable std::mutex mutex_;
  std::deque<ImuSample> samples_;
  double retention_s_;
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__IMU_BUFFER_HPP_
