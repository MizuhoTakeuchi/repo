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

#ifndef PLUGINS__FACTOR_GRAPH_FUSION__VEHICLE_TWIST_BUFFER_HPP_
#define PLUGINS__FACTOR_GRAPH_FUSION__VEHICLE_TWIST_BUFFER_HPP_

#include "plugins/factor_graph_fusion/fg_types.hpp"

#include <deque>
#include <mutex>
#include <vector>

namespace autoware::pose_twist_fusion::fg
{

/// Ring buffer of wheel speed samples.
///
/// Beyond replay (D15) it also stores the **layer A rejection flag** of each sample
/// (D42): a sample rejected by the chi-squared gate inside Graph1 must not be used
/// by Graph2 either, and replay has to reproduce the very same decision.
class VehicleTwistBuffer
{
public:
  explicit VehicleTwistBuffer(double retention_s);

  void push(const TwistSample & sample);
  [[nodiscard]] std::vector<TwistSample> since(double t) const;
  [[nodiscard]] std::vector<TwistSample> range(double t0, double t1) const;
  /// Writes the gate result (rejected / d2) back for the samples in `evaluated`,
  /// matched by stamp. Used by Graph1 so the flags survive into the Graph2 tick.
  void apply_gate_results(const std::vector<TwistSample> & evaluated);
  [[nodiscard]] bool empty() const;
  [[nodiscard]] size_t size() const;
  [[nodiscard]] double latest_stamp() const;
  void clear();

private:
  mutable std::mutex mutex_;
  std::deque<TwistSample> samples_;
  double retention_s_;
};

}  // namespace autoware::pose_twist_fusion::fg

#endif  // PLUGINS__FACTOR_GRAPH_FUSION__VEHICLE_TWIST_BUFFER_HPP_
