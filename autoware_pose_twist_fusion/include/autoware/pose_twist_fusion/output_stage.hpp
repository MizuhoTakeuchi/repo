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

#ifndef AUTOWARE__POSE_TWIST_FUSION__OUTPUT_STAGE_HPP_
#define AUTOWARE__POSE_TWIST_FUSION__OUTPUT_STAGE_HPP_

#include "autoware/pose_twist_fusion/parameters.hpp"
#include "autoware/pose_twist_fusion/smoothing_state.hpp"
#include "autoware/pose_twist_fusion/types.hpp"

#include <cstdint>
#include <functional>
#include <optional>

namespace autoware::pose_twist_fusion
{

/// Owner of the anchor-jump smoothing and of the internal -> published derivation
/// (design §5.3.7, D37 / D40 / D43).
///
/// Deliberately free of ROS types so that the mathematics can be unit tested
/// (試験 21 の単体部分) without spinning a node.
class OutputStage
{
public:
  struct Config
  {
    double output_rate_hz{50.0};
    ResetParams reset;
    bool twist_smoothing_consistency{true};
    bool exact_smoothing_adjoint{false};
    double undetermined_variance{1.0e4};
  };

  /// What happened during one step; the node turns this into diagnostics (D45).
  struct Event
  {
    bool anchor_update{false};
    bool smoothing_triggered{false};
    bool smoothing_finished{false};
    /// The latched rate had to exceed smoothing_max_*_rate because the jump does
    /// not fit into smoothing_max_duration_ms -> diagnostics WARN.
    bool rate_limit_exceeded{false};
    bool relatched{false};
    double jump_lin{0.0};
    double jump_rot{0.0};
  };

  explicit OutputStage(const Config & config);

  /// One output step.
  ///
  /// Supplies `t_now` evaluated from the **previous** generation's snapshot.
  /// It is only invoked on the step that detects an anchor update, so plugins pay
  /// the propagation cost once per anchor rather than once per output step.
  using PreviousEstimateProvider = std::function<std::optional<InternalEstimate>(double)>;

  /// @param t_now      output timer time [s]
  /// @param current    internal estimate of the current generation, evaluated at t_now
  /// @param previous   provider for the previous generation (may be empty; then the
  ///                   anchor jump is applied immediately)
  /// @param generation generation of `current`
  PublishedEstimate step(
    double t_now, const InternalEstimate & current, const PreviousEstimateProvider & previous,
    EstimateGeneration generation, Event * event = nullptr);

  /// initialpose / (re-)initialization: apply immediately, never smooth (§5.3.7).
  void apply_immediately();

  /// Number of smoothing activations since start (diagnostics counter, D45).
  [[nodiscard]] std::uint64_t smoothing_count() const { return smoothing_count_; }
  [[nodiscard]] std::uint64_t rate_exceeded_count() const { return rate_exceeded_count_; }
  [[nodiscard]] const detail::SmoothingState & state() const { return smoothing_; }

private:
  void latch(double dt, Event * event);

  Config config_;
  detail::SmoothingState smoothing_;
  bool has_generation_{false};
  EstimateGeneration generation_{0};
  double last_time_{0.0};
  bool has_last_time_{false};
  std::uint64_t smoothing_count_{0};
  std::uint64_t rate_exceeded_count_{0};
};

}  // namespace autoware::pose_twist_fusion

#endif  // AUTOWARE__POSE_TWIST_FUSION__OUTPUT_STAGE_HPP_
