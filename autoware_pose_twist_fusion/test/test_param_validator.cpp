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
// Covers every row of design_last.md §6.2, including which rows are start-up
// failures and which ones are only diagnostics WARN / INFO.

#include "autoware/pose_twist_fusion/param_validator.hpp"

#include <gtest/gtest.h>

#include <string>

using autoware::pose_twist_fusion::Parameters;
using autoware::pose_twist_fusion::validate_parameters;

namespace
{
bool has_error_containing(
  const autoware::pose_twist_fusion::ValidationResult & result, const std::string & needle)
{
  for (const auto & error : result.errors) {
    if (error.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

bool has_warning_containing(
  const autoware::pose_twist_fusion::ValidationResult & result, const std::string & needle)
{
  for (const auto & warning : result.warnings) {
    if (warning.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}
}  // namespace

TEST(ParamValidator, DefaultsAreValid)
{
  // "The node must start and converge with the defaults alone" (design §5.3.8).
  const Parameters params;
  const auto result = validate_parameters(params);
  EXPECT_TRUE(result.ok()) << result.error_message();
}

TEST(ParamValidator, PeriodOrdering)
{
  Parameters params;
  params.factor_graph_fusion.graph1_node_interval_ms = 200;  // > max_node_interval
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "period ordering"));
}

TEST(ParamValidator, LagWindowLowerBound)
{
  Parameters params;
  // 1000 >= 500 + 300 + 100 holds; shrinking the lag breaks it.
  params.factor_graph_fusion.graph2_lag_ms = 800;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "graph2_lag_ms"));
}

TEST(ParamValidator, LagEqualToOptimizeIntervalIsRejected)
{
  Parameters params;
  params.factor_graph_fusion.graph2_lag_ms = 500;
  params.factor_graph_fusion.graph2_optimize_interval_ms = 500;
  params.factor_graph_fusion.timing.max_measurement_delay_ms = 0;
  params.factor_graph_fusion.graph2.lag_margin_ms = 0;
  // The literal reading of R10 (reset every tick) collapses t_next_begin onto the
  // window edge, which §0.1 (2) explicitly refuses to provide.
  EXPECT_TRUE(has_error_containing(
    validate_parameters(params), "t_begin < t_next_begin < t_end"));
}

TEST(ParamValidator, HistoryLengthLowerBound)
{
  Parameters params;
  params.factor_graph_fusion.timing.history_length_ms = 500;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "history_length_ms"));
}

TEST(ParamValidator, BufferLengthCoversOptimizerFailures)
{
  Parameters params;
  // 1000 + 500 * (1 + 3) + 500 = 3500 is the minimum (D35).
  params.factor_graph_fusion.buffer.imu_buffer_ms = 3000;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "imu_buffer_ms"));
  params.factor_graph_fusion.buffer.imu_buffer_ms = 3500;
  params.factor_graph_fusion.buffer.vehicle_twist_buffer_ms = 3000;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "vehicle_twist_buffer_ms"));
}

TEST(ParamValidator, WeightLowerBound)
{
  Parameters params;
  params.factor_graph_fusion.covariance_correction.w_min = 0.0;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "w_min"));
  params.factor_graph_fusion.covariance_correction.w_min = 1.5;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "w_min"));
}

TEST(ParamValidator, OutputRateMustNotExceedTheGraphRate)
{
  Parameters params;
  params.output_rate_hz = 200.0;  // 5 ms < graph1_node_interval_ms (20 ms)
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "output_rate_hz is faster"));
}

TEST(ParamValidator, Enums)
{
  Parameters params;
  params.fusion_method = "kalman";
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "fusion_method"));

  params = Parameters{};
  params.factor_graph_fusion.covariance_correction.weight_function = "sigmoid";
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "weight_function"));

  params = Parameters{};
  params.factor_graph_fusion.graph2.marginalization = "drop";
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "marginalization"));

  params = Parameters{};
  params.factor_graph_fusion.wheel_speed.graph2_binding = "all";
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "graph2_binding"));
}

TEST(ParamValidator, NodeBudget)
{
  Parameters params;
  params.factor_graph_fusion.graph2_max_nodes = 15;  // 1000/100 + 10 = 20 required
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "graph2_max_nodes"));
}

TEST(ParamValidator, BoundaryMergeNoise)
{
  Parameters params;
  params.factor_graph_fusion.graph2.boundary_merge_yaw_rate_rps = 0.0;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "boundary_merge_yaw_rate_rps"));
}

TEST(ParamValidator, WheelSpeedBinding)
{
  Parameters params;
  params.factor_graph_fusion.wheel_speed.graph2_max_time_offset_ms = 5;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "graph2_max_time_offset_ms"));

  params = Parameters{};
  params.factor_graph_fusion.wheel_speed.graph2_max_time_offset_ms = 80;  // > 100/2
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "graph2_max_time_offset_ms"));

  params = Parameters{};
  params.factor_graph_fusion.wheel_speed.timeout_ms = 10;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "wheel_speed.timeout_ms"));
}

TEST(ParamValidator, MeasurementHistoryCap)
{
  Parameters params;
  params.factor_graph_fusion.graph2.measurement_history_max = 32;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "measurement_history_max"));
}

TEST(ParamValidator, SmoothingThresholds)
{
  Parameters params;
  params.factor_graph_fusion.reset.smoothing_done_m = 0.05;  // >= max_anchor_jump_m
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "smoothing_done_m"));

  params = Parameters{};
  params.factor_graph_fusion.reset.smoothing_relatch_ratio = 1.0;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "smoothing_relatch_ratio"));

  params = Parameters{};
  params.factor_graph_fusion.reset.smoothing_max_duration_ms = 10;  // < 1000/50
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "smoothing_max_duration_ms"));
}

TEST(ParamValidator, AnchorJumpThresholdIsStrictlyBelowTheTestCriterion)
{
  // D45: equality with the test 3 criterion (0.05 m / 0.005 rad) must be a start-up
  // failure, otherwise the test fails on the boundary case by construction.
  Parameters params;
  params.factor_graph_fusion.reset.max_anchor_jump_m = 0.05;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "max_anchor_jump_m"));

  params = Parameters{};
  params.factor_graph_fusion.reset.max_anchor_jump_rad = 0.005;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "max_anchor_jump_rad"));

  params = Parameters{};
  params.factor_graph_fusion.reset.max_anchor_jump_m = 0.049;
  params.factor_graph_fusion.reset.max_anchor_jump_rad = 0.0049;
  EXPECT_TRUE(validate_parameters(params).ok());
}

TEST(ParamValidator, CrossCovarianceInflationWarnsButStarts)
{
  Parameters params;
  params.factor_graph_fusion.reset.carry_cross_covariance = false;
  params.factor_graph_fusion.reset.cross_covariance_inflation = 1.0;
  const auto result = validate_parameters(params);
  EXPECT_TRUE(result.ok()) << result.error_message();
  EXPECT_TRUE(has_warning_containing(result, "NOT conservative"));

  params.factor_graph_fusion.reset.cross_covariance_inflation = 0.5;
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "cross_covariance_inflation"));
}

TEST(ParamValidator, RetiredParametersFailStartup)
{
  Parameters params;
  params.declared_removed_parameters = {"slam.max_relative_jump_m"};
  const auto result = validate_parameters(params);
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(has_error_containing(result, "was removed"));

  params.declared_removed_parameters = {"graph2.force_boundary_node"};
  EXPECT_TRUE(has_error_containing(validate_parameters(params), "INV-2"));
}

TEST(ParamValidator, GnssInitializationYaw)
{
  Parameters params;
  params.factor_graph_fusion.initialization.pose_source = "gnss";
  params.factor_graph_fusion.initialization.gnss_yaw_source = "zero_with_large_covariance";
  params.factor_graph_fusion.initialization.gnss_yaw_fallback_stddev_rad = 1.0;
  auto result = validate_parameters(params);
  EXPECT_TRUE(result.ok()) << result.error_message();
  EXPECT_TRUE(has_warning_containing(result, "test-only"));

  params.factor_graph_fusion.initialization.gnss_yaw_fallback_stddev_rad = 2.0;
  EXPECT_TRUE(
    has_error_containing(validate_parameters(params), "gnss_yaw_fallback_stddev_rad"));
}

TEST(ParamValidator, ThreeLayerWarning)
{
  Parameters params;
  params.factor_graph_fusion.robust_kernel = "huber";
  const auto result = validate_parameters(params);
  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(has_warning_containing(result, "layers A, B and C"));
}

TEST(ParamValidator, EkfFlattenInfo)
{
  Parameters params;
  params.fusion_method = "ekf";
  params.output.flatten_to_2d = false;
  const auto result = validate_parameters(params);
  EXPECT_TRUE(result.ok());
  ASSERT_FALSE(result.infos.empty());
  EXPECT_NE(result.infos.front().find("flatten_to_2d"), std::string::npos);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
