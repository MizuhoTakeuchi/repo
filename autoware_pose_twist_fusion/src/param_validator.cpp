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

#include "autoware/pose_twist_fusion/param_validator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::pose_twist_fusion
{

namespace
{
bool in_set(const std::string & value, const std::vector<std::string> & allowed)
{
  return std::find(allowed.begin(), allowed.end(), value) != allowed.end();
}

void check_enum(
  ValidationResult & r, const std::string & name, const std::string & value,
  const std::vector<std::string> & allowed)
{
  if (in_set(value, allowed)) {
    return;
  }
  std::ostringstream oss;
  oss << name << ": '" << value << "' is not one of {";
  for (size_t i = 0; i < allowed.size(); ++i) {
    oss << (i != 0 ? ", " : "") << allowed[i];
  }
  oss << "}";
  r.errors.push_back(oss.str());
}

void check_positive(ValidationResult & r, const std::string & name, double value)
{
  if (!(std::isfinite(value) && value > 0.0)) {
    r.errors.push_back(name + " must be finite and > 0 (got " + std::to_string(value) + ")");
  }
}
}  // namespace

std::string ValidationResult::error_message() const
{
  std::ostringstream oss;
  oss << "invalid parameters (" << errors.size() << "):";
  for (const auto & e : errors) {
    oss << "\n  - " << e;
  }
  return oss.str();
}

ValidationResult validate_parameters(const Parameters & p)
{
  ValidationResult r;
  const auto & f = p.factor_graph_fusion;
  const auto & t = f.timing;
  const auto & g2 = f.graph2;
  const auto & reset = f.reset;
  const auto & cc = f.covariance_correction;
  const auto & ws = f.wheel_speed;
  const auto & init = f.initialization;

  // --- retired parameters: never silently ignore a setting (design §6.2) -----
  for (const auto & name : p.declared_removed_parameters) {
    if (name == "slam.max_relative_jump_m" || name == "slam.max_relative_jump_rad") {
      r.errors.push_back(
        name +
        " was removed: SLAM jump detection is now a chi-squared test on the dead-reckoning "
        "residual plus a dynamic upper bound (slam.max_speed_mps / max_yaw_rate_rps / "
        "jump_margin_m / jump_margin_rad). A fixed distance threshold rejects normal driving "
        "(15 m/s x 100 ms = 1.5 m). See design_last.md §5.3.3c / D38.");
    } else if (name == "graph2.force_boundary_node") {
      r.errors.push_back(
        "graph2.force_boundary_node was removed: forcing a node at t_next_begin is a "
        "precondition of the algorithm (INV-2), not an option. See design_last.md D29.");
    } else {
      r.errors.push_back(name + " is a retired parameter and must be removed.");
    }
  }

  // --- enums ----------------------------------------------------------------
  check_enum(
    r, "fusion_method", p.fusion_method,
    {"ekf", "lio_sam_imu_preintegration", "factor_graph_fusion", "passthrough"});
  check_enum(
    r, "covariance_correction.weight_function", cc.weight_function,
    {"exp_excess_nis", "inverse_nis", "inverse_linear"});
  check_enum(r, "slam.use_as", f.slam.use_as, {"relative", "absolute"});
  check_enum(
    r, "imu.lever_arm_compensation", f.imu.lever_arm_compensation,
    {"rotation_only", "centrifugal", "full"});
  check_enum(r, "imu.factor_type", f.imu.factor_type, {"imu_factor", "combined_imu_factor"});
  check_enum(r, "robust_kernel", f.robust_kernel, {"none", "huber", "cauchy", "dcs", "gm"});
  check_enum(r, "lio_sam.covariance_source", p.lio_sam.covariance_source, {"propagated", "fixed"});
  check_enum(r, "initialization.pose_source", init.pose_source, {"initialpose", "ndt", "gnss"});
  check_enum(
    r, "initialization.gnss_yaw_source", init.gnss_yaw_source,
    {"auto", "dual_antenna", "course", "initialpose", "zero_with_large_covariance"});
  check_enum(r, "graph2.optimizer", g2.optimizer, {"levenberg_marquardt", "isam2"});
  check_enum(r, "graph2.marginalization", g2.marginalization, {"linear_container", "prior_triple"});
  check_enum(r, "wheel_speed.graph2_binding", ws.graph2_binding, {"nearest", "interpolate", "none"});

  // --- periods --------------------------------------------------------------
  if (!(0 < f.graph1_node_interval_ms &&
        f.graph1_node_interval_ms < f.graph2_max_node_interval_ms &&
        f.graph2_max_node_interval_ms <= f.graph2_optimize_interval_ms)) {
    r.errors.push_back(
      "period ordering violated: require 0 < graph1_node_interval_ms(" +
      std::to_string(f.graph1_node_interval_ms) + ") < graph2_max_node_interval_ms(" +
      std::to_string(f.graph2_max_node_interval_ms) + ") <= graph2_optimize_interval_ms(" +
      std::to_string(f.graph2_optimize_interval_ms) + ")");
  }

  // --- fixed lag window -----------------------------------------------------
  const int lag_lower_bound =
    f.graph2_optimize_interval_ms + t.max_measurement_delay_ms + g2.lag_margin_ms;
  if (f.graph2_lag_ms < lag_lower_bound) {
    r.errors.push_back(
      "graph2_lag_ms(" + std::to_string(f.graph2_lag_ms) +
      ") must be >= graph2_optimize_interval_ms + max_measurement_delay_ms + lag_margin_ms (" +
      std::to_string(lag_lower_bound) +
      "); otherwise a delayed measurement can fall outside the window and is lost.");
  }
  if (!(0 < f.graph2_optimize_interval_ms && f.graph2_optimize_interval_ms < f.graph2_lag_ms)) {
    r.errors.push_back(
      "require 0 < graph2_optimize_interval_ms < graph2_lag_ms so that "
      "t_begin < t_next_begin < t_end holds (boundary node must not degenerate).");
  }

  // --- history length -------------------------------------------------------
  const int history_lower_bound =
    t.max_measurement_delay_ms + f.graph2_optimize_interval_ms + f.graph1_node_interval_ms * 5;
  if (t.history_length_ms < history_lower_bound) {
    r.errors.push_back(
      "timing.history_length_ms(" + std::to_string(t.history_length_ms) + ") must be >= " +
      std::to_string(history_lower_bound) +
      " (= max_measurement_delay_ms + graph2_optimize_interval_ms + 5 x graph1_node_interval_ms)");
  }
  if (!(0 < t.max_measurement_delay_ms && t.max_measurement_delay_ms < t.history_length_ms)) {
    r.errors.push_back("require 0 < timing.max_measurement_delay_ms < timing.history_length_ms");
  }

  // --- replay buffers (must survive a run of optimizer failures, D35) --------
  const int buffer_lower_bound =
    f.graph2_lag_ms + f.graph2_optimize_interval_ms * (1 + f.failure.optimizer_failure_limit) + 500;
  if (f.buffer.imu_buffer_ms < buffer_lower_bound) {
    r.errors.push_back(
      "buffer.imu_buffer_ms(" + std::to_string(f.buffer.imu_buffer_ms) +
      ") must be >= " + std::to_string(buffer_lower_bound));
  }
  if (f.buffer.vehicle_twist_buffer_ms < buffer_lower_bound) {
    r.errors.push_back(
      "buffer.vehicle_twist_buffer_ms(" + std::to_string(f.buffer.vehicle_twist_buffer_ms) +
      ") must be >= " + std::to_string(buffer_lower_bound));
  }

  // --- weights / thresholds -------------------------------------------------
  if (!(cc.w_min > 0.0 && cc.w_min <= 1.0)) {
    r.errors.push_back("covariance_correction.w_min must satisfy 0 < w_min <= 1");
  }
  check_positive(r, "covariance_correction.gate_chi2.dim1", cc.gate_chi2.dim1);
  check_positive(r, "covariance_correction.gate_chi2.dim3", cc.gate_chi2.dim3);
  check_positive(r, "covariance_correction.gate_chi2.dim6", cc.gate_chi2.dim6);
  check_positive(r, "covariance_correction.recovery_gate_scale", cc.recovery_gate_scale);
  check_positive(r, "covariance_correction.low_speed_min_pos_stddev", cc.low_speed_min_pos_stddev);
  check_positive(r, "covariance_correction.low_speed_min_rot_stddev", cc.low_speed_min_rot_stddev);

  // --- output rate ----------------------------------------------------------
  check_positive(r, "output_rate_hz", p.output_rate_hz);
  if (p.output_rate_hz > 0.0 &&
      1000.0 / p.output_rate_hz < static_cast<double>(f.graph1_node_interval_ms)) {
    r.errors.push_back(
      "output_rate_hz is faster than the graph1 node rate: require 1000/output_rate_hz >= "
      "graph1_node_interval_ms");
  }
  check_positive(r, "tf_rate_hz", p.tf_rate_hz);

  // --- noise values ---------------------------------------------------------
  check_positive(r, "imu.accel_noise_stddev", f.imu.accel_noise_stddev);
  check_positive(r, "imu.gyro_noise_stddev", f.imu.gyro_noise_stddev);
  check_positive(r, "imu.accel_bias_rw_stddev", f.imu.accel_bias_rw_stddev);
  check_positive(r, "imu.gyro_bias_rw_stddev", f.imu.gyro_bias_rw_stddev);
  check_positive(r, "imu.integration_cov", f.imu.integration_cov);
  check_positive(r, "imu.gravity", f.imu.gravity);
  check_positive(r, "imu.euler_term_sigma", f.imu.euler_term_sigma);
  check_positive(r, "wheel_speed.fallback_stddev", ws.fallback_stddev);
  check_positive(r, "wheel_speed.nonholonomic_lat_stddev", ws.nonholonomic_lat_stddev);
  check_positive(r, "wheel_speed.nonholonomic_vert_stddev", ws.nonholonomic_vert_stddev);
  check_positive(r, "gnss.fallback_stddev.x", f.gnss.fallback_stddev.x);
  check_positive(r, "gnss.fallback_stddev.y", f.gnss.fallback_stddev.y);
  check_positive(r, "gnss.fallback_stddev.z", f.gnss.fallback_stddev.z);
  check_positive(r, "ndt.fallback_stddev.pos", f.ndt.fallback_pos_stddev);
  check_positive(r, "ndt.fallback_stddev.rot", f.ndt.fallback_rot_stddev);
  check_positive(r, "initialization.init_pose_stddev.pos", init.init_pose_pos_stddev);
  check_positive(r, "initialization.init_pose_stddev.rot", init.init_pose_rot_stddev);
  check_positive(r, "initialization.init_vel_stddev", init.init_vel_stddev);
  check_positive(r, "initialization.init_accel_bias_stddev", init.init_accel_bias_stddev);
  check_positive(r, "initialization.init_gyro_bias_stddev", init.init_gyro_bias_stddev);
  check_positive(r, "initialization.pose_cov_scale", init.pose_cov_scale);
  check_positive(r, "output.undetermined_variance", p.output.undetermined_variance);

  // --- node budget ----------------------------------------------------------
  const int node_lower_bound =
    (f.graph2_max_node_interval_ms > 0 ? f.graph2_lag_ms / f.graph2_max_node_interval_ms : 0) + 10;
  if (f.graph2_max_nodes < node_lower_bound) {
    r.errors.push_back(
      "graph2_max_nodes(" + std::to_string(f.graph2_max_nodes) + ") must be >= " +
      std::to_string(node_lower_bound) +
      " (= graph2_lag_ms / graph2_max_node_interval_ms + 10), otherwise the forced nodes alone "
      "reach the cap.");
  }

  // --- boundary merge noise (D34 / D44) -------------------------------------
  check_positive(r, "graph2.boundary_merge_velocity_mps", g2.boundary_merge_velocity_mps);
  check_positive(r, "graph2.boundary_merge_yaw_rate_rps", g2.boundary_merge_yaw_rate_rps);
  if (f.node_merge_tolerance_ms <= 0) {
    r.errors.push_back("node_merge_tolerance_ms must be > 0");
  }

  // --- wheel speed binding --------------------------------------------------
  if (ws.graph2_binding != "none") {
    if (ws.graph2_max_time_offset_ms < 10) {
      r.errors.push_back(
        "wheel_speed.graph2_max_time_offset_ms must be >= half the wheel speed period "
        "(>= 10 ms for the assumed 50 Hz input)");
    }
    if (ws.graph2_max_time_offset_ms > f.graph2_max_node_interval_ms / 2) {
      r.errors.push_back(
        "wheel_speed.graph2_max_time_offset_ms must be <= graph2_max_node_interval_ms / 2 so a "
        "sample cannot be attributed to the wrong node");
    }
    check_positive(r, "wheel_speed.graph2_time_offset_accel", ws.graph2_time_offset_accel);
  }
  if (ws.timeout_ms <= 20) {
    r.errors.push_back(
      "wheel_speed.timeout_ms must exceed the wheel speed period (> 20 ms at 50 Hz)");
  }

  // --- measurement history cap ---------------------------------------------
  const int assumed_absolute_rate_hz = 30;
  const int history_max_lower_bound =
    static_cast<int>(assumed_absolute_rate_hz * (f.graph2_lag_ms / 1000.0) * 4.0);
  if (g2.measurement_history_max < history_max_lower_bound) {
    r.errors.push_back(
      "graph2.measurement_history_max(" + std::to_string(g2.measurement_history_max) +
      ") must be >= " + std::to_string(history_max_lower_bound));
  }

  // --- smoothing ------------------------------------------------------------
  check_positive(r, "reset.smoothing_max_lin_rate_mps", reset.smoothing_max_lin_rate_mps);
  check_positive(r, "reset.smoothing_max_ang_rate_rps", reset.smoothing_max_ang_rate_rps);
  if (p.output_rate_hz > 0.0 &&
      static_cast<double>(reset.smoothing_max_duration_ms) < 1000.0 / p.output_rate_hz) {
    r.errors.push_back(
      "reset.smoothing_max_duration_ms must cover at least one output step "
      "(>= 1000 / output_rate_hz)");
  }
  if (!(reset.smoothing_done_m > 0.0 && reset.smoothing_done_m < reset.max_anchor_jump_m)) {
    r.errors.push_back("require 0 < reset.smoothing_done_m < reset.max_anchor_jump_m");
  }
  if (!(reset.smoothing_done_rad > 0.0 && reset.smoothing_done_rad < reset.max_anchor_jump_rad)) {
    r.errors.push_back("require 0 < reset.smoothing_done_rad < reset.max_anchor_jump_rad");
  }
  if (!(reset.smoothing_relatch_ratio > 1.0)) {
    r.errors.push_back("reset.smoothing_relatch_ratio must be > 1");
  }
  // D45: strict inequality against the 試験 3 acceptance thresholds.
  if (!(reset.max_anchor_jump_m > 0.0 && reset.max_anchor_jump_m < 0.05)) {
    r.errors.push_back(
      "reset.max_anchor_jump_m must satisfy 0 < x < 0.05 (test 3 allows a 5 cm jump; equality "
      "would make the test fail on the boundary case). Recommended default 0.03.");
  }
  if (!(reset.max_anchor_jump_rad > 0.0 && reset.max_anchor_jump_rad < 0.005)) {
    r.errors.push_back(
      "reset.max_anchor_jump_rad must satisfy 0 < x < 0.005 (test 3 allows 5 mrad). "
      "Recommended default 0.003.");
  }

  // --- cross covariance -----------------------------------------------------
  if (!reset.carry_cross_covariance) {
    if (reset.cross_covariance_inflation < 1.0) {
      r.errors.push_back("reset.cross_covariance_inflation must be >= 1.0");
    } else if (reset.cross_covariance_inflation < 3.0) {
      r.warnings.push_back(
        "reset.carry_cross_covariance=false with cross_covariance_inflation=" +
        std::to_string(reset.cross_covariance_inflation) +
        " < 3.0: dropping the cross correlations is NOT conservative (D33). Only the "
        "Cauchy-Schwarz bound n=3 guarantees conservativeness.");
    }
  }

  // --- SLAM jump detection --------------------------------------------------
  if (f.slam.use) {
    check_positive(r, "slam.max_speed_mps", f.slam.max_speed_mps);
    check_positive(r, "slam.max_yaw_rate_rps", f.slam.max_yaw_rate_rps);
    check_positive(r, "slam.jump_margin_m", f.slam.jump_margin_m);
    check_positive(r, "slam.jump_margin_rad", f.slam.jump_margin_rad);
    check_positive(r, "slam.covariance_scale", f.slam.covariance_scale);
    check_positive(r, "slam.relative_covariance_scale", f.slam.relative_covariance_scale);
  }

  // --- GNSS initialization yaw (D41) ---------------------------------------
  if (init.pose_source == "gnss") {
    check_positive(r, "initialization.gnss_yaw_min_speed_mps", init.gnss_yaw_min_speed_mps);
    check_positive(r, "initialization.gnss_yaw_max_stddev_rad", init.gnss_yaw_max_stddev_rad);
    if (init.gnss_yaw_source == "zero_with_large_covariance") {
      if (!(init.gnss_yaw_fallback_stddev_rad > 0.0 &&
            init.gnss_yaw_fallback_stddev_rad <= 1.5)) {
        r.errors.push_back(
          "initialization.gnss_yaw_fallback_stddev_rad must satisfy 0 < x <= 1.5");
      }
      r.warnings.push_back(
        "initialization.gnss_yaw_source=zero_with_large_covariance: starting with an unknown yaw "
        "and a large variance is a test-only setting (D41). Downstream receives a plausible but "
        "wrong heading.");
    }
  }

  // --- three-layer warning --------------------------------------------------
  if (f.robust_kernel != "none" && cc.enable_gate && cc.enable_weighting) {
    r.warnings.push_back(
      "layers A, B and C are all enabled: the combination over-attenuates absolute pose inputs. "
      "Tune layers A/B first and enable robust_kernel only if still needed (D21).");
  }

  // --- mode consistency -----------------------------------------------------
  if (p.fusion_method == "ekf" && !p.output.flatten_to_2d) {
    r.infos.push_back(
      "fusion_method=ekf ignores output.flatten_to_2d=false and always publishes 2D semantics "
      "(design §3.3 / D20).");
  }

  return r;
}

}  // namespace autoware::pose_twist_fusion
