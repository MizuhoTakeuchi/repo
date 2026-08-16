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

#include "autoware/pose_twist_fusion/parameters.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace autoware::pose_twist_fusion
{

namespace
{
template <typename T>
T declare(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, default_value);
}

Vec3Param declare_vec3(
  rclcpp::Node & node, const std::string & prefix, const Vec3Param & default_value)
{
  Vec3Param v;
  v.x = declare<double>(node, prefix + ".x", default_value.x);
  v.y = declare<double>(node, prefix + ".y", default_value.y);
  v.z = declare<double>(node, prefix + ".z", default_value.z);
  return v;
}

PoseParam declare_pose(
  rclcpp::Node & node, const std::string & prefix, const PoseParam & default_value)
{
  PoseParam v;
  v.x = declare<double>(node, prefix + ".x", default_value.x);
  v.y = declare<double>(node, prefix + ".y", default_value.y);
  v.z = declare<double>(node, prefix + ".z", default_value.z);
  v.roll = declare<double>(node, prefix + ".roll", default_value.roll);
  v.pitch = declare<double>(node, prefix + ".pitch", default_value.pitch);
  v.yaw = declare<double>(node, prefix + ".yaw", default_value.yaw);
  return v;
}

/// Parameters the design retired. They are never declared, so we look at the
/// override list instead - a retired setting must fail the start-up rather than
/// being silently dropped (design §6.2).
std::vector<std::string> find_retired_overrides(const rclcpp::Node & node)
{
  static const std::vector<std::string> kRetired = {
    "factor_graph_fusion.slam.max_relative_jump_m",
    "factor_graph_fusion.slam.max_relative_jump_rad",
    "factor_graph_fusion.graph2.force_boundary_node",
  };
  std::vector<std::string> found;
  for (const auto & override_pair : node.get_node_options().parameter_overrides()) {
    for (const auto & retired : kRetired) {
      if (override_pair.get_name() == retired) {
        // Report the short name used in the design document.
        found.push_back(retired.substr(std::string("factor_graph_fusion.").size()));
      }
    }
  }
  return found;
}
}  // namespace

Parameters declare_parameters(rclcpp::Node & node)
{
  Parameters p;

  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = true;  // D17: switching the algorithm requires a restart.
    desc.description = "ekf | lio_sam_imu_preintegration | factor_graph_fusion";
    if (node.has_parameter("fusion_method")) {
      p.fusion_method = node.get_parameter("fusion_method").as_string();
    } else {
      p.fusion_method = node.declare_parameter<std::string>("fusion_method", p.fusion_method, desc);
    }
  }

  p.output_rate_hz = declare<double>(node, "output_rate_hz", p.output_rate_hz);
  p.tf_rate_hz = declare<double>(node, "tf_rate_hz", p.tf_rate_hz);
  p.enable_tf = declare<bool>(node, "enable_tf", p.enable_tf);
  p.pose_frame_id = declare<std::string>(node, "pose_frame_id", p.pose_frame_id);
  p.base_frame_id = declare<std::string>(node, "base_frame_id", p.base_frame_id);
  p.imu_topic = declare<std::string>(node, "imu_topic", p.imu_topic);
  p.initialpose_topic = declare<std::string>(node, "initialpose_topic", p.initialpose_topic);

  p.output.flatten_to_2d = declare<bool>(node, "output.flatten_to_2d", p.output.flatten_to_2d);
  p.output.max_extrapolation_ms =
    declare<int>(node, "output.max_extrapolation_ms", p.output.max_extrapolation_ms);
  p.output.twist_smoothing_consistency =
    declare<bool>(node, "output.twist_smoothing_consistency", p.output.twist_smoothing_consistency);
  p.output.exact_smoothing_adjoint =
    declare<bool>(node, "output.exact_smoothing_adjoint", p.output.exact_smoothing_adjoint);
  p.output.undetermined_variance =
    declare<double>(node, "output.undetermined_variance", p.output.undetermined_variance);

  p.covariance_use_exact_jacobian =
    declare<bool>(node, "covariance.use_exact_jacobian", p.covariance_use_exact_jacobian);

  p.qos.imu_depth = declare<int>(node, "qos.imu_depth", p.qos.imu_depth);
  p.qos.twist_depth = declare<int>(node, "qos.twist_depth", p.qos.twist_depth);
  p.qos.pose_depth = declare<int>(node, "qos.pose_depth", p.qos.pose_depth);
  p.qos.output_depth = declare<int>(node, "qos.output_depth", p.qos.output_depth);

  // --- factor_graph_fusion --------------------------------------------------
  auto & f = p.factor_graph_fusion;
  const std::string fg = "factor_graph_fusion.";
  f.graph1_node_interval_ms =
    declare<int>(node, fg + "graph1_node_interval_ms", f.graph1_node_interval_ms);
  f.graph2_max_node_interval_ms =
    declare<int>(node, fg + "graph2_max_node_interval_ms", f.graph2_max_node_interval_ms);
  f.graph2_optimize_interval_ms =
    declare<int>(node, fg + "graph2_optimize_interval_ms", f.graph2_optimize_interval_ms);
  f.graph2_lag_ms = declare<int>(node, fg + "graph2_lag_ms", f.graph2_lag_ms);
  f.node_merge_tolerance_ms =
    declare<int>(node, fg + "node_merge_tolerance_ms", f.node_merge_tolerance_ms);
  f.graph2_max_nodes = declare<int>(node, fg + "graph2_max_nodes", f.graph2_max_nodes);

  f.graph2.optimizer = declare<std::string>(node, fg + "graph2.optimizer", f.graph2.optimizer);
  f.graph2.marginalization =
    declare<std::string>(node, fg + "graph2.marginalization", f.graph2.marginalization);
  f.graph2.lag_margin_ms = declare<int>(node, fg + "graph2.lag_margin_ms", f.graph2.lag_margin_ms);
  f.graph2.measurement_history_max =
    declare<int>(node, fg + "graph2.measurement_history_max", f.graph2.measurement_history_max);
  f.graph2.boundary_merge_velocity_mps = declare<double>(
    node, fg + "graph2.boundary_merge_velocity_mps", f.graph2.boundary_merge_velocity_mps);
  f.graph2.boundary_merge_yaw_rate_rps = declare<double>(
    node, fg + "graph2.boundary_merge_yaw_rate_rps", f.graph2.boundary_merge_yaw_rate_rps);

  f.buffer.imu_buffer_ms = declare<int>(node, fg + "buffer.imu_buffer_ms", f.buffer.imu_buffer_ms);
  f.buffer.vehicle_twist_buffer_ms =
    declare<int>(node, fg + "buffer.vehicle_twist_buffer_ms", f.buffer.vehicle_twist_buffer_ms);

  f.timing.history_length_ms =
    declare<int>(node, fg + "timing.history_length_ms", f.timing.history_length_ms);
  f.timing.max_measurement_delay_ms =
    declare<int>(node, fg + "timing.max_measurement_delay_ms", f.timing.max_measurement_delay_ms);
  f.timing.future_tolerance_ms =
    declare<int>(node, fg + "timing.future_tolerance_ms", f.timing.future_tolerance_ms);
  f.timing.exact_interpolation =
    declare<bool>(node, fg + "timing.exact_interpolation", f.timing.exact_interpolation);

  f.reset.carry_cross_covariance =
    declare<bool>(node, fg + "reset.carry_cross_covariance", f.reset.carry_cross_covariance);
  f.reset.cross_covariance_inflation = declare<double>(
    node, fg + "reset.cross_covariance_inflation", f.reset.cross_covariance_inflation);
  f.reset.max_anchor_jump_m =
    declare<double>(node, fg + "reset.max_anchor_jump_m", f.reset.max_anchor_jump_m);
  f.reset.max_anchor_jump_rad =
    declare<double>(node, fg + "reset.max_anchor_jump_rad", f.reset.max_anchor_jump_rad);
  f.reset.smoothing_max_lin_rate_mps = declare<double>(
    node, fg + "reset.smoothing_max_lin_rate_mps", f.reset.smoothing_max_lin_rate_mps);
  f.reset.smoothing_max_ang_rate_rps = declare<double>(
    node, fg + "reset.smoothing_max_ang_rate_rps", f.reset.smoothing_max_ang_rate_rps);
  f.reset.smoothing_max_duration_ms =
    declare<int>(node, fg + "reset.smoothing_max_duration_ms", f.reset.smoothing_max_duration_ms);
  f.reset.smoothing_done_m =
    declare<double>(node, fg + "reset.smoothing_done_m", f.reset.smoothing_done_m);
  f.reset.smoothing_done_rad =
    declare<double>(node, fg + "reset.smoothing_done_rad", f.reset.smoothing_done_rad);
  f.reset.smoothing_relatch_ratio =
    declare<double>(node, fg + "reset.smoothing_relatch_ratio", f.reset.smoothing_relatch_ratio);

  auto & cc = f.covariance_correction;
  const std::string cp = fg + "covariance_correction.";
  cc.enable_gate = declare<bool>(node, cp + "enable_gate", cc.enable_gate);
  cc.enable_weighting = declare<bool>(node, cp + "enable_weighting", cc.enable_weighting);
  cc.weight_function = declare<std::string>(node, cp + "weight_function", cc.weight_function);
  cc.w_min = declare<double>(node, cp + "w_min", cc.w_min);
  cc.gate_chi2.dim1 = declare<double>(node, cp + "gate_chi2.dim1", cc.gate_chi2.dim1);
  cc.gate_chi2.dim3 = declare<double>(node, cp + "gate_chi2.dim3", cc.gate_chi2.dim3);
  cc.gate_chi2.dim6 = declare<double>(node, cp + "gate_chi2.dim6", cc.gate_chi2.dim6);
  cc.low_speed_relax_mps = declare<double>(node, cp + "low_speed_relax_mps", cc.low_speed_relax_mps);
  cc.low_speed_min_pos_stddev =
    declare<double>(node, cp + "low_speed_min_pos_stddev", cc.low_speed_min_pos_stddev);
  cc.low_speed_min_rot_stddev =
    declare<double>(node, cp + "low_speed_min_rot_stddev", cc.low_speed_min_rot_stddev);
  cc.recovery_gate_scale = declare<double>(node, cp + "recovery_gate_scale", cc.recovery_gate_scale);

  f.robust_kernel = declare<std::string>(node, fg + "robust_kernel", f.robust_kernel);
  f.robust_kernel_param = declare<double>(node, fg + "robust_kernel_param", f.robust_kernel_param);

  auto & imu = f.imu;
  const std::string ip = fg + "imu.";
  imu.frame_id = declare<std::string>(node, ip + "frame_id", imu.frame_id);
  imu.use_tf_extrinsic = declare<bool>(node, ip + "use_tf_extrinsic", imu.use_tf_extrinsic);
  imu.extrinsic_fallback = declare_pose(node, ip + "extrinsic_fallback", imu.extrinsic_fallback);
  imu.lever_arm_compensation =
    declare<std::string>(node, ip + "lever_arm_compensation", imu.lever_arm_compensation);
  imu.euler_term_sigma = declare<double>(node, ip + "euler_term_sigma", imu.euler_term_sigma);
  imu.factor_type = declare<std::string>(node, ip + "factor_type", imu.factor_type);
  imu.gravity = declare<double>(node, ip + "gravity", imu.gravity);
  imu.use_orientation = declare<bool>(node, ip + "use_orientation", imu.use_orientation);
  imu.accel_noise_stddev = declare<double>(node, ip + "accel_noise_stddev", imu.accel_noise_stddev);
  imu.gyro_noise_stddev = declare<double>(node, ip + "gyro_noise_stddev", imu.gyro_noise_stddev);
  imu.accel_bias_rw_stddev =
    declare<double>(node, ip + "accel_bias_rw_stddev", imu.accel_bias_rw_stddev);
  imu.gyro_bias_rw_stddev =
    declare<double>(node, ip + "gyro_bias_rw_stddev", imu.gyro_bias_rw_stddev);
  imu.integration_cov = declare<double>(node, ip + "integration_cov", imu.integration_cov);
  imu.initial_bias[0] = declare<double>(node, ip + "initial_bias.ax", imu.initial_bias[0]);
  imu.initial_bias[1] = declare<double>(node, ip + "initial_bias.ay", imu.initial_bias[1]);
  imu.initial_bias[2] = declare<double>(node, ip + "initial_bias.az", imu.initial_bias[2]);
  imu.initial_bias[3] = declare<double>(node, ip + "initial_bias.gx", imu.initial_bias[3]);
  imu.initial_bias[4] = declare<double>(node, ip + "initial_bias.gy", imu.initial_bias[4]);
  imu.initial_bias[5] = declare<double>(node, ip + "initial_bias.gz", imu.initial_bias[5]);

  auto & ws = f.wheel_speed;
  const std::string wp = fg + "wheel_speed.";
  ws.use = declare<bool>(node, wp + "use", ws.use);
  ws.topic = declare<std::string>(node, wp + "topic", ws.topic);
  ws.fallback_stddev = declare<double>(node, wp + "fallback_stddev", ws.fallback_stddev);
  ws.timeout_ms = declare<int>(node, wp + "timeout_ms", ws.timeout_ms);
  ws.graph2_binding = declare<std::string>(node, wp + "graph2_binding", ws.graph2_binding);
  ws.graph2_max_time_offset_ms =
    declare<int>(node, wp + "graph2_max_time_offset_ms", ws.graph2_max_time_offset_ms);
  ws.graph2_time_offset_accel =
    declare<double>(node, wp + "graph2_time_offset_accel", ws.graph2_time_offset_accel);
  ws.use_nonholonomic = declare<bool>(node, wp + "use_nonholonomic", ws.use_nonholonomic);
  ws.nonholonomic_lat_stddev =
    declare<double>(node, wp + "nonholonomic_lat_stddev", ws.nonholonomic_lat_stddev);
  ws.nonholonomic_vert_stddev =
    declare<double>(node, wp + "nonholonomic_vert_stddev", ws.nonholonomic_vert_stddev);
  ws.nonholonomic_min_speed_mps =
    declare<double>(node, wp + "nonholonomic_min_speed_mps", ws.nonholonomic_min_speed_mps);

  auto & gnss = f.gnss;
  const std::string gp = fg + "gnss.";
  gnss.use = declare<bool>(node, gp + "use", gnss.use);
  gnss.topic = declare<std::string>(node, gp + "topic", gnss.topic);
  gnss.frame_id = declare<std::string>(node, gp + "frame_id", gnss.frame_id);
  gnss.use_tf_extrinsic = declare<bool>(node, gp + "use_tf_extrinsic", gnss.use_tf_extrinsic);
  gnss.lever_arm_fallback = declare_vec3(node, gp + "lever_arm_fallback", gnss.lever_arm_fallback);
  gnss.fallback_stddev = declare_vec3(node, gp + "fallback_stddev", gnss.fallback_stddev);
  gnss.timeout_ms = declare<int>(node, gp + "timeout_ms", gnss.timeout_ms);

  auto & ndt = f.ndt;
  const std::string np = fg + "ndt.";
  ndt.use = declare<bool>(node, np + "use", ndt.use);
  ndt.topic = declare<std::string>(node, np + "topic", ndt.topic);
  ndt.fallback_pos_stddev = declare<double>(node, np + "fallback_stddev.pos", ndt.fallback_pos_stddev);
  ndt.fallback_rot_stddev = declare<double>(node, np + "fallback_stddev.rot", ndt.fallback_rot_stddev);
  ndt.timeout_ms = declare<int>(node, np + "timeout_ms", ndt.timeout_ms);

  auto & slam = f.slam;
  const std::string sp = fg + "slam.";
  slam.use = declare<bool>(node, sp + "use", slam.use);
  slam.topic = declare<std::string>(node, sp + "topic", slam.topic);
  slam.use_as = declare<std::string>(node, sp + "use_as", slam.use_as);
  slam.frame_id = declare<std::string>(node, sp + "frame_id", slam.frame_id);
  slam.covariance_is_relative =
    declare<bool>(node, sp + "covariance_is_relative", slam.covariance_is_relative);
  slam.relative_covariance_scale =
    declare<double>(node, sp + "relative_covariance_scale", slam.relative_covariance_scale);
  slam.max_speed_mps = declare<double>(node, sp + "max_speed_mps", slam.max_speed_mps);
  slam.max_yaw_rate_rps = declare<double>(node, sp + "max_yaw_rate_rps", slam.max_yaw_rate_rps);
  slam.jump_margin_m = declare<double>(node, sp + "jump_margin_m", slam.jump_margin_m);
  slam.jump_margin_rad = declare<double>(node, sp + "jump_margin_rad", slam.jump_margin_rad);
  slam.interp_sigma_pos_per_s =
    declare<double>(node, sp + "interp_sigma_per_s.pos", slam.interp_sigma_pos_per_s);
  slam.interp_sigma_rot_per_s =
    declare<double>(node, sp + "interp_sigma_per_s.rot", slam.interp_sigma_rot_per_s);
  slam.covariance_scale = declare<double>(node, sp + "covariance_scale", slam.covariance_scale);
  slam.timeout_ms = declare<int>(node, sp + "timeout_ms", slam.timeout_ms);

  auto & init = f.initialization;
  const std::string np2 = fg + "initialization.";
  init.pose_source = declare<std::string>(node, np2 + "pose_source", init.pose_source);
  init.gnss_yaw_source = declare<std::string>(node, np2 + "gnss_yaw_source", init.gnss_yaw_source);
  init.gnss_yaw_max_stddev_rad =
    declare<double>(node, np2 + "gnss_yaw_max_stddev_rad", init.gnss_yaw_max_stddev_rad);
  init.gnss_yaw_min_speed_mps =
    declare<double>(node, np2 + "gnss_yaw_min_speed_mps", init.gnss_yaw_min_speed_mps);
  init.gnss_yaw_min_duration_s =
    declare<double>(node, np2 + "gnss_yaw_min_duration_s", init.gnss_yaw_min_duration_s);
  init.gnss_yaw_timeout_s =
    declare<double>(node, np2 + "gnss_yaw_timeout_s", init.gnss_yaw_timeout_s);
  init.gnss_yaw_fallback_stddev_rad =
    declare<double>(node, np2 + "gnss_yaw_fallback_stddev_rad", init.gnss_yaw_fallback_stddev_rad);
  init.pose_cov_scale = declare<double>(node, np2 + "pose_cov_scale", init.pose_cov_scale);
  init.init_pose_pos_stddev =
    declare<double>(node, np2 + "init_pose_stddev.pos", init.init_pose_pos_stddev);
  init.init_pose_rot_stddev =
    declare<double>(node, np2 + "init_pose_stddev.rot", init.init_pose_rot_stddev);
  init.init_vel_stddev = declare<double>(node, np2 + "init_vel_stddev", init.init_vel_stddev);
  init.init_accel_bias_stddev =
    declare<double>(node, np2 + "init_accel_bias_stddev", init.init_accel_bias_stddev);
  init.init_gyro_bias_stddev =
    declare<double>(node, np2 + "init_gyro_bias_stddev", init.init_gyro_bias_stddev);
  init.required_imu_samples =
    declare<int>(node, np2 + "required_imu_samples", init.required_imu_samples);
  init.bias_init_static_s = declare<double>(node, np2 + "bias_init_static_s", init.bias_init_static_s);
  init.bias_init_timeout_s =
    declare<double>(node, np2 + "bias_init_timeout_s", init.bias_init_timeout_s);

  auto & fail = f.failure;
  const std::string fp = fg + "failure.";
  fail.max_velocity_mps = declare<double>(node, fp + "max_velocity_mps", fail.max_velocity_mps);
  fail.max_accel_bias = declare<double>(node, fp + "max_accel_bias", fail.max_accel_bias);
  fail.max_gyro_bias = declare<double>(node, fp + "max_gyro_bias", fail.max_gyro_bias);
  fail.optimizer_failure_limit =
    declare<int>(node, fp + "optimizer_failure_limit", fail.optimizer_failure_limit);
  fail.optimizer_failure_reinit_limit =
    declare<int>(node, fp + "optimizer_failure_reinit_limit", fail.optimizer_failure_reinit_limit);
  fail.failure_cov_inflation =
    declare<double>(node, fp + "failure_cov_inflation", fail.failure_cov_inflation);
  fail.imu_timeout_ms = declare<int>(node, fp + "imu_timeout_ms", fail.imu_timeout_ms);
  fail.long_dropout_s = declare<double>(node, fp + "long_dropout_s", fail.long_dropout_s);

  // --- lio_sam --------------------------------------------------------------
  auto & ls = p.lio_sam;
  const std::string lp = "lio_sam.";
  ls.imu_topic = declare<std::string>(node, lp + "imu_topic", ls.imu_topic);
  ls.lidar_odometry_topic =
    declare<std::string>(node, lp + "lidar_odometry_topic", ls.lidar_odometry_topic);
  ls.covariance_source = declare<std::string>(node, lp + "covariance_source", ls.covariance_source);
  ls.fallback_pose_pos_stddev =
    declare<double>(node, lp + "fallback_pose_stddev.pos", ls.fallback_pose_pos_stddev);
  ls.fallback_pose_rot_stddev =
    declare<double>(node, lp + "fallback_pose_stddev.rot", ls.fallback_pose_rot_stddev);
  ls.fallback_twist_lin_stddev =
    declare<double>(node, lp + "fallback_twist_stddev.lin", ls.fallback_twist_lin_stddev);
  ls.fallback_twist_ang_stddev =
    declare<double>(node, lp + "fallback_twist_stddev.ang", ls.fallback_twist_ang_stddev);
  ls.imu_acc_noise = declare<double>(node, lp + "imu_acc_noise", ls.imu_acc_noise);
  ls.imu_gyr_noise = declare<double>(node, lp + "imu_gyr_noise", ls.imu_gyr_noise);
  ls.imu_acc_bias_n = declare<double>(node, lp + "imu_acc_bias_n", ls.imu_acc_bias_n);
  ls.imu_gyr_bias_n = declare<double>(node, lp + "imu_gyr_bias_n", ls.imu_gyr_bias_n);
  ls.imu_gravity = declare<double>(node, lp + "imu_gravity", ls.imu_gravity);
  ls.reset_key_count = declare<int>(node, lp + "reset_key_count", ls.reset_key_count);
  ls.failure_velocity_mps =
    declare<double>(node, lp + "failure_velocity_mps", ls.failure_velocity_mps);
  ls.failure_bias = declare<double>(node, lp + "failure_bias", ls.failure_bias);
  ls.extrinsic_imu_to_base = declare_pose(node, lp + "extrinsic_imu_to_base", ls.extrinsic_imu_to_base);

  // --- ekf ------------------------------------------------------------------
  auto & ekf = p.ekf;
  const std::string ep = "ekf.";
  ekf.input_twist_topic = declare<std::string>(node, ep + "input_twist_topic", ekf.input_twist_topic);
  ekf.input_pose_topic = declare<std::string>(node, ep + "input_pose_topic", ekf.input_pose_topic);
  ekf.enable_yaw_bias_estimation =
    declare<bool>(node, ep + "enable_yaw_bias_estimation", ekf.enable_yaw_bias_estimation);
  ekf.extend_state_step = declare<int>(node, ep + "extend_state_step", ekf.extend_state_step);
  ekf.predict_frequency = declare<double>(node, ep + "predict_frequency", ekf.predict_frequency);
  ekf.pose_additional_delay =
    declare<double>(node, ep + "pose_additional_delay", ekf.pose_additional_delay);
  ekf.pose_gate_dist = declare<double>(node, ep + "pose_gate_dist", ekf.pose_gate_dist);
  ekf.pose_smoothing_steps = declare<double>(node, ep + "pose_smoothing_steps", ekf.pose_smoothing_steps);
  ekf.twist_additional_delay =
    declare<double>(node, ep + "twist_additional_delay", ekf.twist_additional_delay);
  ekf.twist_gate_dist = declare<double>(node, ep + "twist_gate_dist", ekf.twist_gate_dist);
  ekf.twist_smoothing_steps =
    declare<double>(node, ep + "twist_smoothing_steps", ekf.twist_smoothing_steps);
  ekf.proc_stddev_yaw_c = declare<double>(node, ep + "proc_stddev_yaw_c", ekf.proc_stddev_yaw_c);
  ekf.proc_stddev_vx_c = declare<double>(node, ep + "proc_stddev_vx_c", ekf.proc_stddev_vx_c);
  ekf.proc_stddev_wz_c = declare<double>(node, ep + "proc_stddev_wz_c", ekf.proc_stddev_wz_c);
  ekf.z_filter_proc_dev = declare<double>(node, ep + "z_filter_proc_dev", ekf.z_filter_proc_dev);
  ekf.roll_filter_proc_dev =
    declare<double>(node, ep + "roll_filter_proc_dev", ekf.roll_filter_proc_dev);
  ekf.pitch_filter_proc_dev =
    declare<double>(node, ep + "pitch_filter_proc_dev", ekf.pitch_filter_proc_dev);
  ekf.threshold_observable_velocity_mps = declare<double>(
    node, ep + "threshold_observable_velocity_mps", ekf.threshold_observable_velocity_mps);

  p.declared_removed_parameters = find_retired_overrides(node);
  return p;
}

}  // namespace autoware::pose_twist_fusion
