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

#ifndef AUTOWARE__POSE_TWIST_FUSION__PARAMETERS_HPP_
#define AUTOWARE__POSE_TWIST_FUSION__PARAMETERS_HPP_

#include <array>
#include <string>
#include <vector>

namespace rclcpp
{
class Node;
}

namespace autoware::pose_twist_fusion
{

/// All parameter defaults follow design_last.md §6.1. They are "implementation
/// mandatory defaults" (L2): the node must start and converge with them alone.
struct Vec3Param
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct PoseParam
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

struct OutputParams
{
  bool flatten_to_2d{false};
  int max_extrapolation_ms{100};
  bool twist_smoothing_consistency{true};
  bool exact_smoothing_adjoint{false};
  double undetermined_variance{1.0e4};
};

struct QosParams
{
  int imu_depth{100};
  int twist_depth{100};
  int pose_depth{10};
  int output_depth{1};
};

struct Graph2Params
{
  std::string optimizer{"levenberg_marquardt"};
  std::string marginalization{"linear_container"};
  int lag_margin_ms{100};
  int measurement_history_max{512};
  double boundary_merge_velocity_mps{20.0};
  double boundary_merge_yaw_rate_rps{1.0};
};

struct BufferParams
{
  int imu_buffer_ms{3500};
  int vehicle_twist_buffer_ms{3500};
};

struct TimingParams
{
  int history_length_ms{1500};
  int max_measurement_delay_ms{300};
  int future_tolerance_ms{20};
  bool exact_interpolation{false};
};

struct ResetParams
{
  bool carry_cross_covariance{true};
  double cross_covariance_inflation{3.0};
  double max_anchor_jump_m{0.03};
  double max_anchor_jump_rad{0.003};
  double smoothing_max_lin_rate_mps{1.0};
  double smoothing_max_ang_rate_rps{0.2};
  int smoothing_max_duration_ms{1000};
  double smoothing_done_m{0.005};
  double smoothing_done_rad{0.0005};
  double smoothing_relatch_ratio{2.0};
};

struct GateChi2
{
  double dim1{10.83};
  double dim3{16.27};
  double dim6{22.46};
};

struct CovarianceCorrectionParams
{
  bool enable_gate{true};
  bool enable_weighting{true};
  std::string weight_function{"exp_excess_nis"};
  double w_min{0.05};
  GateChi2 gate_chi2;
  double low_speed_relax_mps{0.5};
  double low_speed_min_pos_stddev{0.10};
  double low_speed_min_rot_stddev{0.01};
  double recovery_gate_scale{10.0};
};

struct ImuParams
{
  std::string frame_id{"imu_link"};
  bool use_tf_extrinsic{true};
  PoseParam extrinsic_fallback;
  std::string lever_arm_compensation{"centrifugal"};
  double euler_term_sigma{0.02};
  std::string factor_type{"imu_factor"};
  double gravity{9.80665};
  bool use_orientation{false};
  double accel_noise_stddev{3.9e-3};
  double gyro_noise_stddev{1.5e-3};
  double accel_bias_rw_stddev{6.4e-4};
  double gyro_bias_rw_stddev{3.5e-5};
  double integration_cov{1.0e-8};
  std::array<double, 6> initial_bias{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

struct WheelSpeedParams
{
  bool use{true};
  std::string topic{"/sensing/vehicle_velocity_converter/twist_with_covariance"};
  double fallback_stddev{0.2};
  int timeout_ms{200};
  std::string graph2_binding{"nearest"};
  int graph2_max_time_offset_ms{30};
  double graph2_time_offset_accel{3.0};
  bool use_nonholonomic{true};
  double nonholonomic_lat_stddev{0.1};
  double nonholonomic_vert_stddev{0.05};
  double nonholonomic_min_speed_mps{0.5};
};

struct GnssParams
{
  bool use{true};
  std::string topic{"/sensing/gnss/pose_with_covariance"};
  std::string frame_id{"gnss_link"};
  bool use_tf_extrinsic{true};
  Vec3Param lever_arm_fallback;
  Vec3Param fallback_stddev{1.0, 1.0, 3.0};
  int timeout_ms{2000};
};

struct NdtParams
{
  bool use{true};
  std::string topic{"/localization/pose_estimator/pose_with_covariance"};
  double fallback_pos_stddev{0.2};
  double fallback_rot_stddev{0.02};
  int timeout_ms{1000};
};

struct SlamParams
{
  bool use{true};
  std::string topic{"lio_sam/mapping/odometry"};
  std::string use_as{"relative"};
  std::string frame_id{"map"};
  bool covariance_is_relative{false};
  double relative_covariance_scale{1.0};
  double max_speed_mps{20.0};
  double max_yaw_rate_rps{1.0};
  double jump_margin_m{0.30};
  double jump_margin_rad{0.05};
  double interp_sigma_pos_per_s{0.05};
  double interp_sigma_rot_per_s{0.01};
  double covariance_scale{3.0};
  int timeout_ms{1000};
};

struct InitializationParams
{
  std::string pose_source{"initialpose"};
  std::string gnss_yaw_source{"auto"};
  double gnss_yaw_max_stddev_rad{0.2};
  double gnss_yaw_min_speed_mps{2.0};
  double gnss_yaw_min_duration_s{1.0};
  double gnss_yaw_timeout_s{30.0};
  double gnss_yaw_fallback_stddev_rad{1.0};
  double pose_cov_scale{4.0};
  double init_pose_pos_stddev{1.0};
  double init_pose_rot_stddev{0.1};
  double init_vel_stddev{0.5};
  double init_accel_bias_stddev{0.1};
  double init_gyro_bias_stddev{0.01};
  int required_imu_samples{20};
  double bias_init_static_s{1.0};
  double bias_init_timeout_s{2.0};
};

struct FailureParams
{
  double max_velocity_mps{40.0};
  double max_accel_bias{1.0};
  double max_gyro_bias{0.5};
  int optimizer_failure_limit{3};
  int optimizer_failure_reinit_limit{10};
  double failure_cov_inflation{10.0};
  int imu_timeout_ms{100};
  double long_dropout_s{5.0};
};

struct FactorGraphParams
{
  int graph1_node_interval_ms{20};
  int graph2_max_node_interval_ms{100};
  int graph2_optimize_interval_ms{500};
  int graph2_lag_ms{1000};
  int node_merge_tolerance_ms{5};
  int graph2_max_nodes{80};

  Graph2Params graph2;
  BufferParams buffer;
  TimingParams timing;
  ResetParams reset;
  CovarianceCorrectionParams covariance_correction;
  std::string robust_kernel{"none"};
  double robust_kernel_param{1.345};
  ImuParams imu;
  WheelSpeedParams wheel_speed;
  GnssParams gnss;
  NdtParams ndt;
  SlamParams slam;
  InitializationParams initialization;
  FailureParams failure;
};

struct LioSamParams
{
  std::string imu_topic{"/sensing/imu/imu_data"};
  std::string lidar_odometry_topic{"lio_sam/mapping/odometry"};
  std::string covariance_source{"propagated"};
  double fallback_pose_pos_stddev{0.3};
  double fallback_pose_rot_stddev{0.03};
  double fallback_twist_lin_stddev{0.3};
  double fallback_twist_ang_stddev{0.03};
  // imuPreintegration.cpp parameters (original implementation defaults)
  double imu_acc_noise{3.9939570888238808e-03};
  double imu_gyr_noise{1.5636343949698187e-03};
  double imu_acc_bias_n{6.4356659353532566e-05};
  double imu_gyr_bias_n{3.5640318696367613e-05};
  double imu_gravity{9.80511};
  int reset_key_count{100};
  double failure_velocity_mps{30.0};
  double failure_bias{1.0};
  PoseParam extrinsic_imu_to_base;
};

struct EkfParams
{
  std::string input_twist_topic{"/localization/twist_estimator/twist_with_covariance"};
  std::string input_pose_topic{"/localization/pose_estimator/pose_with_covariance"};
  bool enable_yaw_bias_estimation{true};
  int extend_state_step{50};
  double predict_frequency{50.0};
  // pose measurement
  double pose_additional_delay{0.0};
  double pose_gate_dist{10.0};
  double pose_smoothing_steps{5};
  // twist measurement
  double twist_additional_delay{0.0};
  double twist_gate_dist{10.0};
  double twist_smoothing_steps{2};
  // process noise
  double proc_stddev_yaw_c{0.005};
  double proc_stddev_vx_c{10.0};
  double proc_stddev_wz_c{5.0};
  // simple 1d filter
  double z_filter_proc_dev{1.0};
  double roll_filter_proc_dev{0.01};
  double pitch_filter_proc_dev{0.01};
  double threshold_observable_velocity_mps{0.5};
};

struct Parameters
{
  std::string fusion_method{"factor_graph_fusion"};
  double output_rate_hz{50.0};
  double tf_rate_hz{50.0};
  bool enable_tf{true};
  std::string pose_frame_id{"map"};
  std::string base_frame_id{"base_link"};
  std::string imu_topic{"/sensing/imu/imu_data"};
  std::string initialpose_topic{"initialpose"};

  OutputParams output;
  bool covariance_use_exact_jacobian{false};
  QosParams qos;
  FactorGraphParams factor_graph_fusion;
  LioSamParams lio_sam;
  EkfParams ekf;

  /// Names of parameters that the design retired (design §6.2 "廃止パラメータ").
  /// Filled by the loader when the user set one of them; validation turns this
  /// into a start-up failure so that the setting is never silently ignored.
  std::vector<std::string> declared_removed_parameters;
};

/// Declares every parameter on the node and returns the populated struct.
/// `fusion_method` is declared read-only (D17).
Parameters declare_parameters(rclcpp::Node & node);

}  // namespace autoware::pose_twist_fusion

#endif  // AUTOWARE__POSE_TWIST_FUSION__PARAMETERS_HPP_
