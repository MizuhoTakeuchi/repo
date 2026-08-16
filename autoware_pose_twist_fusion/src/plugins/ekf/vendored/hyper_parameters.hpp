// Copyright 2022 Autoware Foundation
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
// Vendored from autoware_ekf_localizer (src/include/hyper_parameters.hpp).
// The only change is that the parameters are passed in as plain values instead of
// being declared on a node: this package declares them under its own `ekf.*`
// namespace (design §6.1) and must not collide with the original node's names.

#ifndef EKF_VENDORED__HYPER_PARAMETERS_HPP_
#define EKF_VENDORED__HYPER_PARAMETERS_HPP_

#include <algorithm>
#include <cstddef>
#include <string>

namespace autoware::pose_twist_fusion::ekf_vendored
{

struct HyperParameters
{
  bool show_debug_info{false};
  double ekf_rate{50.0};
  double ekf_dt{1.0 / 50.0};
  double tf_rate_{50.0};
  bool enable_yaw_bias_estimation{true};
  size_t extend_state_step{50};
  std::string pose_frame_id{"map"};
  double pose_additional_delay{0.0};
  double pose_gate_dist{10.0};
  size_t pose_smoothing_steps{5};
  size_t max_pose_queue_size{5};
  double twist_additional_delay{0.0};
  double twist_gate_dist{10.0};
  size_t twist_smoothing_steps{2};
  size_t max_twist_queue_size{5};
  double proc_stddev_vx_c{10.0};
  double proc_stddev_wz_c{5.0};
  double proc_stddev_yaw_c{0.005};
  double z_filter_proc_dev{1.0};
  double roll_filter_proc_dev{0.01};
  double pitch_filter_proc_dev{0.01};
  size_t pose_no_update_count_threshold_warn{50};
  size_t pose_no_update_count_threshold_error{250};
  size_t twist_no_update_count_threshold_warn{50};
  size_t twist_no_update_count_threshold_error{250};
  double ellipse_scale{3.0};
  double error_ellipse_size{1.5};
  double warn_ellipse_size{1.2};
  double error_ellipse_size_lateral_direction{0.3};
  double warn_ellipse_size_lateral_direction{0.25};
  double diagnostics_publish_frequency{10.0};
  double diagnostics_publish_period{0.1};
  double threshold_observable_velocity_mps{0.5};
};

}  // namespace autoware::pose_twist_fusion::ekf_vendored

#endif  // EKF_VENDORED__HYPER_PARAMETERS_HPP_
