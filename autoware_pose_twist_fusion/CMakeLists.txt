cmake_minimum_required(VERSION 3.14)
project(autoware_pose_twist_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(GTSAM 4 REQUIRED)
include(CheckCXXSourceCompiles)

# ---------------------------------------------------------------------------
# GTSAM build-flag verification and feature detection (implementation_plan §2)
# ---------------------------------------------------------------------------
set(CMAKE_REQUIRED_INCLUDES ${GTSAM_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})
set(CMAKE_REQUIRED_FLAGS "-std=c++17")

# 1) GTSAM_POSE3_EXPMAP / GTSAM_ROT3_EXPMAP are a hard requirement: without them
#    Pose3::retract() is not the SE(3) Expmap and the covariance convention of
#    design §5.3.5 (right perturbation, body tangent) silently breaks.
check_cxx_source_compiles("
#include <gtsam/config.h>
#if !defined(GTSAM_POSE3_EXPMAP) || !defined(GTSAM_ROT3_EXPMAP)
#error \"expmap disabled\"
#endif
int main() { return 0; }
" GTSAM_HAS_EXPMAP_CONVENTION)
if(NOT GTSAM_HAS_EXPMAP_CONVENTION)
  message(FATAL_ERROR
    "GTSAM was built without GTSAM_POSE3_EXPMAP / GTSAM_ROT3_EXPMAP. "
    "autoware_pose_twist_fusion requires the full SE(3) Expmap retraction "
    "(design_last.md §5.3.5 / §10). Rebuild GTSAM with those options ON.")
endif()

# 2) Lever-arm aware GPS factor (gtsam::GPSFactorArm, GTSAM >= 4.2).
check_cxx_source_compiles("
#include <gtsam/navigation/GPSFactor.h>
int main() { gtsam::GPSFactorArm * p = nullptr; (void)p; return 0; }
" GNSS_FACTOR_USE_GTSAM_ARM)

# 3) PreintegratedImuMeasurements::predict() with Jacobian arguments (D39).
check_cxx_source_compiles("
#include <gtsam/navigation/ImuFactor.h>
int main() {
gtsam::PreintegrationParams::shared_ptr p = gtsam::PreintegrationParams::MakeSharedU(9.81);
gtsam::PreintegratedImuMeasurements pim(p);
gtsam::Matrix99 H1;
gtsam::Matrix96 H2;
pim.predict(gtsam::NavState(), gtsam::imuBias::ConstantBias(), H1, H2);
return 0;
}
" LIO_SAM_PREDICT_JACOBIAN)

# 4) Pose3::LogmapDerivative argument type (Vector6 or Pose3) - needed by the
#    smoothing covariance transport J_r^-1(-c) (D40).
check_cxx_source_compiles("
#include <gtsam/geometry/Pose3.h>
int main() {
gtsam::Vector6 c = gtsam::Vector6::Zero();
gtsam::Matrix6 m = gtsam::Pose3::LogmapDerivative(c);
(void)m;
return 0;
}
" SMOOTHING_LOGMAP_DERIVATIVE_ARG_VECTOR)
check_cxx_source_compiles("
#include <gtsam/geometry/Pose3.h>
int main() {
gtsam::Matrix6 m = gtsam::Pose3::LogmapDerivative(gtsam::Pose3());
(void)m;
return 0;
}
" SMOOTHING_LOGMAP_DERIVATIVE_ARG_POSE)
if(NOT SMOOTHING_LOGMAP_DERIVATIVE_ARG_VECTOR AND NOT SMOOTHING_LOGMAP_DERIVATIVE_ARG_POSE)
  message(STATUS
    "gtsam::Pose3::LogmapDerivative not usable - falling back to the closed-form "
    "right-Jacobian implementation in covariance_convert.cpp")
endif()
unset(CMAKE_REQUIRED_INCLUDES)
unset(CMAKE_REQUIRED_FLAGS)

set(PTF_COMPILE_DEFS "")
if(GNSS_FACTOR_USE_GTSAM_ARM)
  list(APPEND PTF_COMPILE_DEFS PTF_GNSS_FACTOR_USE_GTSAM_ARM=1)
endif()
if(LIO_SAM_PREDICT_JACOBIAN)
  list(APPEND PTF_COMPILE_DEFS PTF_LIO_SAM_PREDICT_JACOBIAN=1)
endif()
if(SMOOTHING_LOGMAP_DERIVATIVE_ARG_POSE)
  list(APPEND PTF_COMPILE_DEFS PTF_SMOOTHING_LOGMAP_DERIVATIVE_ARG_POSE=1)
elseif(SMOOTHING_LOGMAP_DERIVATIVE_ARG_VECTOR)
  list(APPEND PTF_COMPILE_DEFS PTF_SMOOTHING_LOGMAP_DERIVATIVE_ARG_VECTOR=1)
endif()

ament_auto_find_build_dependencies()

# ---------------------------------------------------------------------------
# core library (node + output stage + plugins)
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# core library: the node, the output stage and everything mode independent
# ---------------------------------------------------------------------------
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/parameters.cpp
  src/covariance_convert.cpp
  src/output_stage.cpp
  src/output_publisher.cpp
  src/param_validator.cpp
  src/diagnostics_builder.cpp
  src/pose_twist_fusion_node.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
  SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${PTF_COMPILE_DEFS})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen gtsam)

# ---------------------------------------------------------------------------
# plugin library
#
# The plugins live in their own shared library on purpose: class_loader can only
# associate the registered factories with a library that *it* opened, so a plugin
# library must never be linked into the executable directly.
# ---------------------------------------------------------------------------
ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  src/plugins/passthrough_fusion_plugin.cpp
  src/plugins/ekf/ekf_fusion_plugin.cpp
  src/plugins/ekf/vendored/ekf_module.cpp
  src/plugins/ekf/vendored/covariance.cpp
  src/plugins/ekf/vendored/mahalanobis.cpp
  src/plugins/ekf/vendored/measurement.cpp
  src/plugins/ekf/vendored/state_transition.cpp
  src/plugins/ekf/vendored/warning_message.cpp
  src/plugins/lio_sam/lio_sam_covariance.cpp
  src/plugins/lio_sam/lio_sam_imu_preintegration_plugin.cpp
  src/plugins/factor_graph_fusion/factor_graph_fusion_plugin.cpp
  src/plugins/factor_graph_fusion/imu_frame_adapter.cpp
  src/plugins/factor_graph_fusion/imu_buffer.cpp
  src/plugins/factor_graph_fusion/vehicle_twist_buffer.cpp
  src/plugins/factor_graph_fusion/state_history.cpp
  src/plugins/factor_graph_fusion/graph1_propagator.cpp
  src/plugins/factor_graph_fusion/measurement_queue.cpp
  src/plugins/factor_graph_fusion/measurement_history.cpp
  src/plugins/factor_graph_fusion/graph2_window.cpp
  src/plugins/factor_graph_fusion/graph2_optimizer.cpp
  src/plugins/factor_graph_fusion/graph2_marginalizer.cpp
  src/plugins/factor_graph_fusion/anchor_manager.cpp
  src/plugins/factor_graph_fusion/covariance_corrector.cpp
  src/plugins/factor_graph_fusion/slam_pair_gate.cpp
  src/plugins/factor_graph_fusion/initializer.cpp
  src/plugins/factor_graph_fusion/factors/wheel_speed_factor.cpp
  src/plugins/factor_graph_fusion/factors/nonholonomic_factor.cpp
  src/plugins/factor_graph_fusion/factors/gnss_lever_arm_factor.cpp
  src/plugins/factor_graph_fusion/factors/factor_builders.cpp
)

target_include_directories(${PROJECT_NAME}_plugins
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})
target_compile_definitions(${PROJECT_NAME}_plugins PUBLIC ${PTF_COMPILE_DEFS})
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME} Eigen3::Eigen gtsam)

pluginlib_export_plugin_description_file(autoware_pose_twist_fusion plugins.xml)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::pose_twist_fusion::PoseTwistFusionNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB PTF_TEST_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  foreach(filepath ${PTF_TEST_FILES})
    get_filename_component(test_name ${filepath} NAME_WE)
    ament_add_gtest(${test_name} ${filepath})
    target_include_directories(${test_name}
      PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
      SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})
    target_link_libraries(${test_name} ${PROJECT_NAME}_plugins ${PROJECT_NAME} Eigen3::Eigen gtsam)
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endforeach()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_output_interface.launch.py TIMEOUT "60")
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
  schema
)
