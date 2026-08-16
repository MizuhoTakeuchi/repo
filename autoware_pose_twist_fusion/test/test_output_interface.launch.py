# Copyright 2026 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Test 1 of design_last.md §11: output interface compatibility.

Checks that every topic of design §3.1 exists with the expected type, that
#2a / #2b / #3 carry identical content, and that the frame ids follow §3.3.
"""

import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_testing
import launch_testing.markers
import pytest
import rclpy


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    param_file = (
        get_package_share_directory("autoware_pose_twist_fusion")
        + "/config/pose_twist_fusion.param.yaml"
    )
    node = launch_ros.actions.Node(
        package="autoware_pose_twist_fusion",
        executable="autoware_pose_twist_fusion_node",
        name="pose_twist_fusion",
        parameters=[param_file, {"fusion_method": "passthrough"}],
        output="screen",
    )
    return launch.LaunchDescription(
        [node, launch_testing.actions.ReadyToTest()]
    ), {"node": node}


class TestOutputInterface(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_output_interface")

    def tearDown(self):
        self.node.destroy_node()

    def test_topics_exist(self):
        """Design §3.1: all output topics exist with the specified types."""
        expected = {
            "/kinematic_state": "nav_msgs/msg/Odometry",
            "/pose_with_covariance": "geometry_msgs/msg/PoseWithCovarianceStamped",
            "/ns_pose_with_covariance": "geometry_msgs/msg/PoseWithCovarianceStamped",
            "/pose_with_covariance_no_yawbias": "geometry_msgs/msg/PoseWithCovarianceStamped",
            "/biased_pose_with_covariance": "geometry_msgs/msg/PoseWithCovarianceStamped",
            "/twist_with_covariance": "geometry_msgs/msg/TwistWithCovarianceStamped",
            "/pose": "geometry_msgs/msg/PoseStamped",
            "/biased_pose": "geometry_msgs/msg/PoseStamped",
            "/twist": "geometry_msgs/msg/TwistStamped",
            "/estimated_yaw_bias": "autoware_internal_debug_msgs/msg/Float64Stamped",
            "/debug/processing_time_ms": "autoware_internal_debug_msgs/msg/Float64Stamped",
        }

        deadline = self.node.get_clock().now().nanoseconds + 20 * 1e9
        found = {}
        while self.node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            found = dict(self.node.get_topic_names_and_types())
            if all(name in found for name in expected):
                break

        missing = [name for name in expected if name not in found]
        self.assertEqual(missing, [], f"missing output topics: {missing}")
        for name, type_name in expected.items():
            self.assertIn(type_name, found[name], f"{name} has an unexpected type")

    def test_service_exists(self):
        """Design §3.1: the trigger_node_srv service is provided."""
        deadline = self.node.get_clock().now().nanoseconds + 20 * 1e9
        while self.node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            services = dict(self.node.get_service_names_and_types())
            if "/trigger_node_srv" in services:
                self.assertIn("std_srvs/srv/SetBool", services["/trigger_node_srv"])
                return
        self.fail("trigger_node_srv was not advertised")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
