"""
ROS 2 integration tests for Foreman: real node, real status topic, real set_profile
service and action, driven by fake_controller_manager and the real dummy_lifecycle_node.

Uses config/scenario_integration_test.yaml (1 hardware, 1 controller, 1 lifecycle
node; profiles for full and partial functionality).
"""

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from foreman_msgs.action import SetProfile as SetProfileAction
from foreman_msgs.msg import ForemanStatus
from foreman_msgs.srv import SetProfile as SetProfileSrv

ALL_PROFILES = {"active", "all_inactive", "idle", "ros2_control_active", "ros2_control_inactive"}


@pytest.mark.launch_test
def generate_test_description():
    config_path = os.path.join(
        get_package_share_directory("foreman"), "config", "scenario_integration_test.yaml"
    )

    foreman_node = launch_ros.actions.Node(
        package="foreman",
        executable="foreman_node",
        name="foreman",
        parameters=[{"config_path": config_path, "controller_manager": "fake_controller_manager"}],
    )
    fake_controller_manager_node = launch_ros.actions.Node(
        package="foreman",
        executable="fake_controller_manager",
        name="fake_controller_manager",
    )
    dummy_lifecycle_node = launch_ros.actions.Node(
        package="foreman",
        executable="dummy_lifecycle_node",
        name="dummy_lifecycle_node",
    )

    return (
        launch.LaunchDescription(
            [
                foreman_node,
                fake_controller_manager_node,
                dummy_lifecycle_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestForemanIntegration(unittest.TestCase):
    """Drives the real /foreman/set_profile service and action, checks /foreman/status."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_integration_client")
        cls.status = None
        # match /foreman/status's transient-local QoS, or a late-joining subscriber misses it
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        cls.node.create_subscription(ForemanStatus, "/foreman/status", cls._on_status, status_qos)
        cls.set_profile_client = cls.node.create_client(SetProfileSrv, "/foreman/set_profile")
        cls.set_profile_action_client = ActionClient(
            cls.node, SetProfileAction, "/foreman/set_profile"
        )
        # wait for full observation, not just ready -- and clean up on failure,
        # since tearDownClass never runs for a failed setUpClass
        try:
            cls._wait_for(
                lambda: cls.status is not None
                and set(cls.status.available_profiles) == ALL_PROFILES,
                timeout=60.0,
            )
        except Exception:
            cls.set_profile_action_client.destroy()
            cls.node.destroy_node()
            rclpy.shutdown()
            raise

    @classmethod
    def tearDownClass(cls):
        cls.set_profile_action_client.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_status(cls, msg):
        cls.status = msg

    @classmethod
    def _wait_for(cls, predicate, timeout):
        """Spin the client node until predicate() is true, or raise on timeout."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if predicate():
                return
        raise TimeoutError("Condition not met within timeout")

    def _call_set_profile(self, profile_name):
        self.set_profile_client.wait_for_service(timeout_sec=10.0)
        future = self.set_profile_client.call_async(SetProfileSrv.Request(profile=profile_name))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
        return future.result()

    def _send_set_profile_goal(self, profile_name):
        self.set_profile_action_client.wait_for_server(timeout_sec=10.0)
        send_goal_future = self.set_profile_action_client.send_goal_async(
            SetProfileAction.Goal(profile=profile_name)
        )
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=10.0)
        return send_goal_future.result()

    def _get_result(self, goal_handle):
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=15.0)
        return result_future.result().result

    def test_when_node_starts_expect_status_reports_all_profiles_and_ready(self):
        self._wait_for(lambda: bool(self.status.available_profiles), timeout=15.0)
        self.assertTrue(self.status.ready)
        self.assertEqual(set(self.status.all_profiles), ALL_PROFILES)
        self.assertEqual(set(self.status.available_profiles), ALL_PROFILES)

    def test_when_set_profile_action_requests_active_expect_goal_succeeds_and_status_matches(
        self,
    ):
        goal_handle = self._send_set_profile_goal("active")
        self.assertTrue(goal_handle.accepted)

        result = self._get_result(goal_handle)
        self.assertTrue(result.success)
        self._wait_for(
            lambda: self.status.target_profile == "active"
            and self.status.current_profile == "active",
            timeout=5.0,
        )

    def test_when_set_profile_service_requests_a_profile_expect_response_matches_status(self):
        response = self._call_set_profile("all_inactive")

        self.assertTrue(response.success)
        self._wait_for(
            lambda: self.status.target_profile == "all_inactive"
            and self.status.current_profile == "all_inactive",
            timeout=5.0,
        )

    def test_when_second_goal_arrives_while_first_is_active_expect_it_is_rejected(self):
        # start from a profile the busy first goal won't already satisfy
        self._call_set_profile("idle")

        first_handle = self._send_set_profile_goal("all_inactive")
        self.assertTrue(first_handle.accepted)

        # goal acceptance doesn't guarantee _execute() has started -- confirm via
        # status, and specifically a not-yet-reached one: target alone can lag
        # behind a depth-1 topic and land on the already-completed status instead
        self._wait_for(
            lambda: self.status.target_profile == "all_inactive"
            and self.status.current_profile != "all_inactive",
            timeout=5.0,
        )

        # rejected by the shared execution_lock, not preempted
        second_handle = self._send_set_profile_goal("active")
        self.assertTrue(second_handle.accepted)
        second_result = self._get_result(second_handle)
        self.assertFalse(second_result.success)
        self.assertIn("already active", second_result.message)

        first_result = self._get_result(first_handle)
        self.assertTrue(first_result.success)

    def test_when_targeting_ros2_control_active_expect_lifecycle_node_state_ignored(self):
        response = self._call_set_profile("ros2_control_active")

        self.assertTrue(response.success)
        self._wait_for(
            lambda: self.status.target_profile == "ros2_control_active"
            and self.status.current_profile == "ros2_control_active",
            timeout=5.0,
        )
