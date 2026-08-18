import threading
import unittest
from unittest.mock import MagicMock

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup

from foreman.adapters.ros_set_profile_server import RosSetProfileServer
from foreman.types import ErrorSnapshot
from foreman.types import ForemanErrorCategory
from foreman.types import ForemanResponse
from foreman.types import ForemanSnapshot
from foreman.types import OperatingMode
from foreman.types import StopState
from foreman_msgs.srv import SetProfile


def _snapshot(profile="force_ctrl", ready=True, at_profile=False, error=None):
    """Build a ForemanSnapshot with a no-error default."""
    if error is None:
        error = ErrorSnapshot(
            is_error=False, category=ForemanErrorCategory.NONE.value, message="", components=[]
        )
    return ForemanSnapshot(
        profile=profile,
        ready=ready,
        at_profile=at_profile,
        error=error,
        components=[],
        all_profiles=[],
        available_profiles=[],
        operating_mode=OperatingMode.AUTOMATIC.value,
        stop_state=StopState.RUNNING.value,
    )


def _error_snapshot(message="boom"):
    return ErrorSnapshot(
        is_error=True,
        category=ForemanErrorCategory.EXECUTION.value,
        message=message,
        components=["ctrl_a"],
    )


class TestRosSetProfileServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_set_profile_service")
        self.addCleanup(self.node.destroy_node)
        self.engine = MagicMock()

    def _server(self, execution_lock=None):
        if execution_lock is None:
            execution_lock = threading.Lock()
        server = RosSetProfileServer(self.node, self.engine, execution_lock=execution_lock)
        server._poll_period = 0.0
        return server

    def test_service_is_advertised_in_node_namespace(self):
        self._server()
        expected = f"/{self.node.get_name()}/set_profile"
        advertised = dict(self.node.get_service_names_and_types())
        self.assertIn(expected, advertised)
        self.assertEqual(advertised[expected], ["foreman_msgs/srv/SetProfile"])

    def test_shutdown_stops_waiting_for_the_profile(self):
        # Without _shutting_down the wait loop never ends and the process cannot exit.
        self.engine.request_profile.return_value = ForemanResponse(True, "Profile accepted.")
        self.engine.get_engine_snapshot.return_value = _snapshot(at_profile=False)
        server = self._server()
        server.request_shutdown()

        response = server._handle_set_profile(
            SetProfile.Request(profile="force_ctrl"), SetProfile.Response()
        )

        self.assertFalse(response.success)
        self.assertIn("Stopped waiting", response.message)

    def test_service_uses_reentrant_callback_group(self):
        self.assertIsInstance(self._server()._callback_group, ReentrantCallbackGroup)

    def test_rejected_profile_returns_without_waiting(self):
        self.engine.request_profile.return_value = ForemanResponse(
            False, "Profile 'nope' not found in configuration."
        )
        request = SetProfile.Request(profile="nope")
        response = SetProfile.Response()

        response = self._server()._handle_set_profile(request, response)

        self.assertFalse(response.success)
        self.assertIn("not found", response.message)
        self.engine.get_engine_snapshot.assert_not_called()

    def test_busy_set_profile_execution_returns_without_requesting_profile(self):
        execution_lock = threading.Lock()
        execution_lock.acquire()
        request = SetProfile.Request(profile="force_ctrl")
        response = SetProfile.Response()

        response = self._server(execution_lock=execution_lock)._handle_set_profile(
            request, response
        )

        self.assertFalse(response.success)
        self.assertIn("already active", response.message)
        self.engine.request_profile.assert_not_called()
        execution_lock.release()

    def test_succeeds_only_once_engine_reports_at_profile(self):
        self.engine.request_profile.return_value = ForemanResponse(True, "Profile accepted.")
        self.engine.get_engine_snapshot.side_effect = [
            _snapshot(at_profile=False),
            _snapshot(at_profile=False),
            _snapshot(at_profile=True),
        ]
        request = SetProfile.Request(profile="force_ctrl")
        response = SetProfile.Response()

        response = self._server()._handle_set_profile(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.message, "Profile 'force_ctrl' reached.")
        self.assertEqual(self.engine.get_engine_snapshot.call_count, 3)

    def test_successful_profile_releases_execution_lock(self):
        self.engine.request_profile.return_value = ForemanResponse(True, "Profile accepted.")
        self.engine.get_engine_snapshot.return_value = _snapshot(at_profile=True)
        execution_lock = threading.Lock()
        request = SetProfile.Request(profile="force_ctrl")
        response = SetProfile.Response()

        response = self._server(execution_lock=execution_lock)._handle_set_profile(
            request, response
        )

        self.assertTrue(response.success)
        self.assertTrue(execution_lock.acquire(blocking=False))
        execution_lock.release()

    def test_engine_error_returns_failure(self):
        self.engine.request_profile.return_value = ForemanResponse(True, "Profile accepted.")
        self.engine.get_engine_snapshot.side_effect = [
            _snapshot(at_profile=False),
            _snapshot(error=_error_snapshot(message="Service rejected the transition.")),
        ]
        request = SetProfile.Request(profile="force_ctrl")
        response = SetProfile.Response()

        response = self._server()._handle_set_profile(request, response)

        self.assertFalse(response.success)
        self.assertIn("Service rejected the transition.", response.message)

    def test_preempted_profile_returns_failure(self):
        self.engine.request_profile.return_value = ForemanResponse(True, "Profile accepted.")
        self.engine.get_engine_snapshot.return_value = _snapshot(profile="other_profile")
        request = SetProfile.Request(profile="force_ctrl")
        response = SetProfile.Response()

        response = self._server()._handle_set_profile(request, response)

        self.assertFalse(response.success)
        self.assertIn("preempted", response.message)


if __name__ == "__main__":
    unittest.main()
