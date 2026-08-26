import unittest
from unittest.mock import MagicMock

import rclpy

from foreman.adapters.autostart_adapter import AutostartAdapter
from foreman.types import (
    Component,
    ComponentType,
    ErrorSnapshot,
    ForemanErrorCategory,
    ForemanResponse,
    ForemanSnapshot,
    LifecycleState,
)


def _component(name, state=LifecycleState.UNCONFIGURED):
    return Component(name=name, component_type=ComponentType.HARDWARE, lifecycle_state=state)


def _snapshot(components=None, is_error=False):
    """Build a ForemanSnapshot with just the fields AutostartAdapter reads."""
    return ForemanSnapshot(
        target_profile="None",
        current_profile="None",
        ready=True,
        error=ErrorSnapshot(
            is_error=is_error,
            category=(
                ForemanErrorCategory.EXECUTION.value
                if is_error
                else ForemanErrorCategory.NONE.value
            ),
            message="boom" if is_error else "",
            components=[],
        ),
        components=components if components is not None else [],
        all_profiles=[],
        available_profiles=[],
    )


class TestAutostartAdapter(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_autostart_adapter")
        self.addCleanup(self.node.destroy_node)
        self.engine = MagicMock()
        self.engine._config.tracked_components = {"hw1"}

    def _adapter(self, autostart=True):
        return AutostartAdapter(self.node, self.engine, "active", autostart=autostart)

    def test_when_autostart_disabled_expect_no_profile_requested(self):
        adapter = self._adapter(autostart=False)

        adapter.autostart()

        self.engine.request_profile.assert_not_called()

    def test_when_previous_transition_errored_expect_state_reset_for_retry(self):
        adapter = self._adapter()
        adapter.transition_success = True
        adapter._stable_ticks = 10
        adapter._last_observed_states = {"hw1": LifecycleState.ACTIVE}
        self.engine.get_engine_snapshot.return_value = _snapshot(is_error=True)

        adapter.autostart()

        self.assertFalse(adapter.transition_success)
        self.assertEqual(adapter._stable_ticks, 0)
        self.assertIsNone(adapter._last_observed_states)
        self.engine.request_profile.assert_not_called()

    def test_when_a_tracked_component_is_missing_expect_autostart_waits(self):
        adapter = self._adapter()
        adapter._stable_ticks = 5
        self.engine.get_engine_snapshot.return_value = _snapshot(components=[])

        adapter.autostart()

        self.assertEqual(adapter._stable_ticks, 0)
        self.engine.request_profile.assert_not_called()

    def test_when_state_keeps_changing_expect_autostart_never_requests(self):
        adapter = self._adapter()
        for state in [LifecycleState.UNCONFIGURED, LifecycleState.INACTIVE, LifecycleState.ACTIVE]:
            self.engine.get_engine_snapshot.return_value = _snapshot(
                components=[_component("hw1", state)]
            )
            adapter.autostart()

        self.engine.request_profile.assert_not_called()

    def test_when_state_stable_for_required_ticks_expect_profile_requested(self):
        adapter = self._adapter()
        self.engine.get_engine_snapshot.return_value = _snapshot(
            components=[_component("hw1", LifecycleState.UNCONFIGURED)]
        )
        self.engine.request_profile.return_value = ForemanResponse(True, "accepted")

        for _ in range(AutostartAdapter.STABLE_TICKS_REQUIRED + 1):
            adapter.autostart()

        self.engine.request_profile.assert_called_once_with("active")
        self.assertTrue(adapter.transition_success)

    def test_when_send_profile_request_succeeds_expect_true(self):
        adapter = self._adapter()
        self.engine.request_profile.return_value = ForemanResponse(True, "accepted")

        self.assertTrue(adapter.send_profile_request())

    def test_when_send_profile_request_fails_expect_false(self):
        adapter = self._adapter()
        self.engine.request_profile.return_value = ForemanResponse(False, "not found")

        self.assertFalse(adapter.send_profile_request())

    def test_when_transition_not_yet_successful_expect_is_done_false(self):
        adapter = self._adapter()

        self.assertFalse(adapter.is_done)

    def test_when_transition_successful_and_no_error_expect_is_done_true(self):
        adapter = self._adapter()
        adapter.transition_success = True
        self.engine.get_engine_snapshot.return_value = _snapshot(is_error=False)

        self.assertTrue(adapter.is_done)

    def test_when_transition_successful_but_engine_errored_expect_is_done_false(self):
        adapter = self._adapter()
        adapter.transition_success = True
        self.engine.get_engine_snapshot.return_value = _snapshot(is_error=True)

        self.assertFalse(adapter.is_done)

    def test_when_all_tracked_components_present_and_ready_expect_all_components_ready_true(self):
        adapter = self._adapter()
        self.engine.get_engine_snapshot.return_value = _snapshot(
            components=[_component("hw1", LifecycleState.INACTIVE)]
        )

        self.assertTrue(adapter.all_components_ready())

    def test_when_a_tracked_component_is_missing_expect_all_components_ready_false(self):
        adapter = self._adapter()
        self.engine.get_engine_snapshot.return_value = _snapshot(components=[])

        self.assertFalse(adapter.all_components_ready())

    def test_when_a_component_is_missing_expect_not_ready_components_lists_it(self):
        adapter = self._adapter()
        self.engine._config.tracked_components = {"hw1", "hw2"}
        self.engine.get_engine_snapshot.return_value = _snapshot(
            components=[_component("hw1", LifecycleState.INACTIVE)]
        )

        self.assertEqual(adapter._get_not_ready_components(), ["hw2"])
