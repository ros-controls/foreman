import os
import tempfile
import unittest

import rclpy
from rclpy.parameter import Parameter

from foreman.adapters.ros_node_parameters import RosNodeParameters


class TestRosNodeParameters(unittest.TestCase):
    """Tests for the controller_manager node parameter via RosNodeParameters."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # load_parameters() requires config_path to point to an existing file.
        # The contents are irrelevant here; only the file's existence is checked.
        temp_config = tempfile.NamedTemporaryFile(suffix=".yaml", delete=False)
        temp_config.write(b"controllers: {}\n")
        temp_config.close()

        self.config_path = temp_config.name

        # Remove the file after the test.
        self.addCleanup(os.unlink, self.config_path)

    def make_node(self, node_name, controller_manager=None):
        # parameter_overrides is the test equivalent of:
        # --ros-args -p name:=value
        overrides = [Parameter("config_path", value=self.config_path)]

        if controller_manager is not None:
            overrides.append(
                Parameter(
                    "controller_manager",
                    value=controller_manager,
                )
            )

        node = rclpy.create_node(
            node_name,
            parameter_overrides=overrides,
        )

        self.addCleanup(node.destroy_node)

        return node

    def test_default_controller_manager(self):
        # No override, the declared default is used.
        node = self.make_node(
            "test_default_controller_manager"
        )

        params = RosNodeParameters(node).load_parameters()

        self.assertEqual(
            params.controller_manager,
            "controller_manager",
        )

    def test_controller_manager_with_namespace(self):
        # A namespaced controller manager should be returned as provided.
        node = self.make_node(
            "test_controller_manager_with_namespace",
            controller_manager="robot/controller_manager",
        )

        params = RosNodeParameters(node).load_parameters()

        self.assertEqual(
            params.controller_manager,
            "robot/controller_manager",
        )


if __name__ == "__main__":
    unittest.main()
