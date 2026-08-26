import tempfile
import unittest

import rclpy
from rclpy.parameter import Parameter

from foreman.adapters.ros_node_parameters import RosNodeParameters


class TestRosNodeParameters(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_ros_node_parameters")
        self.addCleanup(self.node.destroy_node)
        # declares config_path ("") and controller_manager ("controller_manager")
        self.parameters = RosNodeParameters(self.node)

    def test_when_config_path_missing_expect_value_error(self):
        with self.assertRaises(ValueError):
            self.parameters.load_parameters()

    def test_when_config_path_does_not_exist_expect_file_not_found_error(self):
        self.node.set_parameters([Parameter("config_path", value="/no/such/file.yaml")])

        with self.assertRaises(FileNotFoundError):
            self.parameters.load_parameters()

    def test_when_config_path_valid_expect_parameters_returned(self):
        with tempfile.NamedTemporaryFile(suffix=".yaml") as f:
            self.node.set_parameters(
                [
                    Parameter("config_path", value=f.name),
                    Parameter("controller_manager", value="my_cm"),
                ]
            )

            result = self.parameters.load_parameters()

            self.assertEqual(str(result.config_path), f.name)
            self.assertEqual(result.controller_manager, "my_cm")

    def test_when_controller_manager_not_set_expect_default(self):
        with tempfile.NamedTemporaryFile(suffix=".yaml") as f:
            self.node.set_parameters([Parameter("config_path", value=f.name)])

            result = self.parameters.load_parameters()

            self.assertEqual(result.controller_manager, "controller_manager")


if __name__ == "__main__":
    unittest.main()
