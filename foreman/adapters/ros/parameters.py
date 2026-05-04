from pathlib import Path
from rclpy.node import Node
from rclpy.parameter import Parameter

from foreman.types import ForemanParameters

class Parameters:
    """ROS2 adapter resolving node parameters"""
    config_path: Path
    def __init__(self, node: Node):
        self._node = node
        # declare on init.
        self._declare_parameters()

    def _declare_parameters(self):
        """Declare the expected ROS 2 parameters"""
        self._node.declare_parameter('config_path', '')

    def load_parameters(self) -> ForemanParameters:
        """
        Reads the ROS parameters and returns the domain-friendly dataclass.
        Raises ValueError or FileNotFoundError if invalid.
        """
        config_path_str = self._node.get_parameter('config_path').value
        
        if not config_path_str:
            raise ValueError(
                "Parameter 'config_path' must be provided. "
                "Pass it via launch file or CLI (--ros-args -p config_path:=/path/to/yaml)."
            )
            
        resolved_path = Path(config_path_str)
        if not resolved_path.exists():
            raise FileNotFoundError(f"Configuration file not found at: {resolved_path.absolute()}")
            
        return ForemanParameters(
            config_path=resolved_path
        )