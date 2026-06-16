from comm.datalayer import NodeClass
import ctrlxdatalayer
from ctrlxdatalayer.metadata_utils import AllowedOperation
from ctrlxdatalayer.metadata_utils import MetadataBuilder
from ctrlxdatalayer.metadata_utils import ReferenceType
from ctrlxdatalayer.provider import Provider
from ctrlxdatalayer.provider_node import NodeCallback
from ctrlxdatalayer.variant import Result
from ctrlxdatalayer.variant import Variant

from foreman.adapters.datalayer.ctrlx_node_provider_base import CtrlXNodeProviderBase
from foreman.engine import ForemanEngine


class CtrlXNodeSetGoal(CtrlXNodeProviderBase):
    """Write-only node that receives target goals and forwards them to the Foreman Engine."""

    def __init__(self, provider: Provider, address: str, engine: ForemanEngine, ros_logger):
        self.engine = engine
        self.ros_logger = ros_logger

        b = MetadataBuilder(allowed=AllowedOperation.WRITE)
        b = b.set_display_name("Foreman Set Goal")
        b = b.set_node_class(NodeClass.NodeClass.Variable)
        b.add_reference(ReferenceType.write(), "types/datalayer/string")
        metadata = b.build()

        super().__init__(provider, address, metadata)

    def _on_write(self, userdata, address: str, data: Variant, cb: NodeCallback):
        """Request goal from Foreman Engine."""
        try:
            if data.get_type() != ctrlxdatalayer.variant.VariantType.STRING:
                cb(Result.TYPE_MISMATCH, data)
                return

            requested_goal = data.get_string()
            self.ros_logger.info(f"UI requested goal: '{requested_goal}' via Data Layer")

            response = self.engine.request_goal(requested_goal)

            if not response.success:
                self.ros_logger.warn(f"Goal rejected: {response.message}")
                cb(Result.INVALID_VALUE, data)
            else:
                cb(Result.OK, data)
        except Exception:
            cb(Result.FAILED, data)
