from comm.datalayer import NodeClass
from ctrlxdatalayer.metadata_utils import AllowedOperation
from ctrlxdatalayer.metadata_utils import MetadataBuilder
from ctrlxdatalayer.metadata_utils import ReferenceType
from ctrlxdatalayer.provider import Provider
from ctrlxdatalayer.provider_node import NodeCallback
from ctrlxdatalayer.variant import Result
from ctrlxdatalayer.variant import Variant

from foreman.adapters.datalayer.ctrlx_node_provider_base import CtrlXNodeProviderBase
from foreman.adapters.datalayer.flatbuffer_serialize import serialize_foreman_snapshot
from foreman.engine import ForemanEngine


class CtrlXNodeSnapshot(CtrlXNodeProviderBase):
    """Read-only node that serves the Foreman Engine state."""

    def __init__(self, provider: Provider, address: str, type_path: str, engine: ForemanEngine):
        self.engine = engine

        b = MetadataBuilder(allowed=AllowedOperation.READ)
        b = b.set_display_name("Foreman State Snapshot")
        b = b.set_node_class(NodeClass.NodeClass.Variable)
        b.add_reference(ReferenceType.read(), type_path)
        metadata = b.build()

        super().__init__(provider, address, metadata)

    def _on_read(self, userdata, address: str, data: Variant, cb: NodeCallback):
        """Serialize ForemanSnapshot and publish it."""
        try:
            snapshot_data = self.engine.get_engine_snapshot()
            fbs_bytes = serialize_foreman_snapshot(snapshot_data)

            v = Variant()
            v.set_flatbuffers(fbs_bytes)
            cb(Result.OK, v)
        except Exception as e:
            cb(Result.FAILED, None)
