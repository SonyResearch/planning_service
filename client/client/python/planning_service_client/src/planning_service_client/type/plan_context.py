from __future__ import annotations

from typing import List

from planning_service_client.type.constraints import ConstraintsSet
from planning_service_client.type.proto import ProtoClass
from proto import basic_types_pb2
from proto.basic_types_pb2 import Model as ModelPB


class PlanContext(ProtoClass):
    """
    Unique context for motion planning consisting of geometry and constraints.
    """

    def __init__(
        self,
        name: str = "",
        model_directive_raw: str = "",
        models: List[ModelPB] = None,
        constraints: ConstraintsSet = None,
        id: int = 0,
    ) -> None:
        self._name = name
        self._model_directive_raw = model_directive_raw
        self._models = models
        self._constraints = constraints
        self._id = id

    def __eq__(self, other: PlanContext) -> bool:
        return (
            self._name == other._name
            and self._model_directive_raw == other._model_directive_raw
            and self._constraints == other._constraints
            and self._models == other._models
            and self._id == other._id
        )

    def to_proto(self) -> basic_types_pb2.PlanContext:
        """
        Create a corresponding Protobuf message from the given instance.
        """
        constraints = ConstraintsSet().to_proto() if self._constraints is None else self._constraints.to_proto()
        return basic_types_pb2.PlanContext(
            name=self._name,
            model_directive_raw=self._model_directive_raw.encode("utf-8"),
            models=self._models,
            constraints=constraints,
            id=basic_types_pb2.PlanContextId(value=self._id),
        )

    @classmethod
    def from_proto(cls, msg: basic_types_pb2.PlanContext) -> PlanContext:
        """
        Create a new class instance from the given Protobuf message.
        """
        return PlanContext(
            name=msg.name,
            model_directive_raw=msg.model_directive_raw,
            models=msg.models,
            constraints=ConstraintsSet.from_proto(msg.constraints),
            id=msg.id.value,
        )
