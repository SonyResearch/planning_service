from dataclasses import dataclass, field

from planning_service_client.type.robot_state import RigidTransform
from proto import basic_types_pb2


@dataclass(frozen=False)
class MultimodalParameters:
    """
    Multimodal parameters used for a motion problem definition. By default the offsets are a null transform (no offset)

    Raises:
        ValueError: If the multimodal type is not taken into account.

    Returns:
        basic_types_pb2.Parameters: A Parameters proto message.
    """

    start_offset: RigidTransform = field(
        default_factory=lambda: RigidTransform(translation=[0, 0, 0], quaternion=[1, 0, 0, 0])
    )
    goal_offset: RigidTransform = field(
        default_factory=lambda: RigidTransform(translation=[0, 0, 0], quaternion=[1, 0, 0, 0])
    )

    def __eq__(self, other) -> bool:
        """
        Overload the equality operator.

        Args:
            other (MultimodalParameters): The other multimodal parameters to compare with.

        Returns:
            bool: True if the multimodal parameters are equal, False otherwise.
        """
        return self.start_offset == other.start_offset and self.goal_offset == other.goal_offset

    def to_proto(self) -> basic_types_pb2.Parameters:
        """
        Convert the multimodal parameters to a proto message.

        Returns:
            basic_types_pb2.Parameters: The proto message.
        """
        return basic_types_pb2.Parameters(
            multimodal_start_offset=self.start_offset.to_proto(),
            multimodal_goal_offset=self.goal_offset.to_proto(),
        )
