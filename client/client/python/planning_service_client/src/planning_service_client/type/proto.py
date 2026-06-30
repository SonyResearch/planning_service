from __future__ import annotations

from abc import ABC, abstractmethod

from google.protobuf import message


class ProtoClass(ABC):
    """
    Abstract class used to define protobuf [de]serialization methods for message types.
    """

    @abstractmethod
    def __init__(self) -> None:
        pass

    @abstractmethod
    def __eq__(self, other: ProtoClass) -> bool:
        """
        Equality operator.
        """
        pass

    @abstractmethod
    def to_proto(self) -> message.Message:
        """
        Abstract method.

        Create a Protobuf message from the instance.
        """
        pass

    @classmethod
    @abstractmethod
    def from_proto(cls, msg: message.Message) -> ProtoClass:
        """
        Abstract static method.

        Create a new instance from a Protobuf message.
        """
        pass
