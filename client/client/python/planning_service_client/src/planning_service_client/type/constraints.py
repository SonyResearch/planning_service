from __future__ import annotations

from abc import abstractmethod
from typing import List, Tuple, Union

from planning_service_client.type.proto import ProtoClass
from proto import basic_types_pb2


class Constraint(ProtoClass):
    """
    Abstract base class for all Constraints.
    """

    @abstractmethod
    def __init__(self) -> None:
        pass


class Deprecated_PositionConstraint(Constraint):
    """
    Constraint on Cartesian position of a target frame.

    Constrains the position of a point Q, rigidly attached to a frame B, to be within a bounding box measured and
    expressed in frame A. Namely p_AQ_lower <= p_AQ <= p_AQ_upper. Read more at:
    https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_position_constraint.html.
    """

    def __init__(
        self,
        frame_A: str,
        frame_B: str,
        p_AQ_lower: Tuple[float, float, float],
        p_AQ_upper: Tuple[float, float, float],
        p_BQ: Tuple[float, float, float],
    ) -> None:
        if len(frame_A) == 0 or len(frame_B) == 0:
            raise ValueError("Empty frame(s) provided.")
        if len(p_AQ_lower) != 3 or len(p_AQ_upper) != 3 or len(p_BQ) != 3:
            raise ValueError("Inputs of incorrect dimension!")
        self._frame_A = frame_A
        self._frame_B = frame_B
        self._p_AQ_lower = p_AQ_lower
        self._p_AQ_upper = p_AQ_upper
        self._p_BQ = p_BQ

    def __eq__(self, other: Deprecated_PositionConstraint) -> bool:
        return (
            self._frame_A == other._frame_A
            and self._frame_B == other._frame_B
            and self._p_AQ_lower == other._p_AQ_lower
            and self._p_AQ_upper == other._p_AQ_upper
            and self._p_BQ == other._p_BQ
        )

    def __repr__(self) -> str:
        string = "\n"
        string += f"frame_A:    {self._frame_A}\n"
        string += f"frame_B:    {self._frame_B}\n"
        string += f"p_AQ_lower: {self._p_AQ_lower}\n"
        string += f"p_AQ_upper: {self._p_AQ_upper}\n"
        string += f"p_BQ:       {self._p_BQ}\n"
        return string

    def to_proto(self) -> basic_types_pb2.Deprecated_PositionConstraint:
        """
        Create a corresponding Protobuf message from the given instance.
        """
        return basic_types_pb2.Deprecated_PositionConstraint(
            frame_A=self._frame_A,
            frame_B=self._frame_B,
            p_AQ_lower=self._p_AQ_lower,
            p_AQ_upper=self._p_AQ_upper,
            p_BQ=self._p_BQ,
        )

    @classmethod
    def from_proto(cls, msg: basic_types_pb2.Deprecated_PositionConstraint) -> Deprecated_PositionConstraint:
        """
        Create a new class instance from the given Protobuf message.
        """
        return Deprecated_PositionConstraint(
            frame_A=msg.frame_A,
            frame_B=msg.frame_B,
            p_AQ_lower=msg.p_AQ_lower,
            p_AQ_upper=msg.p_AQ_upper,
            p_BQ=msg.p_BQ,
        )


class Deprecated_AngleBetweenVectorsConstraint(Constraint):
    """
    Constraint on the angle between two vecotrs.

    Constrains that the angle between a vector a and another vector b is
    between [θ_lower, θ_upper].

    a is fixed to a frame A, while b is fixed to a frame B.
    Mathematically, if we denote a_unit_A as a expressed in frame A
    after normalization (a_unit_A has unit length), and b_unit_B as b
    expressed in frame B after normalization, the constraint is
    cos(θ_upper) ≤ a_unit_Aᵀ * R_AB * b_unit_B ≤ cos(θ_lower). Read more
    at:
    https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_angle_between_vectors_constraint.html.
    """

    def __init__(
        self,
        frame_A: str,
        frame_B: str,
        a_A: Tuple[float, float, float],
        b_B: Tuple[float, float, float],
        angle_lower: float,
        angle_upper: float,
    ) -> None:
        if len(frame_A) == 0 or len(frame_B) == 0:
            raise ValueError("Empty frame(s) provided.")
        if len(a_A) != 3 or len(b_B) != 3:
            raise ValueError("Inputs of incorrect dimension!")
        if angle_lower < 0:
            raise ValueError("Minimum angle must be nonnegative.")
        if angle_upper < angle_lower:
            raise ValueError("'angle_upper' must be at greater than or equal to 'angle_lower.'")
        self._frame_A = frame_A
        self._frame_B = frame_B
        self._a_A = a_A
        self._b_B = b_B
        self._angle_lower = angle_lower
        self._angle_upper = angle_upper

    def __eq__(self, other: Deprecated_AngleBetweenVectorsConstraint) -> bool:
        return (
            self._frame_A == other._frame_A
            and self._frame_B == other._frame_B
            and self._a_A == other._a_A
            and self._b_B == other._b_B
            and self._angle_lower == other._angle_lower
            and self._angle_upper == other._angle_upper
        )

    def __repr__(self) -> str:
        string = "\n"
        string += f"frame_A:     {self._frame_A}\n"
        string += f"frame_B:     {self._frame_B}\n"
        string += f"a_A:         {self._a_A}\n"
        string += f"b_B:         {self._b_B}\n"
        string += f"angle_lower: {self._angle_lower}\n"
        string += f"angle_upper: {self._angle_upper}\n"
        return string

    def to_proto(self) -> basic_types_pb2.Deprecated_AngleBetweenVectorsConstraint:
        """
        Create a corresponding Protobuf message from the given instance.
        """
        return basic_types_pb2.Deprecated_AngleBetweenVectorsConstraint(
            frame_A=self._frame_A,
            frame_B=self._frame_B,
            a_A=self._a_A,
            b_B=self._b_B,
            angle_lower=self._angle_lower,
            angle_upper=self._angle_upper,
        )

    @classmethod
    def from_proto(
        cls, msg: basic_types_pb2.Deprecated_AngleBetweenVectorsConstraint
    ) -> Deprecated_AngleBetweenVectorsConstraint:
        return Deprecated_AngleBetweenVectorsConstraint(
            frame_A=msg.frame_A,
            frame_B=msg.frame_B,
            a_A=msg.a_A,
            b_B=msg.b_B,
            angle_lower=msg.angle_lower,
            angle_upper=msg.angle_upper,
        )


class ConstraintsSet(ProtoClass):
    """
    The set of all desired constraints.
    """

    def __init__(
        self,
        position_constraints: Union[Deprecated_PositionConstraint, List[Deprecated_PositionConstraint]] = None,
        angle_constraints: Union[
            Deprecated_AngleBetweenVectorsConstraint, List[Deprecated_AngleBetweenVectorsConstraint]
        ] = None,
    ) -> None:
        for clist in [position_constraints, angle_constraints]:
            if isinstance(clist, list) and len(clist) == 0:
                raise ValueError("A list of constraints must be populated with at least one instance.")

        if isinstance(position_constraints, Deprecated_PositionConstraint):
            position_constraints = [position_constraints]
        if isinstance(angle_constraints, Deprecated_AngleBetweenVectorsConstraint):
            angle_constraints = [angle_constraints]
        self._position_constraints = position_constraints
        self._angle_constraints = angle_constraints

    def __eq__(self, other: ConstraintsSet) -> bool:
        return (
            self._angle_constraints == other._angle_constraints
            and self._position_constraints == other._position_constraints
        )

    def __repr__(self) -> str:
        string = ""
        string += "position_constraints: ["
        if self._position_constraints is not None:
            string += ",\n".join([pc.__repr__() for pc in self._position_constraints])
        string += "]"
        string += "\n"
        string += "angle_constraints: ["
        if self._angle_constraints is not None:
            string += ",\n".join([ac.__repr__() for ac in self._angle_constraints])
        string += "]"
        return string

    def add(self, constraints: Union[Constraint, List[Constraint]]) -> None:
        """
        Add a Constraint to the set.

        Args:
            constraint (Union[Constraint, List[Constraint]]): Constraint to be added.

        Raises:
            ValueError: If constraint is of unsupported type.
        """
        if isinstance(constraints, list):
            if len(constraints) != 0:
                self.add(constraints=constraints[0])
                self.add(constraints=constraints[1:])
        elif isinstance(constraints, Deprecated_PositionConstraint):
            if self._position_constraints is None:
                self._position_constraints = []
            self._position_constraints.append(constraints)
        elif isinstance(constraints, Deprecated_AngleBetweenVectorsConstraint):
            if self._angle_constraints is None:
                self._angle_constraints = []
            self._angle_constraints.append(constraints)
        else:
            raise ValueError(f"Constraint of unsupported type ({type(constraints).__name__}) provided!")

    def to_proto(self) -> basic_types_pb2.ConstraintsSet:
        """
        Create a corresponding Protobuf message from the given instance.
        """
        return basic_types_pb2.ConstraintsSet(
            pos_constraints=(
                [] if self._position_constraints is None else [pc.to_proto() for pc in self._position_constraints]
            ),
            angle_constraints=(
                [] if self._angle_constraints is None else [ac.to_proto() for ac in self._angle_constraints]
            ),
        )

    @classmethod
    def from_proto(cls, msg: basic_types_pb2.ConstraintsSet) -> ConstraintsSet:
        """
        Create a new class instance from the given Protobuf message.
        """
        return ConstraintsSet(
            position_constraints=[Deprecated_PositionConstraint.from_proto(pc) for pc in msg.pos_constraints],
            angle_constraints=[Deprecated_AngleBetweenVectorsConstraint.from_proto(ac) for ac in msg.angle_constraints],
        )
