from __future__ import annotations

from planning_service_client.type.multimodal_parameters import MultimodalParameters
from planning_service_client.type.proto import ProtoClass
from planning_service_client.type.robot_state import FrameRelativePose, Deprecated_State, SystemConf
from proto import basic_types_pb2


class PlanningProblem(ProtoClass):
    """
    Class representing a planning problem between a start and goal state.
    """

    def __init__(
        self,
        name: str,
        context_id: int,
        goal: Deprecated_State,
        start: Deprecated_State,
        ik_seed: SystemConf = None,
        linear: bool = False,
        out_of_violation: bool = False,
        multimodal: bool = False,
        multimodal_parameters: MultimodalParameters = None,
        replace_goal_with_closest_valid_conf: bool = False,
    ) -> PlanningProblem:
        if not (isinstance(goal, Deprecated_State) and isinstance(start, Deprecated_State)):
            raise TypeError(
                f"Start and goal must be 'Deprecated_State'. Got ('{type(start).__name__}', '{type(goal).__name__}'). "
            )
        if goal.pose() is not None and ik_seed is None:
            raise ValueError("For a goal state defined with a FrameRelativePose, an IK seed must be specified!")
        self._name = name
        self._context_id = context_id
        self._goal = goal
        self._start = start
        self._ik_seed = ik_seed
        self._linear = linear
        self._out_of_violation = out_of_violation
        self._multimodal = multimodal
        self._multimodal_parameters = multimodal_parameters
        self._replace_goal_with_closest_valid_conf = replace_goal_with_closest_valid_conf

    def __eq__(self, other: PlanningProblem) -> bool:
        return (
            self._name == other._name
            and self._context_id == other._context_id
            and self._goal == other._goal
            and self._start == other._start
            and self._ik_seed == other._ik_seed
            and self._linear == other._linear
            and self._out_of_violation == other._out_of_violation
            and self._multimodal == other._multimodal
            and self._multimodal_parameters == other._multimodal_parameters
            and self._replace_goal_with_closest_valid_conf == other._replace_goal_with_closest_valid_conf
        )

    @staticmethod
    def solution_type_to_proto(linear: bool, out_of_violation: bool, multimodal: bool) -> basic_types_pb2.SolutionType:
        """
        Convert a boolean pair to a corresponding SolutionType.
        """
        if out_of_violation:
            return basic_types_pb2.SolutionType.OUT_OF_VIOLATION
        elif linear:
            return basic_types_pb2.SolutionType.CARTESIAN_LINEAR
        elif multimodal:
            return basic_types_pb2.SolutionType.MULTIMODAL
        else:
            return basic_types_pb2.SolutionType.TIME_OPTIMAL

    def to_proto(self) -> basic_types_pb2.ProblemDef:
        """
        Create a corresponding Protobuf message from the given instance.
        """
        return basic_types_pb2.ProblemDef(
            name=self._name,
            context_id=basic_types_pb2.PlanContextId(value=self._context_id),
            goal=self._goal.to_proto(),
            start=self._start.to_proto(),
            params=self._multimodal_parameters.to_proto() if self._multimodal_parameters is not None else None,
            ik_seed=self._ik_seed.to_proto() if self._ik_seed is not None else None,
            solution_type=self.solution_type_to_proto(
                linear=self._linear, out_of_violation=self._out_of_violation, multimodal=self._multimodal
            ),
            replace_goal_with_closest_valid_conf=self._replace_goal_with_closest_valid_conf,
        )

    @classmethod
    def from_proto(cls, msg: basic_types_pb2.ProblemDef) -> PlanningProblem:
        """
        Create a new class instance from the given Protobuf message.
        """
        return PlanningProblem(
            name=msg.name,
            context_id=msg.context_id.value,
            goal=Deprecated_State.from_proto(msg.goal),
            start=Deprecated_State.from_proto(msg.start),
            ik_seed=(None if len(msg.ik_seed.data.items()) == 0 else SystemConf.from_proto(msg.ik_seed)),
            replace_goal_with_closest_valid_conf=msg.replace_goal_with_closest_valid_conf,
        )

    @classmethod
    def pose_to_pose(
        cls,
        name: str,
        context_id: int,
        goal: FrameRelativePose,
        start: FrameRelativePose,
        ik_seed: SystemConf,
        linear: bool,
    ) -> PlanningProblem:
        """
        Factory method to create a pose-pose planning problem.

        Args:
            name (str): Plan name.
            context_id (int): Unique context ID.
            goal (FrameRelativePose): Goal pose.
            start (FrameRelativePose): Start pose.
            ik_seed (SystemConf): IK seed/initial guess.

        Returns:
            PlanningProblem: Planning problem between two poses.
        """
        return PlanningProblem(
            name=name,
            context_id=context_id,
            goal=Deprecated_State(pose=goal),
            start=Deprecated_State(pose=start),
            ik_seed=ik_seed,
            linear=linear,
        )

    @classmethod
    def system_conf_to_pose(
        cls,
        name: str,
        context_id: int,
        goal: FrameRelativePose,
        start: SystemConf,
        linear: bool,
    ) -> PlanningProblem:
        """
        Factory method to create a system conf-pose planning problem.

        Args:
            name (str): Plan name.
            context_id (int): Unique context ID.
            goal (FrameRelativePose): Goal pose.
            start (SystemConf): Start system conf.

        Returns:
            PlanningProblem: Planning problem from a system conf to a relative
            goal pose.
        """
        return PlanningProblem(
            name=name,
            context_id=context_id,
            goal=Deprecated_State(pose=goal),
            start=Deprecated_State(system_conf=start),
            ik_seed=start,
            linear=linear,
        )

    @classmethod
    def system_conf_to_system_conf(
        cls,
        name: str,
        context_id: int,
        goal: SystemConf,
        start: SystemConf,
        replace_goal_with_closest_valid_conf: bool = False,
    ) -> PlanningProblem:
        """
        Factory method to create a system conf-system conf planning problem.

        Args:
            name (str): Plan name.
            context_id (int): Unique context ID.
            goal (SystemConf): Goal system conf.
            start (SystemConf): Start system conf.

        Returns:
            PlanningProblem: Planning problem between two system confs.
        """
        return PlanningProblem(
            name=name,
            context_id=context_id,
            goal=Deprecated_State(system_conf=goal),
            start=Deprecated_State(system_conf=start),
            replace_goal_with_closest_valid_conf=replace_goal_with_closest_valid_conf,
        )

    @classmethod
    def out_of_violation(cls, name: str, context_id: int, start: SystemConf) -> PlanningProblem:
        """
        Factory method to create an out of violation planning problem.

        Args:
            name (str): Plan name.
            context_id (int): Unique context ID.
            start (SystemConf): Start system conf.

        Returns:
            PlanningProblem: Planning problem to move out of violation.
        """
        return PlanningProblem(
            name=name,
            context_id=context_id,
            goal=Deprecated_State(system_conf=SystemConf()),
            start=Deprecated_State(system_conf=start),
            out_of_violation=True,
        )
