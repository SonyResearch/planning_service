#!/usr/bin/env python3
from __future__ import annotations

from enum import Enum
from logging import Logger
from typing import List

from planning_service_client.api.base_client import Client, ClientOptions
from planning_service_client.type.plan_context import PlanContext
from planning_service_client.type.planning_problem import PlanningProblem
from planning_service_client.type.robot_state import SystemConf, SystemConfEdge
from proto import builder_pb2
from proto.builder_pb2_grpc import IrisBuilderStub


class SeedDataType(Enum):
    """
    Enum specifying desired seed data type for IRIS generation.
    """

    CONFIGS = "configs"
    EDGES = "edges"
    ROADMAP = "roadmap"
    PROBLEMS = "problems"


class IrisBuilderClient(Client):
    """
    Client to send requests to generate IRIS regions which are used for motion planning.
    """

    def __init__(self, addr: str, options: ClientOptions = None, logger: Logger = None) -> None:
        """
        Constructor.

        Args:
            addr (str): Address at which the IRIS builder service is awaiting new requests
        """
        super().__init__(addr, IrisBuilderStub, options, logger)
        self._context = None

    def update_roadmap_with_samples(self, num_samples: int) -> builder_pb2.UpdateRoadmapResponse:
        """
        Send a request to start a solve job for the planning problems generated from random samples.
        """
        id = self._new_id()
        context_msg = self._context.to_proto()
        req = builder_pb2.UpdateRoadmapRequest(
            id=id,
            context=context_msg,
            roadmap_data=builder_pb2.RoadmapData(num_samples=num_samples),
        )
        return self._do_unary_unary_rpc(self._stub.HandleUpdateRoadmapRequest, req)

    def update_roadmap_from_saved_problems(
        self, num_problems: int, num_fpp_problems: int, num_random_ik_seed_samples: int
    ) -> builder_pb2.UpdateRoadmapResponse:
        """
        Send a request to start a solve job for the planning problems loaded into the client.

        Then update the roadmap with the solution paths to the planning problems.
        """
        id = self._new_id()
        context_msg = self._context.to_proto()
        req = builder_pb2.UpdateRoadmapRequest(
            id=id,
            context=context_msg,
            generate_from_problems=builder_pb2.GenerateFromProblems(
                num_problems=num_problems,
                num_fpp_problems=num_fpp_problems,
                num_random_ik_seed_samples=num_random_ik_seed_samples,
                insert_solution_in_roadmap=num_random_ik_seed_samples > 0,
            ),
        )
        return self._do_unary_unary_rpc(self._stub.HandleUpdateRoadmapRequest, req)

    def update_roadmap_from_passed_problems(self, problems: List[PlanningProblem]) -> builder_pb2.UpdateRoadmapResponse:
        """
        Send a request to start a solve job for the planning problems passed to the client.

        Then update the roadmap with the solution paths to the planning problems.
        """
        id = self._new_id()
        context_msg = self._context.to_proto()
        req = builder_pb2.UpdateRoadmapRequest(
            id=id,
            context=context_msg,
            problem_def_vec=builder_pb2.ProblemDefVec(data=[problem.to_proto() for problem in problems]),
        )
        return self._do_unary_unary_rpc(self._stub.HandleUpdateRoadmapRequest, req)

    def start_build(
        self,
        seed_data_type: SeedDataType,
        configs: List[SystemConf] = None,
        edges: List[SystemConfEdge] = None,
        num_samples: int = None,
        planning_problems: List[PlanningProblem] = None,
        num_saved_problems: int = 0,
    ) -> builder_pb2.StartBuildResponse:
        """
        Send a request to start a build job for the loaded context and with the specified datatype.

        :param seed_data_type: Enum specifying the data type (thus build type) of the request
        :type seed_data_type: SeedDataType
        :param configs: List of configs, defaults to None
        :type configs: List[SystemConf], optional
        :param edges: List of config edges, defaults to None
        :type edges: List[SystemConfEdge], optional
        :param num_samples: Number of samples from which to solve roadmap problems, defaults to None
        :type num_samples: int, optional
        :param planning_problems: List of planning problems, defaults to None
        :type planning_problems: List[PlanningProblem], optional
        :return: Response from the server
        :rtype: builder_pb2.StartBuildResponse
        """
        id = self._new_id()
        context_msg = self._context.to_proto()
        if seed_data_type == SeedDataType.CONFIGS:
            req = builder_pb2.StartBuildRequest(
                id=id,
                context=context_msg,
                sysconf_vec=builder_pb2.SystemConfVec(data=[conf.to_proto() for conf in configs]),
            )
        elif seed_data_type == SeedDataType.EDGES:
            req = builder_pb2.StartBuildRequest(
                id=id,
                context=context_msg,
                sysconf_edge_vec=builder_pb2.SystemConfEdgeVec(data=[edge.to_proto() for edge in edges]),
            )
        if seed_data_type == SeedDataType.ROADMAP:
            req = builder_pb2.StartBuildRequest(
                id=id,
                context=context_msg,
                num_problems=num_saved_problems,
                roadmap_data=builder_pb2.RoadmapData(num_samples=num_samples),
            )
        if seed_data_type == SeedDataType.PROBLEMS:
            req = builder_pb2.StartBuildRequest(
                id=id,
                context=context_msg,
                problem_def_vec=builder_pb2.ProblemDefVec(data=[problem.to_proto() for problem in planning_problems]),
            )
        return self._do_unary_unary_rpc(self._stub.HandleStartBuildRequest, req)

    def set_context_by_id(self, name: str, id: int) -> None:
        """
        Set the target context by its corresponding unique ID.

        If the context is not currently loaded in the IRIS builder, it will not be able to start a job.
        """
        self._context = PlanContext(name=name, id=id)

    def set_context(self, context: PlanContext):
        """
        Set the active context.
        """
        self._context = context

    def clear_context(self) -> None:
        """
        Clear the active context.
        """
        self._context = None
