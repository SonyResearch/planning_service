#!/usr/bin/env python3
from __future__ import annotations

from logging import Logger
from typing import List, Union

from planning_service_client.api.base_client import Client, ClientOptions
from proto import basic_types_pb2
from proto.registry_pb2 import (
    GetPlanContextSummariesRequest,
    GetPlanContextSummariesResponse,
    GetPlanningArtifactStatusRequest,
    GetPlanningArtifactStatusResponse,
    GetVersionRequest,
    GetVersionResponse,
    MigratePlanningArtifactsRequest,
    RegisterPlanContextRequest,
    RegisterPlanContextResponse,
    RemovePlanContextRequest,
    RemovePlanContextResponse,
)
from proto.registry_pb2_grpc import PlanContextRegistryStub


class RegistryClient(Client):
    """
    Client to send requests to generate IRIS regions which are used for motion planning.
    """

    def __init__(self, addr: str, options: ClientOptions = None, logger: Logger = None) -> None:
        """
        Constructor.

        Args:
            addr (str): Address at which the registry service is awaiting new requests
        """
        super().__init__(addr, PlanContextRegistryStub, options, logger)

    def get_version(self) -> GetVersionResponse:
        """
        Get the version of planning_service.

        Returns:
            GetVersionResponse: The response message containing the version of the service (version field of the
            object).
        """
        return self._do_unary_unary_rpc(self._stub.GetVersion, GetVersionRequest())

    def handle_register_plan_context_request(
        self, system: str, context: basic_types_pb2.PlanContext
    ) -> Union[RegisterPlanContextResponse, None]:
        """
        Send a context to the server, which will save the data to disk, and return the corresponding unique ID.

        :param context: Target context
        :type context: PlanContext
        :return: Response from server
        :rtype: Union[RegisterPlanContextResponse, None]
        """
        req = RegisterPlanContextRequest(system=system, context=context)
        return self._do_unary_unary_rpc(self._stub.HandleRegisterPlanContextRequest, req)

    def remove_context(
        self, system: str, context_id: basic_types_pb2.PlanContextId
    ) -> Union[RemovePlanContextResponse, None]:
        """
        Remove a planning context and its associated model/planner instance.

        :param system: Target system
        :type system: str
        :param context_id: The context to be removed
        :type context_id: PlanContextId
        :return: Response from server
        :rtype: Union[RemovePlanContextResponse, None]
        """
        req = RemovePlanContextRequest(system=system, context_id=context_id)
        return self._do_unary_unary_rpc(self._stub.RemovePlanContext, req)

    def get_plan_context_summaries(self, system: str) -> Union[GetPlanContextSummariesResponse, None]:
        """
        Get a list of context IDs that are currently available on the server.

        :param system: System of interest :type
        system: str.

        :return: List of context IDs
        :rtype: Union[GetPlanContextSummariesResponse, None]
        """

        req = GetPlanContextSummariesRequest(system=system)
        return self._do_unary_unary_rpc(self._stub.GetPlanContextSummaries, req)

    def get_planning_artifact_status(
        self,
        system: str,
        context_ids: Union[basic_types_pb2.PlanContextId, List[basic_types_pb2.PlanContextId]],
        num_samples: int,
    ) -> Union[GetPlanningArtifactStatusResponse, None]:
        """
        Print the status of the planning artifacts for a given context ID.

        :param context_id: Target context ID
        :type context_id: int
        :param num_samples: Number of samples to generate
        :type num_samples: int
        """
        req = GetPlanningArtifactStatusRequest(context_ids=context_ids, num_samples=num_samples, system=system)

        return self._do_unary_unary_rpc(self._stub.GetPlanningArtifactStatus, req)

    def migrate_planning_artifacts(
        self, source_context_id: int, target_context_id: int, num_repair_samples: int, repair_artifacts: bool = True
    ) -> None:
        """
        Migrate the planning artifacts from one context to another.

        :param source_context_id: Source context ID to migrate from
        :type source_context_id: int
        :param target_context_id: Target context ID to migrate to
        :type target_context_id: int
        :param num_repair_samples: Number of samples to use for repairing the regions
        :type num_repair_samples: int
        """
        id = self._new_id()
        req = MigratePlanningArtifactsRequest(
            id=id,
            from_context_id=basic_types_pb2.PlanContextId(value=source_context_id),
            to_context_id=basic_types_pb2.PlanContextId(value=target_context_id),
            num_samples=num_repair_samples,
            repair_artifacts=repair_artifacts,
        )
        return self._do_unary_unary_rpc(self._stub.HandleMigratePlanningArtifactsRequest, req)
