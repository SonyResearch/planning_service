"""
Smoke tests verifying that the bundled `proto` package is importable.

These tests confirm that the generated protobuf message types are available
after installing the wheel (i.e., that the `proto` package is bundled correctly).
"""

import pytest


@pytest.mark.parametrize(
    "module,symbol",
    [
        ("proto.basic_types_pb2", "Conf"),
        ("proto.basic_types_pb2", "ContextId"),
        ("proto.basic_types_pb2", "PlanContextId"),
        ("proto.basic_types_pb2", "SystemConf"),
        ("proto.planner_pb2", "SolvePlanRequest"),
        ("proto.registry_pb2", "GetPlanContextSummariesRequest"),
        ("proto.visualizer_pb2", "StartVisualizerRequest"),
        ("proto.builder_pb2", "StartBuildRequest"),
    ],
)
def test_proto_message_importable(module, symbol):
    import importlib

    mod = importlib.import_module(module)
    assert hasattr(mod, symbol), f"{module}.{symbol} not found"


@pytest.mark.parametrize(
    "module",
    [
        "proto.planner_pb2_grpc",
        "proto.registry_pb2_grpc",
        "proto.visualizer_pb2_grpc",
        "proto.builder_pb2_grpc",
    ],
)
def test_proto_grpc_stub_importable(module):
    import importlib

    importlib.import_module(module)
