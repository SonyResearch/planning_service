from unittest.mock import MagicMock, patch

import pytest
from planning_service_client.api.registry import RegistryClient
from proto import basic_types_pb2


def test_ctor(test_options):
    RegistryClient(addr="localhost:5050", options=test_options)
    assert True


@pytest.fixture
def mock_client():
    class MockRegistryClient(RegistryClient):
        pass

    return MockRegistryClient(addr="localhost:5050")


def test_get_version(mock_client):
    with patch.object(mock_client, "_do_unary_unary_rpc", return_value=MagicMock()) as mock_rpc:
        response = mock_client.get_version()
        mock_rpc.assert_called_once()
        assert response == mock_rpc.return_value


def test_remove_context(mock_client):
    with patch.object(mock_client, "_do_unary_unary_rpc", return_value=MagicMock()) as mock_rpc:
        response = mock_client.remove_context(system="test_context", context_id=basic_types_pb2.PlanContextId(value=10))
        mock_rpc.assert_called_once()
        assert response == mock_rpc.return_value
