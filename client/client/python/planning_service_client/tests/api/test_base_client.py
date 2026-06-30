import time
from concurrent import futures
from unittest.mock import MagicMock, Mock, patch

import grpc
import pytest
from client.python.planning_service_client.tests import dummy_pb2_grpc
from google.protobuf import message
from grpc_health.v1 import health, health_pb2, health_pb2_grpc
from planning_service_client.api.base_client import Client, ClientOptions


@pytest.fixture
def address():
    return "[::]:50051"


@pytest.fixture
def dummy_client():
    class DummyClient(Client):
        def __init__(self, addr, options=None):
            super().__init__(addr, dummy_pb2_grpc.DummyServiceStub, options)

    return DummyClient


@pytest.fixture()
def dummy_server(address):
    servicer = dummy_pb2_grpc.DummyServiceServicer()
    server = grpc.server(thread_pool=futures.ThreadPoolExecutor(max_workers=10))
    dummy_pb2_grpc.add_DummyServiceServicer_to_server(servicer, server)
    # Add health check servicer
    health_pb2_grpc.add_HealthServicer_to_server(health.HealthServicer(), server)
    server.add_insecure_port(address)
    yield server
    server.stop(0)


def test_constructor(dummy_client, address):
    with pytest.raises(TypeError):
        dummy_client()
    client = dummy_client(address)
    assert client._addr == address
    assert client._stubclass == dummy_pb2_grpc.DummyServiceStub
    # Default options
    assert client._options.name == "Client"
    assert client._options.monitor_health
    assert not client._options.reconnect_on_failure
    assert not client.connected()
    assert not client.serving()

    custom_options = ClientOptions()
    custom_options.name = "CustomClient"
    custom_options.monitor_health = False
    custom_options.reconnect_on_failure = True
    client = dummy_client(address, custom_options)
    assert client._options.name == "CustomClient"
    assert not client._options.monitor_health
    assert client._options.reconnect_on_failure


def test_connect(dummy_server, dummy_client, address):
    client = dummy_client(address)
    assert not client.connected()
    assert not client.serving()
    assert client._serve_status == health_pb2.HealthCheckResponse.UNKNOWN
    client.connect(timeout_s=0.1, max_retries=1)
    assert not client.connected()
    dummy_server.start()
    client.connect()
    assert client.connected()


def test_disconnect(dummy_server, dummy_client, address):
    dummy_server.start()
    client = dummy_client(address)
    client.connect()
    assert client.connected()
    client.disconnect()
    assert not client.connected()
    assert client._channel is None
    client.connect()
    assert client.connected()
    client.disconnect()
    assert not client.connected()


def test_watch_health(dummy_server, dummy_client, address):
    client = dummy_client(address)
    dummy_server.start()
    client.connect()
    assert client.connected()
    time.sleep(0.01)
    assert client.serving()
    dummy_server.stop(0)
    time.sleep(0.01)
    assert not client.serving()
    assert client._serve_status == health_pb2.HealthCheckResponse.UNKNOWN
    assert client._watcher is None


def test_context_management(dummy_server, dummy_client, address):
    dummy_server.start()
    with dummy_client(address) as client:
        assert client.connected()
        time.sleep(0.01)
        assert client.serving()


@pytest.fixture
def mock_client():
    class MockClient(Client):
        pass

    with patch("google.protobuf.json_format.MessageToJson", return_value="{}"):
        client = MockClient(addr="localhost:50051", stub=Mock())
        client._connected = True
        return client


@pytest.fixture
def mock_unary_unary():
    callable = Mock(spec=grpc.UnaryUnaryMultiCallable)
    callable._method = b"test.TestService/TestMethod"
    return callable


def test_do_unary_unary(mock_client, mock_unary_unary):
    mock_callable = mock_unary_unary
    mock_req = MagicMock(spec=message.Message)
    mock_resp = MagicMock(spec=message.Message)
    mock_callable.return_value = mock_resp

    resp = mock_client._do_unary_unary_rpc(mock_callable, mock_req)
    mock_callable.assert_called_once_with(mock_req)
    assert resp == mock_resp


@pytest.fixture
def dummy_error():
    class DummyError(grpc.RpcError):
        def __init__(self, code):
            self._code = code

        def code(self):
            return self._code

        def details(self):
            return "Dummy error details"

    return DummyError


@pytest.mark.parametrize(
    "error_code",
    [
        grpc.StatusCode.UNAVAILABLE,
        grpc.StatusCode.DEADLINE_EXCEEDED,
        grpc.StatusCode.INTERNAL,
    ],
)
def test_do_unary_unary_rpc_error(mock_client, mock_unary_unary, dummy_error, error_code):
    mock_callable = mock_unary_unary
    mock_request = MagicMock(spec=message.Message)

    # Create a mock RPC error
    mock_callable.side_effect = dummy_error(error_code)

    # Act
    result = mock_client._do_unary_unary_rpc(mock_callable, mock_request)
    mock_callable.assert_called_once_with(mock_request)
    assert result is None
    if error_code == grpc.StatusCode.UNAVAILABLE:
        assert not mock_client._connected


@pytest.fixture
def mock_unary_stream():
    callable = Mock(spec=grpc.UnaryStreamMultiCallable)
    callable._method = b"test.TestService/TestStreamMethod"
    return callable


def test_do_unary_stream_rpc(mock_client, mock_unary_stream):
    mock_callable = mock_unary_stream
    mock_req = MagicMock(spec=message.Message)
    mock_resp_1 = MagicMock(spec=message.Message)
    mock_resp_2 = MagicMock(spec=message.Message)
    mock_callable.return_value = iter([mock_resp_1, mock_resp_2])

    responses = list(mock_client._do_unary_stream_rpc(mock_callable, mock_req))
    mock_callable.assert_called_once_with(mock_req)
    assert responses[0] == mock_resp_1
    assert responses[1] == mock_resp_2
