from __future__ import annotations

import threading
import time
from abc import ABC
from dataclasses import dataclass
from logging import Logger
from typing import Any, Generator, List, Optional, Tuple, Union

import grpc
from google.protobuf import message
from google.protobuf.json_format import MessageToJson
from grpc_health.v1.health_pb2 import HealthCheckRequest, HealthCheckResponse
from grpc_health.v1.health_pb2_grpc import HealthStub
from planning_service_client.utils.default_logger import default_logger

Callable = Union[
    grpc.UnaryStreamMultiCallable,
    grpc.UnaryUnaryMultiCallable,
    grpc.StreamUnaryMultiCallable,
    grpc.StreamStreamMultiCallable,
]


@dataclass
class ClientOptions:
    """
    Dataclass to hold client options.
    """

    # Client name
    name: str = "Client"
    # If true, continuously monitor the health of the server
    monitor_health: bool = True
    # If true, attempt to reconnect to the server on failure
    reconnect_on_failure: bool = False
    # Options passed to the gRPC channel
    grpc_options: List[Tuple[str, str]] = None
    # Logger level
    log_level: str = "INFO"


# Alias for the Stub class passed by derived classes
StubClass = Any


class Client(ABC):
    """
    Abstract client class used to implement a gRPC client stub.
    """

    # Health check interval
    health_check_interval_ms = 100

    def __init__(self, addr: str, stub: StubClass, options: ClientOptions = None, logger: Logger = None) -> None:
        """
        Constructor.

        :param addr: Address at which the target server is awaiting new requests
        :type addr: str
        :param stub: Stub class for the target service
        :type stub: StubClass
        :param options: Client options, defaults to None
        :type options: ClientOptions, optional
        :param logger: Logger, defaults to None
        :type logger: Logger, optional
        """
        self._addr = addr
        self._options = options or ClientOptions()
        self._logger = logger or default_logger(self._options.name)
        self._logger.setLevel(self._options.log_level)
        # Configure and create gRPC channel
        self._channel_options = self._options.grpc_options or []
        # Set stub class
        self._channel = None
        self._stubclass = stub
        self._set_channel_and_stubs(self._addr, self._channel_options)
        # Lock for connection and health checking
        self._lock = threading.RLock()
        self._exit_triggered = threading.Event()
        # Watcher returned by the health check and the thread running it
        self._watcher = None
        self._watch_thread = None
        # Connection status
        self._connected = False
        # Serving status - note that we can be connected to a server which is not actively serving new requests
        self._serve_status = HealthCheckResponse.UNKNOWN

        # Connection inputs for retry
        self._connect_timeout_s = None
        self._connect_max_retries = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.disconnect()

    def _set_channel_and_stubs(self, addr: str, options: Optional[List[Tuple[str, str]]] = None) -> None:
        """
        Set the channel and stubs.
        """
        self._channel = grpc.insecure_channel(addr, options=options or [])
        self._stub = self._stubclass(self._channel)
        self._health_stub = HealthStub(self._channel)

    def disconnect(self):
        """
        Disconnect from the server.
        """
        # Set bools false
        self._connected = False
        self._serve_status = HealthCheckResponse.UNKNOWN
        # Trigger exit of watcher thread
        self._exit_triggered.set()
        # Close and delete channel
        self._channel.close()
        self._channel = None
        # Join watcher thread
        if self._watch_thread is not None:
            self._logger.debug("Joining health watcher thread...")
            self._watch_thread.join()
        self._exit_triggered.clear()
        # Reset stubs
        self._stub, self._health_stub = None, None

    def set_logger_level(self, level: int):
        self._logger.setLevel(level)

    def connect(self, timeout_s: float = 10.0, max_retries: int = 3) -> bool:
        """
        Connect to the server at the client address.

        For up to the maximum number of retries, wait for the timeout in seconds for the server to enter a ready state.
        If enabled, initiate a continuous monitor of the health of the server.

        :param timeout_s: Timeout in seconds, defaults to 10.0
        :type timeout_s: float. optional
        :param max_retries: Maximum number of retries, defaults to 3
        :type max_retries: int, optional
        :return: True if a connection was made, False otherwise
        :rtype: bool
        """
        if self._connected:
            return self._connected
        if self._channel is None:
            self._set_channel_and_stubs(self._addr, self._channel_options)
        for i in range(max_retries):
            self._logger.info(f"Connection attempt {i + 1}/{max_retries}...")
            try:
                grpc.channel_ready_future(self._channel).result(timeout=timeout_s)
            except grpc.FutureTimeoutError:
                continue
            self._logger.info(f"Connected to {self._addr} after {i + 1} attempt{'s' if i > 0 else ''}.")
            # Log connection inputs for retry
            self._connect_timeout_s = timeout_s
            self._connect_max_retries = max_retries
            self._connected = True
            if self._options.monitor_health:
                with self._lock:
                    # Create new watcher instance with the new connection
                    self._logger.debug("Creating new health watcher...")
                    self._watcher = self._do_unary_stream_rpc(self._health_stub.Watch, HealthCheckRequest())
                    self._watch_thread = threading.Thread(target=self._watch_health)
                    self._watch_thread.start()
                    self._logger.debug("Health watcher created.")
            return True
        self._logger.error(f"Failed to connect to {self._addr} after {max_retries} attempt{'s' if i > 0 else ''}.")
        return False

    def connected(self) -> bool:
        return self._connected

    def serving(self) -> bool:
        return self._serve_status == HealthCheckResponse.SERVING

    def _watch_health(self):
        """
        Watch the health of the server.

        This method continuously reads out status updates from the health watcher and updates the serving status of the
        client accordingly. If the connection is lost, the client will attempt to reconnect to the server if the
        corresponding flag has been set.
        """
        while True:
            with self._lock:
                if self._watcher is not None:
                    self._logger.debug("Health watcher thread is now active.")
                    for response in self._watcher:
                        if response is None:
                            self._watcher = None
                            self._serve_status = HealthCheckResponse.UNKNOWN
                            self._connected = False
                            if self._exit_triggered.is_set():
                                self._logger.debug("Disconnect triggered, exiting health watcher thread.")
                                return
                            self._logger.error("Health watcher failed!")
                            if not self._options.reconnect_on_failure:
                                self._logger.debug("Reconnect on failure is disabled; exiting health watcher thread.")
                                return
                            self._logger.error("Reconnecting...")
                            if not self.connect(
                                timeout_s=self._connect_timeout_s, max_retries=self._connect_max_retries
                            ):
                                return
                        self._logger.debug(f"Serving status: {HealthCheckResponse.ServingStatus.Name(response.status)}")
                        self._serve_status = response.status
            time.sleep(self.health_check_interval_ms / 1000.0)

    def _new_id(self) -> str:
        """
        Create a request ID from the current time in nanos.
        """
        return str(time.monotonic_ns())

    def _parse_callable_name(self, callable: Callable) -> str:
        """
        Extract the method name from the callable.

        The format is always "pkg.subpkg.subpkg.service/method"; for brevity, we only log as "service/method".

        :param callable: gRPC callable
        :type callable: Callable
        :return: Name as "service/method"
        :rtype: str
        """
        # Extract method name from callable - the format is always "pkg.subpkg.subpkg.service/method";
        # for brevity, we only log as "service/method"
        tokens = callable._method.decode("utf-8").split("/")
        return f"{tokens[-2].split('.')[-1]}/{tokens[-1]}"

    def _handle_rpc_error(self, err: grpc.RpcError, callable: Callable):
        """
        Handle an RPC error.

        :param err: Error
        :type err: grpc.RpcError
        :param callable: Callable which generated the error
        :type callable: Callable
        """
        if self._exit_triggered.is_set():
            self._logger.debug("Disconnect triggered, error handling may be suppressed.")
            return
        self._log_rpc_error(err, callable)
        if err.code() == grpc.StatusCode.UNAVAILABLE:
            self._connected = False
            self._serve_status = HealthCheckResponse.UNKNOWN

    def _log_rpc_error(self, err: grpc.RpcError, callable: Callable):
        """
        Given an RPC error and method, extract the method name, and the error code and details, and log them with the
        logger.

        :param err: Error
        :type err: grpc.RpcError
        :param callable: Callable which generated the error
        :type callable: Callable
        """

        self._logger.error(f"RPC {self._parse_callable_name(callable)} failed ({err.code()}, {err.details()})")

    def _do_unary_stream_rpc(
        self, callable: grpc.UnaryStreamMultiCallable, req: message.Message
    ) -> Generator[Union[message.Message, None], None, None]:
        """
        Wrapper for RPC unary-stream callable which provides error handling.

        :param callable: RPC handle to be run.
        :type callable: grpc.UnaryStreamMultiCallable
        :param req: Request message.
        :type req: message.Message
        :raises RuntimeError: If the client is not connected to the server.
        :yield: Response on success, None on failure.
        :rtype: Generator[Union[message.Message, None], None, None]
        """
        self._logger.debug(f"{self._parse_callable_name(callable)}: Request: {MessageToJson(req)}")
        try:
            call = callable(req)
            for response in call:
                self._logger.debug(f"{self._parse_callable_name(callable)}: Response: {MessageToJson(response)}")
                yield response
        except grpc.RpcError as err:
            self._handle_rpc_error(err, callable)
        except Exception as e:
            self._logger.error(f"Error occurred: {e}")
        yield None

    def _do_unary_unary_rpc(
        self, callable: grpc.UnaryUnaryMultiCallable, req: message.Message
    ) -> Union[message.Message, None]:
        """
        Wrapper for RPC unary-unary callable which provides error handling.

        Example: Instead of calling `self._stub.HandleSomeRequest(request)` directly, you would
        instead call `self._do_unary_unary_rpc(self._stub.HandleSomeRequest, request)`.

        :param callable: RPC handle to be run.
        :type callable: grpc.UnaryUnaryMultiCallable
        :param req: Request message.
        :type req: message.Message
        :raises RuntimeError: If the client is not connected to the server.
        :return: Response on success, None on failure.
        :rtype: Union[message.Message, None]
        """
        self._logger.debug(f"{self._parse_callable_name(callable)}: Request: {MessageToJson(req)}")
        try:
            response = callable(req)
            self._logger.debug(f"{self._parse_callable_name(callable)}: Response: {MessageToJson(response)}")
            return response
        except grpc.RpcError as err:
            self._handle_rpc_error(err, callable)
        except Exception as e:
            self._logger.error(f"Error occurred: {e}")
        return None
