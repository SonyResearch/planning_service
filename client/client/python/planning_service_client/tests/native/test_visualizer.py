"""
Unit tests for the native pybind11 `visualizer` sub-module.

Tests cover construction and the locally-executing methods of VisualizerClient (i.e., those that do not require an
active gRPC connection).
"""

import numpy as np
import pytest

import planning_service_client.native.types as types
import planning_service_client.native.visualizer as visualizer

# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────


@pytest.fixture
def client():
    """
    A VisualizerClient pointed at a non-existent server (no connection made).
    """
    return visualizer.VisualizerClient(
        addr="localhost:50052",
        client_id="unit-test",
    )


@pytest.fixture
def system_conf():
    return types.SystemConf(
        {
            "robot": types.Conf(np.array([0.0, 1.0, 2.0])),
        }
    )


# ──────────────────────────────────────────────────────────────────────────────
# Construction
# ──────────────────────────────────────────────────────────────────────────────


class TestVisualizerClientConstruction:
    def test_minimal_ctor(self):
        """
        Construct with only required positional args.
        """
        c = visualizer.VisualizerClient(
            addr="localhost:50052",
            client_id="test",
        )
        assert c is not None

    def test_ctor_with_explicit_config_json(self):
        """
        Construct with an explicit (empty) config_json.
        """
        c = visualizer.VisualizerClient(
            addr="localhost:50052",
            client_id="test",
            config_json="{}",
        )
        assert c is not None

    def test_ctor_keyword_args(self):
        """
        All three arguments can be supplied as keyword arguments.
        """
        c = visualizer.VisualizerClient(
            addr="192.168.1.100:9000",
            client_id="my-client",
            config_json="{}",
        )
        assert c is not None

    def test_ctor_positional_args(self):
        """
        All three arguments can be supplied positionally.
        """
        c = visualizer.VisualizerClient("localhost:50052", "pos-test", "{}")
        assert c is not None


# ──────────────────────────────────────────────────────────────────────────────
# Method existence
# ──────────────────────────────────────────────────────────────────────────────


class TestVisualizerClientMethodPresence:
    @pytest.mark.parametrize(
        "method_name",
        [
            "connect",
            "start_visualizer",
            "stop_visualizer",
            "get_visualizer_status",
            "toggle_object",
            "queue_streamed_configuration",
            "stream_configurations",
            "stream_configurations_async",
            "stop_stream_configurations",
            "calc_pose",
            "toggle_frame",
        ],
    )
    def test_has_method(self, client, method_name):
        assert hasattr(client, method_name), f"VisualizerClient is missing method '{method_name}'"
        assert callable(getattr(client, method_name))


# ──────────────────────────────────────────────────────────────────────────────
# Local (non-network) methods
# ──────────────────────────────────────────────────────────────────────────────


class TestVisualizerClientLocalMethods:
    def test_queue_streamed_configuration_does_not_raise(self, client, system_conf):
        """
        QueueStreamedConfiguration is a local queue push; no server needed.
        """
        client.queue_streamed_configuration(system_conf)

    def test_queue_multiple_configurations(self, client):
        """
        Multiple configurations can be queued without error.
        """
        for i in range(5):
            sc = types.SystemConf({"r": types.Conf(np.array([float(i)]))})
            client.queue_streamed_configuration(sc)

    def test_stop_stream_configurations_does_not_raise(self, client):
        """
        Calling stop before streaming starts should be a no-op.
        """
        client.stop_stream_configurations()

    def test_stop_stream_configurations_idempotent(self, client):
        """
        Calling stop multiple times should not raise.
        """
        client.stop_stream_configurations()
        client.stop_stream_configurations()

    def test_stream_configurations_async_and_stop(self, client):
        """
        Async streaming can be started and immediately stopped.
        """
        client.stream_configurations_async()
        client.stop_stream_configurations()
        # If we reach here without hanging or raising, the test passes.

    def test_queue_then_async_then_stop(self, client, system_conf):
        """
        Queue a configuration, start async streaming, then stop cleanly.
        """
        client.queue_streamed_configuration(system_conf)
        client.stream_configurations_async()
        client.stop_stream_configurations()

    def test_calc_pose_accepts_no_override(self, client):
        """
        calc_pose with no system_conf_override (default None) should fail with a network/RPC error, not a TypeError from
        the optional binding.
        """
        with pytest.raises(Exception) as exc_info:
            client.calc_pose("world", "tool")
        assert not isinstance(exc_info.value, TypeError)

    def test_calc_pose_accepts_none_override(self, client):
        """
        calc_pose with system_conf_override=None should be accepted by the binding and fail with a network/RPC error,
        not a TypeError.
        """
        with pytest.raises(Exception) as exc_info:
            client.calc_pose("world", "tool", system_conf_override=None)
        assert not isinstance(exc_info.value, TypeError)

    def test_calc_pose_accepts_system_conf_override(self, client, system_conf):
        """
        calc_pose with an explicit SystemConf override should be accepted by the binding and fail with a network/RPC
        error, not a TypeError.
        """
        with pytest.raises(Exception) as exc_info:
            client.calc_pose("world", "tool", system_conf_override=system_conf)
        assert not isinstance(exc_info.value, TypeError)
