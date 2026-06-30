"""
System tests for the visualizer service – Python client (native/pybind11).

Frame combinations are loaded from system_tests/visualizer_frames.yaml: every source frame is tested against every
target frame.
"""

import itertools
import math
import os
import time

import numpy as np
import pytest
import yaml
from planning_service_client.native.types import Conf, SystemConf, VisualizerStatus
from planning_service_client.native.visualizer import VisualizerClient

VIZ_ADDR = os.getenv("VIZ_ADDR", "visualizer-dev:5550")

# Locate visualizer_frames.yaml whether running under Bazel or directly.
_TEST_SRCDIR = os.getenv("TEST_SRCDIR")
if _TEST_SRCDIR:
    _FRAMES_FILE = os.path.join(_TEST_SRCDIR, "_main", "system_tests", "visualizer_frames.yaml")
else:
    _FRAMES_FILE = os.path.join(os.path.dirname(__file__), "..", "visualizer_frames.yaml")

with open(_FRAMES_FILE) as _f:
    _cfg = yaml.safe_load(_f)
    _FRAME_PAIRS = list(itertools.product(_cfg["source_frames"], _cfg["target_frames"]))


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _wait_for_active(client: VisualizerClient, *, timeout_s: float = 60.0, poll_s: float = 1.0):
    """
    Poll get_visualizer_status until the session is ACTIVE.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        s = client.get_visualizer_status()
        if s.status == VisualizerStatus.Status.ACTIVE:
            return s
        time.sleep(poll_s)
    raise TimeoutError(f"Visualizer did not reach ACTIVE status within {timeout_s:.0f} s")


# ---------------------------------------------------------------------------
# Module-scoped client – one connection for the whole file
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def viz_client() -> VisualizerClient:
    client = VisualizerClient(VIZ_ADDR, "py_viz_system_test")
    # Wait 30 seconds max
    assert client.connect(num_attempts=30, attempt_interval_ms=1000), "Could not connect to visualizer service"
    _wait_for_active(client)
    return client


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_is_active(viz_client):
    """
    The service should have loaded the default lifty context on startup.
    """
    assert viz_client.get_visualizer_status().status == VisualizerStatus.Status.ACTIVE


@pytest.mark.parametrize(
    "frame_a,frame_b",
    _FRAME_PAIRS,
    ids=[f"{a}__{b}".replace("::", "_") for a, b in _FRAME_PAIRS],
)
def test_calc_pose(viz_client, frame_a, frame_b):
    """
    calc_pose should return a valid FrameRelativePose for every combination.
    """
    pose = viz_client.calc_pose(frame_a, frame_b)
    assert pose.frame_A == frame_a
    assert pose.frame_B == frame_b
    assert pose.translation is not None
    assert pose.quaternion_wxyz is not None


# ---------------------------------------------------------------------------
# Joint-position override for calc_pose
# ---------------------------------------------------------------------------


class TestCalcPoseJointOverride:
    """
    Tests for the system_conf_override parameter of calc_pose.

    Lifty joint order within each Conf vector: [rev, piston]
      rev    ∈ [-1.57, 1.57]  (revolute)
      piston ∈ [0.00,  0.30]  (prismatic)
    """

    _CONF_A = [1.0, 0.25]  # a: rev=1.0, piston=0.25
    _CONF_B = [-1.0, 0.05]  # b: rev=-1.0, piston=0.05

    @staticmethod
    def _make_override() -> SystemConf:
        sc = SystemConf()
        sc.set("a", Conf(np.array(TestCalcPoseJointOverride._CONF_A)))
        sc.set("b", Conf(np.array(TestCalcPoseJointOverride._CONF_B)))
        return sc

    @pytest.mark.parametrize(
        "frame_a,frame_b",
        _FRAME_PAIRS,
        ids=[f"{a}__{b}".replace("::", "_") for a, b in _FRAME_PAIRS],
    )
    def test_returns_finite_well_formed_pose(self, viz_client, frame_a, frame_b):
        """
        calc_pose with an override must return a finite, well-formed pose for every frame combination.
        """
        pose = viz_client.calc_pose(frame_a, frame_b, self._make_override())
        assert pose.frame_A == frame_a
        assert pose.frame_B == frame_b
        assert all(math.isfinite(v) for v in pose.translation)
        assert all(math.isfinite(v) for v in pose.quaternion_wxyz)
        norm_sq = sum(v**2 for v in pose.quaternion_wxyz)
        assert abs(norm_sq - 1.0) < 1e-6

    def test_pose_differs_from_default(self, viz_client):
        """
        The override must actually change the returned pose.
        """
        frame_a = "world"
        frame_b = "a::stamp"
        default_pose = viz_client.calc_pose(frame_a, frame_b)
        overridden_pose = viz_client.calc_pose(frame_a, frame_b, self._make_override())
        assert not np.allclose(
            np.array(default_pose.translation),
            np.array(overridden_pose.translation),
            atol=1e-3,
        ), "Pose did not change when a joint-position override was applied"


# ---------------------------------------------------------------------------
# toggle_frame — path resolution
# ---------------------------------------------------------------------------

# A frame that is guaranteed to exist in the loaded lifty model.
_KNOWN_FRAME = "a::stamp"
# Its canonical Meshcat path (model instance "a", frame "stamp").
_KNOWN_FRAME_ABS_PATH = "/drake/frames/a/stamp"


@pytest.mark.parametrize("visible", [True, False], ids=["on", "off"])
def test_toggle_frame_bare_name(viz_client, visible):
    """
    Case 3: bare name (no '/') is resolved via the loaded model.
    """
    viz_client.toggle_frame(_KNOWN_FRAME, visible)


@pytest.mark.parametrize("visible", [True, False], ids=["on", "off"])
def test_toggle_frame_absolute_path(viz_client, visible):
    """
    Case 1: absolute path under /drake/frames/ is used verbatim.
    """
    viz_client.toggle_frame(_KNOWN_FRAME_ABS_PATH, visible)


@pytest.mark.parametrize("visible", [True, False], ids=["on", "off"])
def test_toggle_frame_relative_with_frames_prefix(viz_client, visible):
    """
    Case 2a: relative path starting with "frames/" – "/drake/" is prepended.
    Equivalent to _KNOWN_FRAME_ABS_PATH.
    """
    viz_client.toggle_frame("frames/a/stamp", visible)


@pytest.mark.parametrize("visible", [True, False], ids=["on", "off"])
def test_toggle_frame_relative_without_frames_prefix(viz_client, visible):
    """
    Case 2b: relative path without "frames/" prefix – "/drake/frames/" is
    prepended.  "a/stamp" → /drake/frames/a/stamp/.
    """
    viz_client.toggle_frame("a/stamp", visible)


def test_toggle_frame_roundtrip(viz_client):
    """
    Toggling off then on should not raise for any supported input form.
    """
    for frame in (_KNOWN_FRAME, _KNOWN_FRAME_ABS_PATH, "frames/a/stamp", "a/stamp"):
        viz_client.toggle_frame(frame, False)
        viz_client.toggle_frame(frame, True)


def test_toggle_frame_empty_name_raises(viz_client):
    """
    An empty frame string must be rejected by the server.
    """
    with pytest.raises(RuntimeError):
        viz_client.toggle_frame("", True)


def test_toggle_frame_wrong_subtree_raises(viz_client):
    """
    An absolute path not under /drake/frames/ must be rejected.
    """
    with pytest.raises(RuntimeError):
        viz_client.toggle_frame("/drake/objects/some_object", True)


def test_toggle_frame_unknown_bare_name_raises(viz_client):
    """
    A bare name that does not exist in the loaded model must be rejected.
    """
    with pytest.raises(RuntimeError):
        viz_client.toggle_frame("this_frame_does_not_exist_xyz", True)
