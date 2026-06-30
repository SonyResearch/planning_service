"""
Tests for the file I/O methods added to native pybind11 types.

Covers save_to_json / load_from_json_file and
save_to_file / load_from_file (text and binary) for:
  - ContextId
  - Conf
  - SystemConf
  - FrameRelativePose
"""

import json
import math

import numpy as np
import pytest

import planning_service_client.native.types as types

# ──────────────────────────────────────────────────────────────────────────────
# Fixtures
# ──────────────────────────────────────────────────────────────────────────────


@pytest.fixture
def context_id():
    return types.ContextId(42, "my_system")


@pytest.fixture
def conf():
    return types.Conf(np.array([0.0, 1.0, -1.0, math.pi]))


@pytest.fixture
def system_conf():
    return types.SystemConf(
        {
            "arm": types.Conf(np.array([1.0, 2.0, 3.0])),
            "hand": types.Conf(np.array([0.0])),
        }
    )


@pytest.fixture
def frame_relative_pose():
    s = math.sqrt(2.0) / 2.0
    return types.FrameRelativePose(
        frame_A="world",
        frame_B="end_effector",
        translation=np.array([1.0, 2.0, 3.0]),
        quaternion_wxyz=np.array([s, 0.0, 0.0, s]),
    )


# ──────────────────────────────────────────────────────────────────────────────
# ContextId
# ──────────────────────────────────────────────────────────────────────────────


class TestContextIdFileIO:
    def test_save_and_load_json(self, context_id, tmp_path):
        path = str(tmp_path / "context_id.json")
        context_id.save_to_json(path)
        recovered = types.ContextId.load_from_json_file(path)
        assert recovered.value == context_id.value
        assert recovered.system == context_id.system

    def test_save_json_produces_valid_json(self, context_id, tmp_path):
        path = tmp_path / "context_id.json"
        context_id.save_to_json(str(path))
        data = json.loads(path.read_text())
        assert int(data["value"]) == context_id.value
        assert data["system"] == context_id.system

    def test_save_and_load_text(self, context_id, tmp_path):
        path = str(tmp_path / "context_id.pb.txt")
        context_id.save_to_file(path, binary=False)
        recovered = types.ContextId.load_from_file(path, binary=False)
        assert recovered.value == context_id.value
        assert recovered.system == context_id.system

    def test_save_and_load_binary(self, context_id, tmp_path):
        path = str(tmp_path / "context_id.pb")
        context_id.save_to_file(path, binary=True)
        recovered = types.ContextId.load_from_file(path, binary=True)
        assert recovered.value == context_id.value
        assert recovered.system == context_id.system

    def test_binary_file_is_not_empty(self, context_id, tmp_path):
        path = tmp_path / "context_id.pb"
        context_id.save_to_file(str(path), binary=True)
        assert path.stat().st_size > 0

    def test_save_to_file_default_is_text(self, context_id, tmp_path):
        path = str(tmp_path / "context_id.pb.txt")
        context_id.save_to_file(path)  # binary defaults to False
        recovered = types.ContextId.load_from_file(path, binary=False)
        assert recovered.value == context_id.value


# ──────────────────────────────────────────────────────────────────────────────
# Conf
# ──────────────────────────────────────────────────────────────────────────────


class TestConfFileIO:
    def test_save_and_load_json(self, conf, tmp_path):
        path = str(tmp_path / "conf.json")
        conf.save_to_json(path)
        recovered = types.Conf.load_from_json_file(path)
        assert conf == recovered

    def test_save_json_produces_valid_json(self, conf, tmp_path):
        path = tmp_path / "conf.json"
        conf.save_to_json(str(path))
        data = json.loads(path.read_text())
        assert "data" in data
        np.testing.assert_array_almost_equal(data["data"], conf.q)

    def test_save_and_load_binary(self, conf, tmp_path):
        path = str(tmp_path / "conf.pb")
        conf.save_to_file(path, binary=True)
        recovered = types.Conf.load_from_file(path, binary=True)
        assert conf == recovered

    def test_save_and_load_text(self, conf, tmp_path):
        path = str(tmp_path / "conf.pb.txt")
        conf.save_to_file(path, binary=False)
        recovered = types.Conf.load_from_file(path, binary=False)
        assert conf == recovered

    def test_empty_conf_roundtrip_json(self, tmp_path):
        original = types.Conf()
        path = str(tmp_path / "empty_conf.json")
        original.save_to_json(path)
        recovered = types.Conf.load_from_json_file(path)
        assert original == recovered

    def test_empty_conf_roundtrip_binary(self, tmp_path):
        original = types.Conf()
        path = str(tmp_path / "empty_conf.pb")
        original.save_to_file(path, binary=True)
        recovered = types.Conf.load_from_file(path, binary=True)
        assert original == recovered

    @pytest.mark.parametrize(
        "q",
        [
            [0.0],
            [1.0, -1.0],
            [0.0, math.pi, -math.pi / 2],
            list(range(7)),
        ],
    )
    def test_binary_roundtrip_parametrized(self, q, tmp_path):
        original = types.Conf(np.array(q, dtype=float))
        path = str(tmp_path / "conf.pb")
        original.save_to_file(path, binary=True)
        recovered = types.Conf.load_from_file(path, binary=True)
        assert original == recovered


# ──────────────────────────────────────────────────────────────────────────────
# SystemConf
# ──────────────────────────────────────────────────────────────────────────────


class TestSystemConfFileIO:
    def test_save_and_load_json(self, system_conf, tmp_path):
        path = str(tmp_path / "system_conf.json")
        system_conf.save_to_json(path)
        recovered = types.SystemConf.load_from_json_file(path)
        assert system_conf == recovered

    def test_save_json_contains_expected_keys(self, system_conf, tmp_path):
        path = tmp_path / "system_conf.json"
        system_conf.save_to_json(str(path))
        data = json.loads(path.read_text())
        assert "arm" in data["data"]
        assert "hand" in data["data"]

    def test_save_and_load_binary(self, system_conf, tmp_path):
        path = str(tmp_path / "system_conf.pb")
        system_conf.save_to_file(path, binary=True)
        recovered = types.SystemConf.load_from_file(path, binary=True)
        assert system_conf == recovered

    def test_save_and_load_text(self, system_conf, tmp_path):
        path = str(tmp_path / "system_conf.pb.txt")
        system_conf.save_to_file(path, binary=False)
        recovered = types.SystemConf.load_from_file(path, binary=False)
        assert system_conf == recovered

    def test_empty_system_conf_roundtrip_json(self, tmp_path):
        original = types.SystemConf()
        path = str(tmp_path / "empty.json")
        original.save_to_json(path)
        recovered = types.SystemConf.load_from_json_file(path)
        assert original == recovered

    def test_empty_system_conf_roundtrip_binary(self, tmp_path):
        original = types.SystemConf()
        path = str(tmp_path / "empty.pb")
        original.save_to_file(path, binary=True)
        recovered = types.SystemConf.load_from_file(path, binary=True)
        assert original == recovered


# ──────────────────────────────────────────────────────────────────────────────
# FrameRelativePose
# ──────────────────────────────────────────────────────────────────────────────


class TestFrameRelativePoseFileIO:
    def test_save_and_load_json(self, frame_relative_pose, tmp_path):
        path = str(tmp_path / "pose.json")
        frame_relative_pose.save_to_json(path)
        recovered = types.FrameRelativePose.load_from_json_file(path)
        assert frame_relative_pose == recovered

    def test_save_and_load_binary(self, frame_relative_pose, tmp_path):
        path = str(tmp_path / "pose.pb")
        frame_relative_pose.save_to_file(path, binary=True)
        recovered = types.FrameRelativePose.load_from_file(path, binary=True)
        assert frame_relative_pose == recovered

    def test_save_and_load_text(self, frame_relative_pose, tmp_path):
        path = str(tmp_path / "pose.pb.txt")
        frame_relative_pose.save_to_file(path, binary=False)
        recovered = types.FrameRelativePose.load_from_file(path, binary=False)
        assert frame_relative_pose == recovered

    def test_save_json_preserves_frame_names(self, frame_relative_pose, tmp_path):
        path = tmp_path / "pose.json"
        frame_relative_pose.save_to_json(str(path))
        data = json.loads(path.read_text())
        assert data["frame_A"] == frame_relative_pose.frame_A
        assert data["frame_B"] == frame_relative_pose.frame_B

    def test_identity_pose_roundtrip_binary(self, tmp_path):
        original = types.FrameRelativePose(
            frame_A="a",
            frame_B="b",
            translation=np.array([0.0, 0.0, 0.0]),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        path = str(tmp_path / "identity.pb")
        original.save_to_file(path, binary=True)
        recovered = types.FrameRelativePose.load_from_file(path, binary=True)
        assert original == recovered
