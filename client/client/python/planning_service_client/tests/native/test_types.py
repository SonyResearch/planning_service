"""
Unit tests for the native pybind11 `types` sub-module.

Covers:
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
# ContextId
# ──────────────────────────────────────────────────────────────────────────────


class TestContextId:
    def test_zero_value_is_falsy(self):
        cid = types.ContextId(0)
        assert not bool(cid)

    def test_nonzero_value_is_truthy(self):
        cid = types.ContextId(42)
        assert bool(cid)

    def test_members(self):
        cid = types.ContextId(7, "my_system")
        assert cid.value == 7
        assert cid.system == "my_system"

    def test_default_system_is_empty_string(self):
        cid = types.ContextId(1)
        assert cid.system == ""

    def test_to_json(self):
        cid = types.ContextId(5, "bot")
        data = json.loads(cid.to_json())
        assert isinstance(data, dict)
        assert data.get("system") == "bot"
        assert int(data.get("value")) == 5

    @pytest.mark.parametrize(
        "value,system",
        [
            (0, ""),
            (1, "a"),
            (2**32 - 1, "large_value"),
        ],
    )
    def test_proto_roundtrip(self, value, system):
        original = types.ContextId(value, system)
        recovered = types.ContextId.from_proto_bytes(original.to_proto_bytes())
        assert recovered.value == original.value
        assert recovered.system == original.system


# ──────────────────────────────────────────────────────────────────────────────
# Conf
# ──────────────────────────────────────────────────────────────────────────────


class TestConf:
    def test_default_ctor_produces_empty_q(self):
        c = types.Conf()
        assert len(c.q) == 0

    def test_ctor_with_vector(self):
        q = np.array([1.0, 2.0, 3.0])
        c = types.Conf(q)
        np.testing.assert_array_almost_equal(c.q, q)

    def test_equality_same_values(self):
        q = np.array([0.1, 0.2, 0.3])
        assert types.Conf(q) == types.Conf(q)

    def test_inequality_different_values(self):
        assert types.Conf(np.array([1.0])) != types.Conf(np.array([2.0]))

    def test_inequality_different_sizes(self):
        assert types.Conf(np.array([1.0, 2.0])) != types.Conf(np.array([1.0, 2.0, 3.0]))

    def test_default_confs_are_equal(self):
        assert types.Conf() == types.Conf()

    def test_to_json(self):
        q = np.array([1.0, 2.0])
        c = types.Conf(q)
        data = json.loads(c.to_json())
        assert isinstance(data, dict)
        assert "data" in data
        assert data["data"] == pytest.approx(q)

    @pytest.mark.parametrize(
        "q",
        [
            [0.0],
            [1.0, -1.0],
            [0.0, math.pi, -math.pi / 2],
            list(range(7)),
        ],
    )
    def test_proto_roundtrip(self, q):
        original = types.Conf(np.array(q, dtype=float))
        recovered = types.Conf.from_proto_bytes(original.to_proto_bytes())
        assert original == recovered

    def test_proto_roundtrip_empty(self):
        original = types.Conf()
        recovered = types.Conf.from_proto_bytes(original.to_proto_bytes())
        assert original == recovered


# ──────────────────────────────────────────────────────────────────────────────
# SystemConf
# ──────────────────────────────────────────────────────────────────────────────


@pytest.fixture
def simple_system_conf():
    return types.SystemConf(
        {
            "robot_a": types.Conf(np.array([1.0, 2.0, 3.0])),
            "robot_b": types.Conf(np.array([0.0])),
        }
    )


class TestSystemConf:
    def test_default_ctor_is_empty(self):
        sc = types.SystemConf()
        assert sc.empty()
        assert sc.size() == 0

    def test_ctor_with_dict_not_empty(self, simple_system_conf):
        assert not simple_system_conf.empty()

    def test_ctor_with_dict_correct_size(self, simple_system_conf):
        assert simple_system_conf.size() == 2

    def test_contains_existing_key(self, simple_system_conf):
        assert simple_system_conf.contains("robot_a")
        assert simple_system_conf.contains("robot_b")

    def test_contains_missing_key(self, simple_system_conf):
        assert not simple_system_conf.contains("robot_c")

    def test_at_returns_correct_conf(self):
        conf = types.Conf(np.array([1.0, 2.0]))
        sc = types.SystemConf({"r": conf})
        assert sc.at("r") == conf

    def test_set_adds_entry(self):
        sc = types.SystemConf()
        sc.set("robot", types.Conf(np.array([5.0, 6.0])))
        assert sc.contains("robot")
        assert sc.size() == 1

    def test_set_stores_correct_value(self):
        sc = types.SystemConf()
        conf = types.Conf(np.array([5.0, 6.0]))
        sc.set("robot", conf)
        assert sc.at("robot") == conf

    def test_set_overwrites_existing_entry(self):
        sc = types.SystemConf({"r": types.Conf(np.array([1.0]))})
        new_conf = types.Conf(np.array([99.0]))
        sc.set("r", new_conf)
        assert sc.at("r") == new_conf

    def test_keys(self, simple_system_conf):
        assert sorted(simple_system_conf.keys()) == ["robot_a", "robot_b"]

    def test_keys_empty(self):
        sc = types.SystemConf()
        assert sc.keys() == []

    def test_values(self, simple_system_conf):
        conf_a = types.Conf(np.array([1.0, 2.0, 3.0]))
        conf_b = types.Conf(np.array([0.0]))
        sc = types.SystemConf({"robot_a": conf_a, "robot_b": conf_b})
        values = sc.values()
        assert len(values) == 2
        assert conf_a in values
        assert conf_b in values

    def test_values_empty(self):
        sc = types.SystemConf()
        assert sc.values() == []

    def test_items(self, simple_system_conf):
        conf_a = types.Conf(np.array([1.0, 2.0, 3.0]))
        conf_b = types.Conf(np.array([0.0]))
        sc = types.SystemConf({"robot_a": conf_a, "robot_b": conf_b})
        items = sc.items()
        assert len(items) == 2
        assert ("robot_a", conf_a) in items
        assert ("robot_b", conf_b) in items

    def test_equality_same(self, simple_system_conf):
        copy = types.SystemConf(
            {
                "robot_a": types.Conf(np.array([1.0, 2.0, 3.0])),
                "robot_b": types.Conf(np.array([0.0])),
            }
        )
        assert simple_system_conf == copy

    def test_equality_empty(self):
        assert types.SystemConf() == types.SystemConf()

    def test_inequality_different_data(self, simple_system_conf):
        other = types.SystemConf({"robot_a": types.Conf(np.array([9.0]))})
        assert simple_system_conf != other

    def test_proto_roundtrip(self, simple_system_conf):
        recovered = types.SystemConf.from_proto_bytes(simple_system_conf.to_proto_bytes())
        assert simple_system_conf == recovered

    def test_proto_roundtrip_empty(self):
        sc = types.SystemConf()
        recovered = types.SystemConf.from_proto_bytes(sc.to_proto_bytes())
        assert sc == recovered

    def test_to_json(self, simple_system_conf):
        data = json.loads(simple_system_conf.to_json())
        assert isinstance(data, dict)
        assert "data" in data
        assert "robot_a" in data["data"]
        assert "robot_b" in data["data"]


# ──────────────────────────────────────────────────────────────────────────────
# FrameRelativePose
# ──────────────────────────────────────────────────────────────────────────────


@pytest.fixture
def identity_pose():
    return types.FrameRelativePose(
        frame_A="world",
        frame_B="robot",
        translation=np.array([0.0, 0.0, 0.0]),
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
    )


class TestFrameRelativePose:
    def test_default_ctor_has_all_properties(self):
        pose = types.FrameRelativePose()
        assert hasattr(pose, "frame_A")
        assert hasattr(pose, "frame_B")
        assert hasattr(pose, "translation")
        assert hasattr(pose, "quaternion_wxyz")

    def test_frame_A(self, identity_pose):
        assert identity_pose.frame_A == "world"

    def test_frame_B(self, identity_pose):
        assert identity_pose.frame_B == "robot"

    def test_identity_translation(self, identity_pose):
        np.testing.assert_array_almost_equal(identity_pose.translation, [0.0, 0.0, 0.0])

    def test_identity_quaternion(self, identity_pose):
        np.testing.assert_array_almost_equal(identity_pose.quaternion_wxyz, [1.0, 0.0, 0.0, 0.0])

    def test_non_trivial_translation(self):
        pose = types.FrameRelativePose(
            frame_A="a",
            frame_B="b",
            translation=np.array([1.0, 2.0, 3.0]),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        np.testing.assert_array_almost_equal(pose.translation, [1.0, 2.0, 3.0])

    def test_non_trivial_rotation(self):
        # 90-degree rotation about z-axis: w=cos(45°), z=sin(45°)
        s = math.sqrt(2.0) / 2.0
        pose = types.FrameRelativePose(
            frame_A="a",
            frame_B="b",
            translation=np.array([0.0, 0.0, 0.0]),
            quaternion_wxyz=np.array([s, 0.0, 0.0, s]),
        )
        np.testing.assert_array_almost_equal(pose.quaternion_wxyz, [s, 0.0, 0.0, s])

    def test_equality_identical(self, identity_pose):
        other = types.FrameRelativePose(
            frame_A="world",
            frame_B="robot",
            translation=np.array([0.0, 0.0, 0.0]),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        assert identity_pose == other

    def test_inequality_different_translation(self, identity_pose):
        other = types.FrameRelativePose(
            frame_A="world",
            frame_B="robot",
            translation=np.array([1.0, 0.0, 0.0]),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        assert identity_pose != other

    def test_inequality_different_frame_A(self, identity_pose):
        other = types.FrameRelativePose(
            frame_A="other_frame",
            frame_B="robot",
            translation=np.array([0.0, 0.0, 0.0]),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        assert identity_pose != other

    def test_proto_roundtrip(self, identity_pose):
        recovered = types.FrameRelativePose.from_proto_bytes(identity_pose.to_proto_bytes())
        assert identity_pose == recovered

    def test_proto_roundtrip_with_translation_and_rotation(self):
        s = math.sqrt(2.0) / 2.0
        original = types.FrameRelativePose(
            frame_A="base",
            frame_B="end_effector",
            translation=np.array([0.5, -0.1, 1.2]),
            quaternion_wxyz=np.array([s, 0.0, 0.0, s]),
        )
        recovered = types.FrameRelativePose.from_proto_bytes(original.to_proto_bytes())
        assert original == recovered

    def test_to_json_returns_valid_json(self, identity_pose):
        data = json.loads(identity_pose.to_json())
        assert isinstance(data, dict)

    def test_to_json_contains_frame_names(self, identity_pose):
        data = json.loads(identity_pose.to_json())
        assert data.get("frame_A") == "world"
        assert data.get("frame_B") == "robot"


# ──────────────────────────────────────────────────────────────────────────────
# Rgba
# ──────────────────────────────────────────────────────────────────────────────


class TestRgba:
    def test_default_ctor(self):
        color = types.Rgba()
        assert color.r == pytest.approx(0.0)
        assert color.g == pytest.approx(0.0)
        assert color.b == pytest.approx(0.0)
        assert color.a == pytest.approx(1.0)

    def test_ctor_with_alpha_default(self):
        color = types.Rgba(0.1, 0.2, 0.3)
        assert color.r == pytest.approx(0.1)
        assert color.g == pytest.approx(0.2)
        assert color.b == pytest.approx(0.3)
        assert color.a == pytest.approx(1.0)

    def test_ctor_with_explicit_alpha(self):
        color = types.Rgba(0.1, 0.2, 0.3, 0.4)
        assert color.r == pytest.approx(0.1)
        assert color.g == pytest.approx(0.2)
        assert color.b == pytest.approx(0.3)
        assert color.a == pytest.approx(0.4)

    @pytest.mark.parametrize(
        "name,expected",
        [
            ("red", [1.0, 0.0, 0.0, 1.0]),
            ("green", [0.0, 1.0, 0.0, 1.0]),
            ("blue", [0.0, 0.0, 1.0, 1.0]),
            ("white", [1.0, 1.0, 1.0, 1.0]),
            ("black", [0.0, 0.0, 0.0, 1.0]),
        ],
    )
    def test_named_colors(self, name, expected):
        color = getattr(types.Rgba, name)()
        actual = [color.r, color.g, color.b, color.a]
        np.testing.assert_allclose(actual, expected)

    def test_named_color_with_custom_alpha(self):
        color = types.Rgba.red(0.25)
        assert color.r == pytest.approx(1.0)
        assert color.g == pytest.approx(0.0)
        assert color.b == pytest.approx(0.0)
        assert color.a == pytest.approx(0.25)

    def test_equality(self):
        a = types.Rgba(0.1, 0.2, 0.3, 0.4)
        b = types.Rgba(0.1, 0.2, 0.3, 0.4)
        c = types.Rgba(0.9, 0.2, 0.3, 0.4)
        assert a == b
        assert a != c

    def test_proto_roundtrip(self):
        original = types.Rgba(0.25, 0.5, 0.75, 0.9)
        recovered = types.Rgba.from_proto_bytes(original.to_proto_bytes())
        assert recovered.r == pytest.approx(original.r)
        assert recovered.g == pytest.approx(original.g)
        assert recovered.b == pytest.approx(original.b)
        assert recovered.a == pytest.approx(original.a)

    def test_str_returns_string(self):
        color = types.Rgba(0.1, 0.2, 0.3, 0.4)
        assert isinstance(str(color), str)
        assert len(str(color)) > 0

    def test_repr_contains_class_name(self):
        color = types.Rgba(0.1, 0.2, 0.3, 0.4)
        assert repr(color).startswith("Rgba(")

    def test_repr_ends_with_paren(self):
        color = types.Rgba(0.1, 0.2, 0.3, 0.4)
        assert repr(color).endswith(")")


# ──────────────────────────────────────────────────────────────────────────────
# __str__ / __repr__ for Conf, ContextId, SystemConf, FrameRelativePose
# ──────────────────────────────────────────────────────────────────────────────


class TestStringMethods:
    def test_conf_str(self):
        c = types.Conf(np.array([1.0, 2.0]))
        assert isinstance(str(c), str)
        assert len(str(c)) > 0

    def test_conf_repr_contains_class_name(self):
        c = types.Conf(np.array([1.0, 2.0]))
        assert repr(c).startswith("Conf(")

    def test_context_id_str(self):
        cid = types.ContextId(42, "sys")
        assert isinstance(str(cid), str)
        assert len(str(cid)) > 0

    def test_context_id_repr_contains_class_name(self):
        cid = types.ContextId(42, "sys")
        assert repr(cid).startswith("ContextId(")

    def test_system_conf_repr_contains_class_name(self):
        sc = types.SystemConf()
        assert repr(sc).startswith("SystemConf(")

    def test_frame_relative_pose_repr_contains_class_name(self):
        pose = types.FrameRelativePose(
            frame_A="a",
            frame_B="b",
            translation=np.array([0.0, 0.0, 0.0]),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        assert repr(pose).startswith("FrameRelativePose(")
