"""
Unit tests for newly exposed native shape/color pybind types.

Covers:
  - Rgba
  - Sphere / Cylinder / Capsule / Box
  - ShapeInFrame
"""

import math

import numpy as np
import pytest

import planning_service_client.native.types as types


class TestShapes:
    def test_sphere_roundtrip(self):
        original = types.Sphere(0.6)
        recovered = types.Sphere.from_proto_bytes(original.to_proto_bytes())
        assert recovered.radius == pytest.approx(0.6)

    def test_cylinder_roundtrip(self):
        original = types.Cylinder(0.5, 2.0)
        recovered = types.Cylinder.from_proto_bytes(original.to_proto_bytes())
        assert recovered.radius == pytest.approx(0.5)
        assert recovered.height == pytest.approx(2.0)

    def test_capsule_roundtrip(self):
        original = types.Capsule(0.4, 1.2)
        recovered = types.Capsule.from_proto_bytes(original.to_proto_bytes())
        assert recovered.radius == pytest.approx(0.4)
        assert recovered.height == pytest.approx(1.2)

    def test_box_roundtrip(self):
        original = types.Box(1.0, 2.0, 3.0)
        recovered = types.Box.from_proto_bytes(original.to_proto_bytes())
        assert recovered.width == pytest.approx(1.0)
        assert recovered.depth == pytest.approx(2.0)
        assert recovered.height == pytest.approx(3.0)

    def test_box_cube_factory(self):
        cube = types.Box.cube(1.5)
        assert cube.width == pytest.approx(1.5)
        assert cube.depth == pytest.approx(1.5)
        assert cube.height == pytest.approx(1.5)


@pytest.fixture
def identity_quaternion_wxyz():
    return np.array([1.0, 0.0, 0.0, 0.0])


class TestShapeInFrame:
    def test_default_ctor_has_identity_pose(self, identity_quaternion_wxyz):
        shape = types.ShapeInFrame()
        assert shape.frame == ""
        np.testing.assert_allclose(shape.translation, [0.0, 0.0, 0.0])
        np.testing.assert_allclose(shape.quaternion_wxyz, identity_quaternion_wxyz)

    def test_default_ctor_has_no_shape_set(self):
        shape = types.ShapeInFrame()
        assert not shape.is_sphere
        assert not shape.is_cylinder
        assert not shape.is_capsule
        assert not shape.is_box
        with pytest.raises(RuntimeError):
            _ = shape.shape

    @pytest.mark.parametrize(
        "shape_obj,expected_type",
        [
            (types.Sphere(0.3), types.Sphere),
            (types.Cylinder(0.3, 1.1), types.Cylinder),
            (types.Capsule(0.3, 1.1), types.Capsule),
            (types.Box(0.3, 1.1, 2.2), types.Box),
        ],
    )
    def test_ctor_from_shape_sets_correct_type(self, shape_obj, expected_type):
        shape = types.ShapeInFrame(shape_obj)
        assert isinstance(shape.shape, expected_type)

    def test_setters_update_pose(self):
        shape = types.ShapeInFrame(types.Sphere(0.2))
        shape.set_frame("tool")
        shape.set_translation(np.array([1.0, 2.0, 3.0]))
        s = math.sqrt(2.0) / 2.0
        shape.set_quaternion(np.array([s, 0.0, 0.0, s]))

        assert shape.frame == "tool"
        np.testing.assert_allclose(shape.translation, [1.0, 2.0, 3.0])
        np.testing.assert_allclose(shape.quaternion_wxyz, [s, 0.0, 0.0, s])

    def test_set_shape_updates_type_flags(self):
        shape = types.ShapeInFrame(types.Sphere(0.2))
        assert shape.is_sphere

        shape.set_shape(types.Cylinder(0.4, 1.3))
        assert not shape.is_sphere
        assert shape.is_cylinder
        assert isinstance(shape.shape, types.Cylinder)

        shape.set_shape(types.Capsule(0.4, 1.3))
        assert shape.is_capsule
        assert isinstance(shape.shape, types.Capsule)

        shape.set_shape(types.Box(1.0, 2.0, 3.0))
        assert shape.is_box
        assert isinstance(shape.shape, types.Box)

    def test_typed_accessor_raises_for_wrong_shape(self):
        shape = types.ShapeInFrame(types.Sphere(0.5))
        with pytest.raises(RuntimeError):
            _ = shape.cylinder
        with pytest.raises(RuntimeError):
            _ = shape.box

    def test_static_shape_factories(self):
        sphere = types.ShapeInFrame.make_sphere(0.5)
        assert sphere.is_sphere
        assert sphere.sphere.radius == pytest.approx(0.5)

        cylinder = types.ShapeInFrame.make_cylinder(0.2, 0.8)
        assert cylinder.is_cylinder
        assert cylinder.cylinder.radius == pytest.approx(0.2)
        assert cylinder.cylinder.height == pytest.approx(0.8)

        capsule = types.ShapeInFrame.make_capsule(0.3, 1.4)
        assert capsule.is_capsule
        assert capsule.capsule.radius == pytest.approx(0.3)
        assert capsule.capsule.height == pytest.approx(1.4)

        box = types.ShapeInFrame.make_box(1.0, 2.0, 3.0)
        assert box.is_box
        assert box.box.width == pytest.approx(1.0)
        assert box.box.depth == pytest.approx(2.0)
        assert box.box.height == pytest.approx(3.0)

    def test_proto_roundtrip_preserves_pose_and_shape(self):
        s = math.sqrt(2.0) / 2.0
        original = types.ShapeInFrame(types.Capsule(0.4, 1.7))
        original.set_frame("my_frame")
        original.set_translation(np.array([0.5, -0.5, 2.0]))
        original.set_quaternion(np.array([s, 0.0, 0.0, s]))

        recovered = types.ShapeInFrame.from_proto_bytes(original.to_proto_bytes())
        assert recovered.frame == "my_frame"
        np.testing.assert_allclose(recovered.translation, [0.5, -0.5, 2.0])
        np.testing.assert_allclose(recovered.quaternion_wxyz, [s, 0.0, 0.0, s])
        assert recovered.is_capsule
        assert recovered.capsule.radius == pytest.approx(0.4)
        assert recovered.capsule.height == pytest.approx(1.7)

    def test_default_color_and_name(self):
        shape = types.ShapeInFrame(types.Sphere(0.1))
        assert shape.color.r == pytest.approx(0.0)
        assert shape.color.g == pytest.approx(0.0)
        assert shape.color.b == pytest.approx(0.0)
        assert shape.color.a == pytest.approx(1.0)
        assert shape.name == ""

    def test_set_color_and_name(self):
        shape = types.ShapeInFrame(types.Box(1.0, 2.0, 3.0))
        shape.set_color(types.Rgba(1.0, 0.0, 0.0, 0.5))
        shape.set_name("my_box")
        assert shape.color.r == pytest.approx(1.0)
        assert shape.color.g == pytest.approx(0.0)
        assert shape.color.b == pytest.approx(0.0)
        assert shape.color.a == pytest.approx(0.5)
        assert shape.name == "my_box"

    def test_color_and_name_roundtrip(self):
        original = types.ShapeInFrame(types.Cylinder(0.1, 0.5))
        original.set_frame("base")
        original.set_color(types.Rgba(0.0, 1.0, 0.0, 0.8))
        original.set_name("green_cylinder")

        recovered = types.ShapeInFrame.from_proto_bytes(original.to_proto_bytes())
        assert recovered.color.r == pytest.approx(0.0)
        assert recovered.color.g == pytest.approx(1.0)
        assert recovered.color.b == pytest.approx(0.0)
        assert recovered.color.a == pytest.approx(0.8)
        assert recovered.name == "green_cylinder"


# ──────────────────────────────────────────────────────────────────────────────
# __str__ / __repr__ for shape types
# ──────────────────────────────────────────────────────────────────────────────

_STRING_METHOD_SHAPES = [
    (types.Sphere(0.5), "Sphere"),
    (types.Cylinder(0.3, 1.0), "Cylinder"),
    (types.Capsule(0.3, 1.0), "Capsule"),
    (types.Box(1.0, 2.0, 3.0), "Box"),
    (types.ShapeInFrame(types.Sphere(0.2)), "ShapeInFrame"),
]


@pytest.mark.parametrize("obj,expected_class_name", _STRING_METHOD_SHAPES)
def test_shape_str_returns_nonempty_string(obj, expected_class_name):
    assert isinstance(str(obj), str)
    assert len(str(obj)) > 0


@pytest.mark.parametrize("obj,expected_class_name", _STRING_METHOD_SHAPES)
def test_shape_repr_contains_class_name(obj, expected_class_name):
    assert repr(obj).startswith(expected_class_name + "(")


@pytest.mark.parametrize("obj,expected_class_name", _STRING_METHOD_SHAPES)
def test_shape_repr_ends_with_paren(obj, expected_class_name):
    assert repr(obj).endswith(")")
