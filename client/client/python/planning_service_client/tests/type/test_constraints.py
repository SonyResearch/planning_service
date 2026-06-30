import pytest
from planning_service_client.type.constraints import (
    ConstraintsSet,
    Deprecated_AngleBetweenVectorsConstraint,
    Deprecated_PositionConstraint,
)


@pytest.fixture
def angle_constraint():
    return Deprecated_AngleBetweenVectorsConstraint("frame_A", "frame_B", [0, 0, 1], [0, 0, 1], 0, 0.5)


@pytest.fixture
def position_constraint():
    return Deprecated_PositionConstraint("frame_A", "frame_B", [0, 0, 0], [1, 1, 1], [0, 0, 0])


@pytest.mark.parametrize("frame_A", ["frame_A", ""])
@pytest.mark.parametrize("frame_B", ["frame_B", ""])
@pytest.mark.parametrize("a_A", [[1, 1, 1], [1, 1], []])
@pytest.mark.parametrize("b_B", [[0, 0, 0], [0, 0], []])
@pytest.mark.parametrize("angle_lower", [0.3, 0, -0.3])
@pytest.mark.parametrize("angle_upper", [0.5, 0.3, 0.1])
def test_angle_constraint_ctor(frame_A, frame_B, a_A, b_B, angle_lower, angle_upper):
    if (
        len(frame_A) == 0
        or len(frame_B) == 0
        or len(a_A) != 3
        or len(b_B) != 3
        or angle_lower < 0
        or angle_upper < angle_lower
    ):
        with pytest.raises(ValueError):
            constraint = Deprecated_AngleBetweenVectorsConstraint(frame_A, frame_B, a_A, b_B, angle_lower, angle_upper)
    else:
        constraint = Deprecated_AngleBetweenVectorsConstraint(frame_A, frame_B, a_A, b_B, angle_lower, angle_upper)
        assert constraint._frame_A == frame_A
        assert constraint._frame_B == frame_B
        assert constraint._a_A == a_A
        assert constraint._b_B == b_B
        assert constraint._angle_lower == angle_lower
        assert constraint._angle_upper == angle_upper


def test_angle_constraint_proto(angle_constraint):
    proto = angle_constraint.to_proto()
    assert proto.frame_A == angle_constraint._frame_A
    assert proto.frame_B == angle_constraint._frame_B
    assert all([x == y for x, y in zip(proto.a_A, angle_constraint._a_A)])
    assert all([x == y for x, y in zip(proto.b_B, angle_constraint._b_B)])
    assert proto.angle_lower == angle_constraint._angle_lower
    assert proto.angle_upper == angle_constraint._angle_upper

    converted = Deprecated_AngleBetweenVectorsConstraint.from_proto(proto)
    assert converted._frame_A == angle_constraint._frame_A
    assert converted._frame_B == angle_constraint._frame_B
    assert converted._a_A == angle_constraint._a_A
    assert converted._b_B == angle_constraint._b_B
    assert converted._angle_lower == angle_constraint._angle_lower
    assert converted._angle_upper == angle_constraint._angle_upper


@pytest.mark.parametrize("frame_A", ["frame_A", ""])
@pytest.mark.parametrize("frame_B", ["frame_B", ""])
@pytest.mark.parametrize("p_AQ_lower", [[1, 1, 1], [1, 1], []])
@pytest.mark.parametrize("p_AQ_upper", [[0, 0, 0], [0, 0], []])
@pytest.mark.parametrize("p_BQ", [[0, 0, 0], [0, 0], []])
def test_position_constraint_ctor(frame_A, frame_B, p_AQ_lower, p_AQ_upper, p_BQ):
    if len(frame_A) == 0 or len(frame_B) == 0 or len(p_AQ_lower) != 3 or len(p_AQ_upper) != 3 or len(p_BQ) != 3:
        with pytest.raises(ValueError):
            constraint = Deprecated_PositionConstraint(frame_A, frame_B, p_AQ_lower, p_AQ_upper, p_BQ)
    else:
        constraint = Deprecated_PositionConstraint(frame_A, frame_B, p_AQ_lower, p_AQ_upper, p_BQ)
        assert constraint._frame_A == frame_A
        assert constraint._frame_B == frame_B
        assert constraint._p_AQ_lower == p_AQ_lower
        assert constraint._p_AQ_upper == p_AQ_upper
        assert constraint._p_BQ == p_BQ


def test_position_constraint_proto(position_constraint):
    proto = position_constraint.to_proto()
    assert proto.frame_A == position_constraint._frame_A
    assert proto.frame_B == position_constraint._frame_B
    assert all([x == y for x, y in zip(proto.p_AQ_lower, position_constraint._p_AQ_lower)])
    assert all([x == y for x, y in zip(proto.p_AQ_upper, position_constraint._p_AQ_upper)])
    assert all([x == y for x, y in zip(proto.p_BQ, position_constraint._p_BQ)])

    converted = Deprecated_PositionConstraint.from_proto(proto)
    assert converted._frame_A == position_constraint._frame_A
    assert converted._frame_B == position_constraint._frame_B
    assert converted._p_AQ_lower == position_constraint._p_AQ_lower
    assert converted._p_AQ_upper == position_constraint._p_AQ_upper
    assert converted._p_BQ == position_constraint._p_BQ


@pytest.mark.parametrize(
    "position_constraints",
    [[position_constraint], position_constraint, [], None],
)
@pytest.mark.parametrize("angle_constraints", [[angle_constraint], angle_constraint, [], None])
def test_constraints_set_ctor(position_constraints, angle_constraints):
    if (isinstance(position_constraints, list) and len(position_constraints) == 0) or (
        isinstance(angle_constraints, list) and len(angle_constraints) == 0
    ):
        with pytest.raises(ValueError):
            constraints_set = ConstraintsSet(position_constraints, angle_constraints)
    else:
        constraints_set = ConstraintsSet(position_constraints, angle_constraints)
        if isinstance(position_constraints, Deprecated_PositionConstraint):
            assert constraints_set._position_constraints == [position_constraints]
        else:
            assert constraints_set._position_constraints == position_constraints
        if isinstance(angle_constraints, Deprecated_AngleBetweenVectorsConstraint):
            assert constraints_set._angle_constraints == [angle_constraints]
        else:
            assert constraints_set._angle_constraints == angle_constraints


def test_add_constraints(position_constraint, angle_constraint):
    constraints_set = ConstraintsSet()
    assert constraints_set._position_constraints is None
    assert constraints_set._angle_constraints is None
    # add position constraint
    constraints_set.add(position_constraint)
    assert len(constraints_set._position_constraints) == 1
    assert position_constraint == constraints_set._position_constraints[0]
    # add angle constraint
    constraints_set.add(angle_constraint)
    assert len(constraints_set._angle_constraints) == 1
    assert angle_constraint == constraints_set._angle_constraints[0]
    # none type
    with pytest.raises(ValueError):
        constraints_set.add(None)

    constraints_set = ConstraintsSet()
    constraints_set.add([])
    assert constraints_set._position_constraints is None
    assert constraints_set._angle_constraints is None
    constraints_set.add([position_constraint, angle_constraint])
    assert len(constraints_set._angle_constraints) == 1
    assert len(constraints_set._position_constraints) == 1
    assert position_constraint == constraints_set._position_constraints[0]
    assert angle_constraint == constraints_set._angle_constraints[0]


def test_constraints_set_proto(position_constraint, angle_constraint):
    constraints_set = ConstraintsSet()
    proto = constraints_set.to_proto()
    assert len(proto.angle_constraints) == 0
    assert len(proto.pos_constraints) == 0

    constraints_set.add([angle_constraint, position_constraint])
    proto = constraints_set.to_proto()
    assert len(proto.angle_constraints) == 1
    assert len(proto.pos_constraints) == 1
    assert Deprecated_AngleBetweenVectorsConstraint.from_proto(proto.angle_constraints[0]) == angle_constraint
    assert Deprecated_PositionConstraint.from_proto(proto.pos_constraints[0]) == position_constraint
    assert ConstraintsSet.from_proto(proto) == constraints_set
