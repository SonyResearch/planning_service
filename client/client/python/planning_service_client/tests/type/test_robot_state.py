import pytest
from planning_service_client.type.robot_state import (
    FrameRelativePose,
    Quaternion,
    RigidTransform,
    Deprecated_State,
    SystemConf,
    SystemConfEdge,
)


def test_sysconf():
    empty = SystemConf()
    assert empty.empty()
    proto = empty.to_proto()
    assert len(proto.data.items()) == 0
    assert SystemConf.from_proto(proto) == empty

    nonempty = empty
    nonempty.add("robot", [0, 1, 2])
    assert nonempty._data is not None
    proto = nonempty.to_proto()
    assert "robot" in proto.data
    assert proto.data["robot"].data == [0, 1, 2]
    assert SystemConf.from_proto(proto) == nonempty


@pytest.fixture
def sysconf_0():
    return SystemConf({"robot": [0, 1, 2]})


@pytest.fixture
def sysconf_1():
    return SystemConf({"robot": [2, 1, 0]})


@pytest.mark.parametrize("u", [SystemConf(), None, [0, 1, 2], sysconf_0])
@pytest.mark.parametrize("v", [SystemConf(), None, [2, 1, 0], sysconf_1])
def test_sysconf_edge_ctor(u, v):
    if not isinstance(u, SystemConf) or not isinstance(v, SystemConf) or u.empty() or v.empty():
        with pytest.raises(ValueError):
            edge = SystemConfEdge(u, v)
    else:
        assert edge._u == sysconf_0
        assert edge._v == sysconf_1


def test_sysconf_edge_proto(sysconf_0, sysconf_1):
    edge = SystemConfEdge(sysconf_0, sysconf_1)
    proto = edge.to_proto()
    assert proto.u == sysconf_0.to_proto()
    assert proto.v == sysconf_1.to_proto()
    assert SystemConfEdge.from_proto(proto) == edge


@pytest.fixture
def quat():
    return Quaternion([1, 0, 0, 0])


@pytest.fixture
def rigid_transform(quat):
    return RigidTransform([0, 0, 0], quat)


def test_quaternion(quat):
    proto = quat.to_proto()
    assert proto.w == quat._wxyz[0]
    assert proto.x == quat._wxyz[1]
    assert proto.y == quat._wxyz[2]
    assert proto.z == quat._wxyz[3]
    assert Quaternion.from_proto(proto) == quat


def test_rigid_transform(quat, rigid_transform):
    with pytest.raises(ValueError):
        transform = RigidTransform([0, 0], quat)

    transform = rigid_transform
    proto = transform.to_proto()
    assert transform._translation[0] == proto.translation.x
    assert transform._translation[1] == proto.translation.y
    assert transform._translation[2] == proto.translation.z
    assert RigidTransform.from_proto(proto) == transform


@pytest.fixture
def pose(rigid_transform):
    return FrameRelativePose("world", "eef", rigid_transform)


def test_pose(rigid_transform, pose):
    with pytest.raises(ValueError):
        pose = FrameRelativePose("", "eef", rigid_transform)
    with pytest.raises(ValueError):
        pose = FrameRelativePose("world", "", rigid_transform)
    proto = pose.to_proto()
    assert proto.frame_A == pose._frame_A
    assert proto.frame_B == pose._frame_B
    assert proto.X_AB == pose._X_AB.to_proto()
    assert FrameRelativePose.from_proto(proto) == pose


def test_state(sysconf_0, pose):
    with pytest.raises(ValueError):
        Deprecated_State()
    with pytest.raises(ValueError):
        Deprecated_State(sysconf_0, pose)
    with pytest.raises(TypeError):
        Deprecated_State(system_conf=pose)
    with pytest.raises(TypeError):
        Deprecated_State(pose=sysconf_0)
    with pytest.raises(TypeError):
        Deprecated_State.make_joint_state(pose)
    with pytest.raises(TypeError):
        Deprecated_State.make_pose_state(sysconf_0)

    sysconf_state = Deprecated_State(system_conf=sysconf_0)
    assert sysconf_state._pose is None
    assert sysconf_state._system_conf == sysconf_0
    proto = sysconf_state.to_proto()
    assert proto.HasField("system_conf")
    assert proto.system_conf == sysconf_0.to_proto()
    assert Deprecated_State.from_proto(proto) == sysconf_state

    pose_state = Deprecated_State(pose=pose)
    assert pose_state._pose == pose
    assert pose_state._system_conf is None
    proto = pose_state.to_proto()
    assert proto.HasField("pose")
    assert proto.pose == pose.to_proto()
    assert Deprecated_State.from_proto(proto) == pose_state
