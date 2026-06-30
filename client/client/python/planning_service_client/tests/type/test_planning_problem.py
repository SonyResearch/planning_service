import pytest
from planning_service_client.type.planning_problem import PlanningProblem
from planning_service_client.type.robot_state import FrameRelativePose, RigidTransform, Deprecated_State, SystemConf


@pytest.fixture
def sysconf_0():
    return SystemConf({"robot": [0, 1, 2]})


@pytest.fixture
def sysconf_1():
    return SystemConf({"robot": [2, 1, 0]})


@pytest.fixture
def rigid_transform_0():
    return RigidTransform([0, 0, 0])


@pytest.fixture
def rigid_transform_1():
    return RigidTransform([1, 0, 0])


def test_planning_problem(sysconf_0, sysconf_1, rigid_transform_0, rigid_transform_1):
    with pytest.raises(TypeError):
        PlanningProblem("name", sysconf_0, sysconf_1, 1000)

    with pytest.raises(ValueError):
        PlanningProblem(
            "name",
            1000,
            Deprecated_State.make_pose_state(FrameRelativePose("world", "eef", rigid_transform_0)),
            Deprecated_State.make_joint_state(sysconf_0),
            linear=False,
        )

    pose_pose = PlanningProblem.pose_to_pose(
        "name",
        1000,
        FrameRelativePose("world", "eef", rigid_transform_0),
        FrameRelativePose("world", "eef", rigid_transform_1),
        sysconf_0,
        linear=False,
    )
    proto = pose_pose.to_proto()
    assert proto.name == pose_pose._name
    assert proto.context_id.value == pose_pose._context_id
    assert proto.goal == pose_pose._goal.to_proto()
    assert proto.start == pose_pose._start.to_proto()
    assert proto.ik_seed == pose_pose._ik_seed.to_proto()
    assert PlanningProblem.from_proto(proto) == pose_pose
