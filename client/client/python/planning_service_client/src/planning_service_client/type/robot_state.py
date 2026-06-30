from __future__ import annotations

from typing import List, Mapping, Union

import numpy as np
from planning_service_client.type.proto import ProtoClass
from proto import basic_types_pb2


class SystemConf(ProtoClass):
    """
    System configuration.

    A 'system configuration' maps individual robots to their respective joint positions. Consider a two-robot system,
    where each robot has 4 joints. Instead of representing their position by some
    q(t) = [q_0(t), q_1(t), ..., q_7(t)], we can write
    sysconf(t) = {"robot_0" : [q_00(t), q_01(t), q_02(t), q_03(t)], "robot_1": [q_10(t), q_11(t), q_12(t), q_13(t)]}.
    """

    def __init__(self, data: Union[Mapping[str, np.ndarray], Mapping[str, List[float]]] = None) -> None:
        self._data = None if data is None else {k: np.array(v) for k, v in data.items()}

    def __eq__(self, other: SystemConf) -> bool:
        return (self.empty() and other.empty()) or (
            self._data.keys() == other._data.keys()
            and np.all([np.isclose(v, other._data[k], atol=1e-3) for k, v in self._data.items()])
        )

    def __repr__(self) -> str:
        tkns = []
        if self._data is not None:
            for robot, conf in self._data.items():
                tkns.append(f"{robot}: {conf}")
        return "{ " + ", ".join(tkns) + " }"

    def empty(self) -> bool:
        return self._data is None

    def add(self, robot: str, conf: Union[np.ndarray, List[float]]) -> None:
        """
        Adds a configuration for the given robot.
        """
        if self._data is None:
            self._data = {}
        self._data[robot] = np.array(conf)

    def to_proto(self) -> basic_types_pb2.SystemConf:
        """
        Create a corresponding Protobuf message from the given instance.
        """
        sysconf_data = {}
        if self._data is not None:
            for robot, conf in self._data.items():
                sysconf_data[robot] = basic_types_pb2.Conf(data=conf.tolist())
        return basic_types_pb2.SystemConf(data=sysconf_data)

    @classmethod
    def from_proto(cls, msg: basic_types_pb2.SystemConf) -> SystemConf:
        system_conf = SystemConf()
        for robot, conf in msg.data.items():
            system_conf.add(robot, np.array([x for x in conf.data]))
        return system_conf


class SystemConfEdge(ProtoClass):
    """
    Undirected edge where each vertex is a system configuration.
    """

    def __init__(self, u: SystemConf, v: SystemConf) -> None:
        if not isinstance(u, SystemConf) or not isinstance(v, SystemConf):
            raise ValueError("Vertices 'u' and 'v' must be of type SystemConf.")
        if u.empty() or v.empty():
            raise ValueError("Vertices 'u' and 'v' must be nonempty to be valid.")
        self._u = u
        self._v = v

    def __eq__(self, other: SystemConfEdge) -> bool:
        return self._u == other._u and self._v == other._v

    def __repr__(self) -> str:
        return f"u: {self._u.__repr__()}\nv: {self._v.__repr__()}"

    def to_proto(self) -> basic_types_pb2.SystemConfEdge:
        """
        Create a corresponding Protobuf message from the given instance.
        """
        return basic_types_pb2.SystemConfEdge(u=self._u.to_proto(), v=self._v.to_proto())

    @classmethod
    def from_proto(cls, msg: basic_types_pb2.SystemConfEdge) -> SystemConfEdge:
        """
        Create a new class instance from the given Protobuf message.
        """
        return SystemConfEdge(u=SystemConf.from_proto(msg.u), v=SystemConf.from_proto(msg.v))


class Quaternion(ProtoClass):
    """
    Quaternion (w, x, y, z).
    """

    def __init__(self, wxyz=Union[np.ndarray, List[float]]) -> None:
        self._wxyz = np.array(wxyz)
        if self._wxyz.size != 4:
            raise ValueError(f"Expected erray of size 4, got {self._wxyz.size}")

    def __eq__(self, other: Quaternion) -> bool:
        return np.all(np.isclose(self._wxyz, other._wxyz, atol=1e-3))

    def __repr__(self) -> str:
        return str(self._wxyz)

    def to_proto(self) -> basic_types_pb2.Quaternion:
        """
        Create a corresponding Protobuf message from the given instance.
        """
        return basic_types_pb2.Quaternion(w=self._wxyz[0], x=self._wxyz[1], y=self._wxyz[2], z=self._wxyz[3])

    @classmethod
    def from_proto(cls, msg: basic_types_pb2.Quaternion) -> Quaternion:
        """
        Create a new class instance from the given Protobuf message.
        """
        return Quaternion([msg.w, msg.x, msg.y, msg.z])


class RigidTransform(ProtoClass):
    """
    A relative transformation between two frames expressed as a translation in Cartesian space and a rotation as
    quaternion.
    """

    def __init__(
        self,
        translation: Union[np.ndarray, List[float]],
        quaternion: Union[Quaternion, np.ndarray, List[float]] = None,
    ) -> None:
        self._translation = np.array(translation)
        if self._translation.size != 3:
            raise ValueError(f"Expected erray of size 4, got {self._translation.size}")
        if quaternion is None:
            self._quaternion = Quaternion([1, 0, 0, 0])
        elif isinstance(quaternion, Quaternion):
            self._quaternion = quaternion
        else:
            self._quaternion = Quaternion(quaternion)

    def __repr__(self) -> str:
        return f"xyz= ({self._translation}), q_AB= ({self._quaternion})"

    def __eq__(self, other: RigidTransform) -> bool:
        return (
            np.all(np.isclose(self._translation, other._translation, atol=1e-3))
            and self._quaternion == other._quaternion
        )

    def to_proto(self) -> basic_types_pb2.RigidTransform:
        """
        Create a corresponding Protobuf message from the given instance.
        """
        return basic_types_pb2.RigidTransform(
            translation=basic_types_pb2.Vector3(x=self._translation[0], y=self._translation[1], z=self._translation[2]),
            quat=self._quaternion.to_proto(),
        )

    @classmethod
    def from_proto(cls, msg: basic_types_pb2.RigidTransform) -> RigidTransform:
        """
        Create a new class instance from the given Protobuf message.
        """
        return RigidTransform(
            translation=[msg.translation.x, msg.translation.y, msg.translation.z],
            quaternion=Quaternion.from_proto(msg.quat),
        )


class FrameRelativePose(ProtoClass):
    def __init__(self, frame_A: str, frame_B: str, X_AB: RigidTransform):
        if len(frame_A) == 0 or len(frame_B) == 0:
            raise ValueError("Frames must not be empty.")
        if not isinstance(X_AB, RigidTransform):
            raise TypeError(f"Expected X_AB of type 'RigidTransform', got '{type(X_AB).__name__}'")
        self._frame_A = frame_A
        self._frame_B = frame_B
        self._X_AB = X_AB

    def __repr__(self) -> str:
        return f"A: {self._frame_A}, B: {self._frame_B}, X_AB: {self._X_AB}"

    def __eq__(self, other: FrameRelativePose) -> bool:
        return self._frame_A == other._frame_A and self._frame_B == other._frame_B and self._X_AB == other._X_AB

    def to_proto(self) -> basic_types_pb2.FrameRelativePose:
        return basic_types_pb2.FrameRelativePose(
            frame_A=self._frame_A, frame_B=self._frame_B, X_AB=self._X_AB.to_proto()
        )

    @classmethod
    def from_proto(self, msg: basic_types_pb2.FrameRelativePose) -> FrameRelativePose:
        return FrameRelativePose(
            frame_A=msg.frame_A,
            frame_B=msg.frame_B,
            X_AB=RigidTransform.from_proto(msg.X_AB),
        )


class Deprecated_State(ProtoClass):
    def __init__(
        self,
        system_conf: SystemConf = None,
        pose: FrameRelativePose = None,
    ):
        if system_conf is None and pose is None or system_conf is not None and pose is not None:
            raise ValueError("Exactly one of 'pose' or 'system_conf' must be specified!")

        if system_conf is not None and not isinstance(system_conf, SystemConf):
            raise TypeError(f"Expected type SystemConf, got '{type(system_conf).__name__}'")
        if pose is not None and not isinstance(pose, FrameRelativePose):
            raise TypeError(f"Expected type FrameRelativePose, got '{type(pose).__name__}'")
        self._system_conf = system_conf
        self._pose = pose

    def __eq__(self, other: Deprecated_State) -> bool:
        return self._system_conf == other._system_conf and self._pose == other._pose

    @classmethod
    def make_pose_state(cls, pose: FrameRelativePose) -> Deprecated_State:
        if not isinstance(pose, FrameRelativePose):
            raise TypeError(f"Expected type 'FrameRelativePose', got '{type(pose).__name__}'")
        return Deprecated_State(system_conf=None, pose=pose)

    @classmethod
    def make_joint_state(cls, system_conf: SystemConf) -> Deprecated_State:
        if not isinstance(system_conf, SystemConf):
            raise TypeError(f"Expected type 'SystemConf', got '{type(system_conf).__name__}'")
        return Deprecated_State(system_conf=system_conf, pose=None)

    def system_conf(self) -> SystemConf:
        return self._system_conf

    def pose(self) -> FrameRelativePose:
        return self._pose

    def to_proto(self) -> basic_types_pb2.Deprecated_State:
        """
        Create a corresponding Protobuf message from the given instance.
        """
        if self._system_conf is not None:
            return basic_types_pb2.Deprecated_State(system_conf=self._system_conf.to_proto(), pose=None)
        elif self._pose is not None:
            return basic_types_pb2.Deprecated_State(
                system_conf=None,
                pose=self._pose.to_proto(),
            )

    @classmethod
    def from_proto(cls, msg: basic_types_pb2.Deprecated_State) -> Deprecated_State:
        """
        Create a new class instance from the given Protobuf message.
        """
        return Deprecated_State(
            system_conf=(None if not msg.HasField("system_conf") else SystemConf.from_proto(msg.system_conf)),
            pose=(None if not msg.HasField("pose") else FrameRelativePose.from_proto(msg.pose)),
        )
