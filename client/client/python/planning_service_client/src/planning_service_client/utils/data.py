import sys
from pathlib import Path
from typing import List, Mapping, Optional, Union

import yaml
from planning_service_client.type.constraints import (
    ConstraintsSet,
    Deprecated_AngleBetweenVectorsConstraint,
    Deprecated_PositionConstraint,
)
from planning_service_client.type.plan_context import PlanContext
from planning_service_client.type.planning_problem import PlanningProblem
from planning_service_client.type.robot_state import FrameRelativePose, RigidTransform, SystemConf, SystemConfEdge
from planning_service_client.utils.math import euler_to_quaternion
from proto.basic_types_pb2 import Conf
from proto.basic_types_pb2 import FrameRelativePose as FrameRelativePosePB
from proto.basic_types_pb2 import Model, ModelFile, ModelFormat, Quaternion
from proto.basic_types_pb2 import RigidTransform as RigidTransformPB
from proto.basic_types_pb2 import Scene, SceneMetadata, Vector3

PACKAGE_PATH_PREFIX = "package://"


# YAML overhead for DMDs
class Rotation(yaml.YAMLObject):
    def __init__(self, deg: List[float]):
        self.deg = deg

    def __repr__(self) -> str:
        return "[" + ", ".join([f"{x:.3f}" for x in self.deg]) + "]"

    @classmethod
    def from_yaml(cls, loader: yaml.SafeLoader, node: yaml.nodes.MappingNode):
        return Rotation(**loader.construct_mapping(node))


# Required for safe_load
yaml.SafeLoader.add_constructor("!Rpy", Rotation.from_yaml)


def load_configs_from_yaml(config_yaml_path: Path) -> Mapping[str, SystemConf]:
    with config_yaml_path.open(mode="r") as file:
        yaml_root = yaml.safe_load(file)
    if not isinstance(yaml_root, list):
        raise TypeError("Your data should be provided as a list of system configurations")
    return [SystemConf(data=config) for config in yaml_root]


def load_edges_from_yaml(edge_yaml_path: Path) -> Mapping[str, SystemConf]:
    with edge_yaml_path.open(mode="r") as file:
        yaml_root = yaml.safe_load(file)
    if not isinstance(yaml_root, list):
        raise TypeError("Your data should be provided as a list of system configurations")
    return [SystemConfEdge(u=SystemConf(data=edge["u"]), v=SystemConf(data=edge["v"])) for edge in yaml_root]


def read_yaml_to_dict(yaml_file_path):
    with open(yaml_file_path, "r") as file:
        data = yaml.safe_load(file)
    return data


def load_constraints_from_disk(constraints_path: Path):
    constraints = ConstraintsSet()
    with constraints_path.open(mode="r") as file:
        # load directive to get model names
        constraints_yaml = yaml.safe_load(file)
    if "angle_constraints" in constraints_yaml:
        for ac in constraints_yaml["angle_constraints"]:
            constraints.add(
                Deprecated_AngleBetweenVectorsConstraint(
                    frame_A=ac["frame_A"],
                    frame_B=ac["frame_B"],
                    a_A=ac["a_A"],
                    b_B=ac["b_B"],
                    angle_lower=ac["angle_lower"],
                    angle_upper=ac["angle_upper"],
                )
            )
    if "position_constraints" in constraints_yaml:
        for pc in constraints_yaml["position_constraints"]:
            constraints.add(
                Deprecated_PositionConstraint(
                    frame_A=pc["frame_A"],
                    frame_B=pc["frame_B"],
                    p_AQ_lower=pc["position_AQ_lower"],
                    p_AQ_upper=pc["position_AQ_upper"],
                    p_BQ=pc["position_BQ"],
                )
            )
    return constraints


def load_context_from_disk(
    name: str,
    model_directive_path: Union[Path, str],
    urdf_dir_path: Union[Path, str],
    constraints_path: Optional[Union[Path, str]] = None,
) -> PlanContext:
    """
    Load a fully specified planning context from disk.

    From the provided resources, load all the data required to fully determine a given planning context.

    Args:
        name (str): Name for the context.
        model_directive_path (Path): Path to model directive YAML file.
        urdf_dir_path (Path): Path to URDF directory where URDFs referenced in the model directive can be found.
        constraints_path (Path): Path to constraints YAML file.
    """
    try:
        model_directive_path = Path(model_directive_path)
        urdf_dir_path = Path(urdf_dir_path)
        if constraints_path is not None:
            constraints_path = Path(constraints_path)
    except Exception as e:
        print(f"Failed to resolve paths to resources. {e}")
        sys.exit(1)

    model_directive_raw = ""
    # read raw contents
    with model_directive_path.open(mode="r") as file:
        model_directive_raw = file.read()
    # load directive to get model names
    model_directive_yaml = yaml.safe_load(model_directive_raw)
    models_list = []
    # iterate over models, grab URDFs, read contents
    for entry in model_directive_yaml["directives"]:
        add_model_directive = entry.get("add_model", None)
        if add_model_directive is not None:
            model_file_path = add_model_directive["file"]
            if PACKAGE_PATH_PREFIX not in model_file_path:
                raise NameError(f"URDF paths in your DMD must begin with prefix `{PACKAGE_PATH_PREFIX}`!")
            model_file_path = Path(model_file_path.split(PACKAGE_PATH_PREFIX)[-1])
            parent_path = Path(*model_file_path.parents[1:])
            with urdf_dir_path.joinpath(Path(model_file_path).name).open(mode="rb") as file:
                urdf_raw = file.read()
            file = ModelFile(
                name=model_file_path.stem,
                package_name=str(model_file_path.parents[0]),
                parent_path=str(parent_path),
                content=urdf_raw,
                format=ModelFormat.MODEL_FORMAT_URDF,
            )
            default_joint_positions = {
                k: Conf(data=v) for k, v in add_model_directive["default_joint_positions"].items()
            }
            models_list.append(
                Model(
                    name=add_model_directive["name"],
                    file=file,
                    default_joint_positions=default_joint_positions,
                )
            )
    # add constraints
    use_constraints = constraints_path is not None
    return PlanContext(
        name=name,
        model_directive_raw=model_directive_raw,
        models=models_list,
        constraints=load_constraints_from_disk(constraints_path) if use_constraints else None,
    )


def migrate_dmd(
    model_directive_path: Union[Path, str],
    urdf_dir_path: Union[Path, str],
) -> Scene:
    try:
        model_directive_path = Path(model_directive_path)
        urdf_dir_path = Path(urdf_dir_path)
    except Exception as e:
        print(f"Failed to resolve paths to resources. {e}")
        sys.exit(1)
    # read raw contents
    with model_directive_path.open(mode="rb") as file:
        model_directive_raw = file.read()
    # load directive to get model names
    model_directive = yaml.safe_load(model_directive_raw)
    models = {}
    # iterate over models, grab URDFs, read contents
    for entry in model_directive["directives"]:
        add_model_directive = entry.get("add_model", None)
        if add_model_directive is not None:
            model_file_path = add_model_directive["file"]
            if PACKAGE_PATH_PREFIX not in model_file_path:
                raise NameError(f"URDF paths in your DMD must begin with prefix `{PACKAGE_PATH_PREFIX}`!")
            model_file_path = Path(model_file_path.split(PACKAGE_PATH_PREFIX)[-1])
            parent_path = Path(*model_file_path.parents[1:])
            with urdf_dir_path.joinpath(Path(model_file_path).name).open(mode="rb") as file:
                urdf_raw = file.read()
            file = ModelFile(
                name=model_file_path.stem,
                package_name=str(model_file_path.parents[0]),
                parent_path=str(parent_path),
                content=urdf_raw,
                format=ModelFormat.MODEL_FORMAT_URDF,
            )
            default_joint_positions = {
                k: Conf(data=v) for k, v in add_model_directive["default_joint_positions"].items()
            }
            model = Model(
                name=add_model_directive["name"],
                file=file,
                default_joint_positions=default_joint_positions,
            )
            models[model.name] = model

        add_weld_directive = entry.get("add_weld", None)
        if add_weld_directive is not None:
            [x, y, z] = add_weld_directive["X_PC"]["translation"]
            [roll, pitch, yaw] = add_weld_directive["X_PC"]["rotation"].deg
            [qw, qx, qy, qz] = euler_to_quaternion(yaw, pitch, roll)
            X_AB = RigidTransformPB(translation=Vector3(x=x, y=y, z=z), quat=Quaternion(w=qw, x=qx, y=qy, z=qz))
            pose = FrameRelativePosePB(
                frame_A=add_weld_directive["parent"], frame_B=add_weld_directive["child"], X_AB=X_AB
            )
            model_name = pose.frame_B.split("::")[0]
            models[model_name].pose.CopyFrom(pose)

    metadata = SceneMetadata(name=model_directive_path.stem, description="migrated from DMD")
    return Scene(models=models.values(), metadata=metadata, model_directive_raw=model_directive_raw)


def load_planning_problem_from_yaml(file_path: str, context_id: int) -> PlanningProblem:
    """
    Load a YAML file and create a PlanningProblem object from the data.

    Args:
        file_path (str): Path to the YAML file.
        context_id (int): Unique context ID for the planning problem.

    Returns:
        PlanningProblem: The constructed planning problem object.
    """

    with open(file_path, "r") as file:
        data = yaml.safe_load(file)

    if "frame_A" in data["goal"]:
        frame_A = data["goal"]["frame_A"]
        frame_B = data["goal"]["frame_B"]

        # Extract roll, pitch, and yaw and convert to quaternion (fragile, assumes the format is always the same)
        roll, pitch, yaw = map(float, data["goal"]["X_AB"].split(" ")[2:5])
        quaternion = euler_to_quaternion(roll, pitch, yaw)

        # Get the translation vector (fragile, assumes the format is always the same)
        x, y, z = map(float, data["goal"]["X_AB"].split(" ")[7:10])

        # Create a RigidTransform object
        rigid_transform = RigidTransform(translation=[x, y, z], quaternion=quaternion)

        # Create a FrameRelativePose object
        frame_relative_pose = FrameRelativePose(frame_A=frame_A, frame_B=frame_B, X_AB=rigid_transform)

        # Create the planning problem from system conf to pose
        problem = PlanningProblem.system_conf_to_pose(
            name="conf_to_pose",
            context_id=context_id,
            goal=frame_relative_pose,
            start=SystemConf(data["start"]),
            linear=False,
        )
    else:
        # Create system conf objects for start and goal
        start_conf = SystemConf(data["start"])
        goal_conf = SystemConf(data["goal"])

        # Create the planning problem from system conf to system conf
        problem = PlanningProblem.system_conf_to_system_conf(
            name="conf_to_conf", context_id=context_id, goal=goal_conf, start=start_conf
        )

    return problem
