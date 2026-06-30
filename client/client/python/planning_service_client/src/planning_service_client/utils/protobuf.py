from pathlib import Path

import proto.basic_types_pb2 as pb


def make_meshfile(path: Path, parent_path: Path, package_name: str, mesh_type: str) -> pb.MeshFile:
    """
    Internal. Create a Meshfile message for a mesh at the specified path.

    Args:
        path (Path): Local path to the meshfile.
        mesh_type (MeshType): The mesh's type (i.e., collision or visual).

    Returns:
        MeshFile: Populated message.
    """
    mesh_types = ("collision", "visual")
    if mesh_type not in mesh_types:
        raise RuntimeError(f"Invalid mesh type: {mesh_type}. Must be one of: {mesh_types}.")
    format = "MESH_FORMAT_" + path.suffix.lstrip(".").upper()
    type = "MESH_TYPE_" + mesh_type.upper()
    with path.open("rb") as f:
        content = f.read()
    return pb.MeshFile(
        name=path.stem,
        package_name=package_name,
        parent_path=str(parent_path),
        content=content,
        format=pb.MeshFormat.Value(format),
        type=pb.MeshType.Value(type),
    )


def make_modelfile(path: Path, parent_path: Path, package_name: str) -> pb.ModelFile:
    """
    Internal. Create a Modelfile message for a model geometry file (e.g., a URDF) at the specified path.

    Args:
        path (Path): Local path to the file.ß

    Returns:
        ModelFile: Populated message.
    """
    format = "MODEL_FORMAT_" + path.suffix.lstrip(".").upper()
    with path.open("rb") as f:
        content = f.read()
    return pb.ModelFile(
        name=path.stem,
        package_name=package_name,
        parent_path=str(parent_path),
        content=content,
        format=pb.ModelFormat.Value(format),
    )
