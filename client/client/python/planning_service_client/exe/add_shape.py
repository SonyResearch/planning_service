from __future__ import annotations

import click
import numpy as np

from planning_service_client.native.types import Box, Capsule, Cylinder, Rgba, ShapeInFrame, Sphere
from planning_service_client.native.visualizer import VisualizerClient

_COLORS: dict[str, Rgba] = {
    "white": Rgba.white(),
    "black": Rgba.black(),
    "red": Rgba.red(),
    "green": Rgba.green(),
    "blue": Rgba.blue(),
}

_COMMON_OPTIONS = [
    click.option("--addr", default="localhost:5550", show_default=True, help="Visualizer server address."),
    click.option("--path", required=True, help="Scene-tree path for the object."),
    click.option(
        "--color",
        default="white",
        show_default=True,
        type=click.Choice(list(_COLORS)),
        help="Object color.",
    ),
    click.option("--frame", default="", show_default=True, help="Reference frame name."),
    click.option("--x", default=0.0, show_default=True, type=float, help="Translation x."),
    click.option("--y", default=0.0, show_default=True, type=float, help="Translation y."),
    click.option("--z", default=0.0, show_default=True, type=float, help="Translation z."),
    click.option("--qw", default=1.0, show_default=True, type=float, help="Quaternion w (real part)."),
    click.option("--qx", default=0.0, show_default=True, type=float, help="Quaternion x."),
    click.option("--qy", default=0.0, show_default=True, type=float, help="Quaternion y."),
    click.option("--qz", default=0.0, show_default=True, type=float, help="Quaternion z."),
]


def _common_options(f):
    for option in reversed(_COMMON_OPTIONS):
        f = option(f)
    return f


def _apply_pose(
    shape_in_frame: ShapeInFrame,
    frame: str,
    x: float,
    y: float,
    z: float,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
) -> None:
    if frame:
        shape_in_frame.set_frame(frame)
    shape_in_frame.set_translation(np.array([x, y, z]))
    shape_in_frame.set_quaternion(np.array([qw, qx, qy, qz]))


def _set_object(
    addr: str,
    path: str,
    color: str,
    shape_in_frame: ShapeInFrame,
    frame: str,
    x: float,
    y: float,
    z: float,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
) -> None:
    _apply_pose(shape_in_frame, frame, x, y, z, qw, qx, qy, qz)
    client = VisualizerClient(addr=addr, client_id="add_shape_client")
    client.connect()
    client.set_object(path=path, shape_in_frame=shape_in_frame, color=_COLORS[color])


@click.group()
def cli() -> None:
    """
    Add or delete shapes in the planning service visualizer.
    """


@cli.command()
@_common_options
@click.option("--width", default=1.0, show_default=True, type=float, help="Box width.")
@click.option("--depth", default=1.0, show_default=True, type=float, help="Box depth.")
@click.option("--height", default=1.0, show_default=True, type=float, help="Box height.")
def box(
    addr: str,
    path: str,
    color: str,
    width: float,
    depth: float,
    height: float,
    frame: str,
    x: float,
    y: float,
    z: float,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
) -> None:
    """
    Add a box.
    """
    _set_object(addr, path, color, ShapeInFrame(Box(width, depth, height)), frame, x, y, z, qw, qx, qy, qz)


@cli.command()
@_common_options
@click.option("--radius", default=1.0, show_default=True, type=float, help="Sphere radius.")
def sphere(
    addr: str,
    path: str,
    color: str,
    radius: float,
    frame: str,
    x: float,
    y: float,
    z: float,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
) -> None:
    """
    Add a sphere.
    """
    _set_object(addr, path, color, ShapeInFrame(Sphere(radius)), frame, x, y, z, qw, qx, qy, qz)


@cli.command()
@_common_options
@click.option("--radius", default=0.5, show_default=True, type=float, help="Cylinder radius.")
@click.option("--height", default=1.0, show_default=True, type=float, help="Cylinder height.")
def cylinder(
    addr: str,
    path: str,
    color: str,
    radius: float,
    height: float,
    frame: str,
    x: float,
    y: float,
    z: float,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
) -> None:
    """
    Add a cylinder.
    """
    _set_object(addr, path, color, ShapeInFrame(Cylinder(radius, height)), frame, x, y, z, qw, qx, qy, qz)


@cli.command()
@_common_options
@click.option("--radius", default=0.5, show_default=True, type=float, help="Capsule radius.")
@click.option("--height", default=1.0, show_default=True, type=float, help="Capsule height.")
def capsule(
    addr: str,
    path: str,
    color: str,
    radius: float,
    height: float,
    frame: str,
    x: float,
    y: float,
    z: float,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
) -> None:
    """
    Add a capsule.
    """
    _set_object(addr, path, color, ShapeInFrame(Capsule(radius, height)), frame, x, y, z, qw, qx, qy, qz)


@cli.command(name="delete")
@click.option("--addr", default="localhost:5550", show_default=True, help="Visualizer server address.")
@click.option("--path", required=True, help="Scene-tree path of the object to delete.")
def delete_cmd(addr: str, path: str) -> None:
    """
    Delete an object from the visualizer.
    """
    client = VisualizerClient(addr=addr, client_id="add_shape_client")
    client.connect()
    client.delete_object(path=path)


if __name__ == "__main__":
    cli()
