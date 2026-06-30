# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "planning-service-proto",
#   "planning-service-client",
# ]
#
# [tool.uv.sources]
# planning-service-proto = { path = "../../gen/python" }
# planning-service-client = { path = "../../client/python/planning_service_client" }
# ///
import importlib
import sys
from pathlib import Path
from typing import List, Optional

"""
Validate that all expected modules from .proto files are present in the installed package.
"""


def _expected_modules(pkg_root: Path, ext: str, suffixes: Optional[List[str]] = None):
    """
    Yield fully-qualified module names expected from every .proto in the source tree.
    """
    for file in sorted(pkg_root.rglob(f"*.{ext}")):
        if file.stem.startswith("_"):
            continue
        rel = file.relative_to(pkg_root.parent)  # e.g. proto/calibration/v1/camera.proto
        stem = rel.with_suffix("")  # proto/calibration/v1/camera
        base = str(stem).replace("/", ".")  # proto.calibration.v1.camera
        if suffixes:
            for suffix in suffixes:
                yield f"{base}_{suffix}"
        # just yield the base module
        else:
            yield base


def validate_package(pkg_root: Path, ext: str, suffixes: Optional[List[str]] = None):
    pkg_name = pkg_root.name
    try:
        # import the top level
        importlib.import_module(pkg_name)
    except ModuleNotFoundError:
        print(f"Module '{pkg_name}' not found! Please make sure to install the package with `pip` before running.")
        sys.exit(1)

    expected = list(_expected_modules(pkg_root, ext, suffixes))
    if not expected:
        print(f"No expected modules found for '{pkg_name}' in {pkg_root} — check that the path is correct.")
        sys.exit(1)
    missing = []
    for m in expected:
        try:
            importlib.import_module(m)
        except (ModuleNotFoundError, ImportError) as e:
            missing.append(f"{m} ({e})")
    if missing:
        print(f"{len(missing)}/{len(expected)} expected module(s) are missing from the installed package:")
        for m in missing:
            print(f"\t- {m}")
        sys.exit(1)
    print(f"Imported all {len(expected)} expected '{pkg_name}' modules successfully.")


if __name__ == "__main__":
    project_root = next(p for p in Path(__file__).resolve().parents if (p / "MODULE.bazel").exists())
    proto_root = project_root / "proto"
    client_root = project_root / "client" / "python" / "planning_service_client" / "src" / "planning_service_client"
    # Import protos
    validate_package(proto_root, "proto", suffixes=["pb2", "pb2_grpc"])
    # Import client
    validate_package(client_root, "py")
