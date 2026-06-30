import os
import sys
from pathlib import Path

import bazel_build as bazel
from hatchling.builders.hooks.plugin.interface import BuildHookInterface

# Bazel target for the pybind native library
_NATIVE_BAZEL_TARGET = (
    "//client/python/planning_service_client/src/planning_service_client/native:_planning_service_client"
)
# bazel-bin relative output path (Bazel always produces this exact name)
_NATIVE_DIR = "client/python/planning_service_client/src/planning_service_client/native"
_NATIVE_OUTPUT = f"{_NATIVE_DIR}/_planning_service_client.so"
#
_NATIVE_DEST = "planning_service_client/native"


class BuildHook(BuildHookInterface):
    PLUGIN_NAME = "custom"

    def _find_so(self, bazel_bin: Path) -> "Path | None":
        so = bazel_bin / _NATIVE_OUTPUT
        return so if so.exists() else None

    def _native_is_stale(self, workspace_root: Path, so_file: Path) -> bool:
        """
        Return True if any source file is newer than *so_file*.
        """
        so_mtime = so_file.stat().st_mtime
        source_globs = [
            "client/cpp/**/*.h",
            "client/cpp/**/*.cc",
            f"{_NATIVE_DIR}/src/**/*.h",
            f"{_NATIVE_DIR}/src/**/*.cc",
            f"{_NATIVE_DIR}/**/*.py",
            f"{_NATIVE_DIR}/BUILD.bazel",
            "client/cpp/**/BUILD.bazel",
        ]
        for pattern in source_globs:
            for path in workspace_root.glob(pattern):
                if path.is_file() and path.stat().st_mtime > so_mtime:
                    print(f"Source file {path} is newer than {so_file.name}, marking native library as stale.")
                    return True
        return False

    def initialize(self, version: str, build_data: dict) -> None:
        super().initialize(version, build_data)

        if os.environ.get("SKIP_NATIVE_BUILD", "0").lower() in ("1", "true"):
            print("Skipping native build (SKIP_NATIVE_BUILD set).")
            return

        force = os.environ.get("FORCE_NATIVE_BUILD", "0").lower() in ("1", "true")

        workspace_root = bazel.find_workspace_root(Path(self.root))
        bazel_bin = workspace_root / "bazel-bin"

        so_file = self._find_so(bazel_bin)
        if so_file and not force:
            if not self._native_is_stale(workspace_root, so_file):
                print(f"{so_file.name} is up to date, skipping Bazel build. Set FORCE_NATIVE_BUILD=1 to force.")
                build_data["force_include"][str(so_file)] = f"{_NATIVE_DEST}/{so_file.name}"
                return
            print(f"{so_file.name} is stale, rebuilding.")

        py_version = f"{sys.version_info.major}.{sys.version_info.minor}"
        bazel_flags = [
            f"--@rules_python//python/config_settings:python_version={py_version}",
        ]
        if os.environ.get("CI"):
            bazel_flags.append("--config=ci")
        bazel.run_bazel_build(_NATIVE_BAZEL_TARGET, workspace_root, flags=bazel_flags)

        so_file = self._find_so(bazel_bin)
        if so_file is None:
            raise RuntimeError(f"Bazel build succeeded but '{bazel_bin / _NATIVE_OUTPUT}' not found.")

        print(f"Injecting {so_file.name} -> {_NATIVE_DEST}/{so_file.name}")
        build_data["force_include"][str(so_file)] = f"{_NATIVE_DEST}/{so_file.name}"
