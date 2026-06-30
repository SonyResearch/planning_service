import importlib
import os
import pathlib
import platform
import shutil
import stat
import subprocess
import tempfile
from pathlib import Path

import requests

BUF_VERSION = "1.50.0"
BUF_URL = (
    f"https://github.com/bufbuild/buf/releases/download/v{BUF_VERSION}/buf-{platform.system()}-{platform.machine()}"
)


class ProtoGenerator:
    """
    Class to handle the generation of protobuf files using `buf`.
    """

    def __init__(self, proto_generated_path: str, buf_gen_py_yaml_path: str, proto_root_path: str):
        self.proto_generated_path = Path(proto_generated_path)

        self.buf_gen_py_yaml_path = Path(buf_gen_py_yaml_path)
        self.proto_root_path = proto_root_path

    def _generate_proto_files(self, buf_path: str) -> None:
        """
        Generate the Python files from the proto files with `buf`.

        Args:
            buf_path (str): The path to the `buf` binary.

        Raises:
            RuntimeError: If the generation command fails.
        """
        build_protobuf_command = [
            buf_path,
            "generate",
            "--clean",
            "--template",
            self.buf_gen_py_yaml_path,
        ]

        try:
            # The buf --clean option deletes the output directory for some terrible reason; so create it.
            os.makedirs(
                self.proto_generated_path,
                exist_ok=True,
            )
            pathlib.Path(self.proto_generated_path / "__init__.py").touch()
            subprocess.run(
                build_protobuf_command,
                check=True,
                capture_output=True,
                cwd=self.proto_root_path,
            )
        except subprocess.CalledProcessError as e:
            if isinstance(e.cmd, list):
                all_string_message = []
                for a in e.cmd:
                    if isinstance(a, str):
                        all_string_message.append(a)
                    else:
                        all_string_message.append(str(a))
                command_msg = " ".join(all_string_message)
            else:
                command_msg = e.cmd

            raise RuntimeError(
                f"Generation command run in cwd: {self.proto_root_path} failed: {command_msg}. "
                f"Exception: {e.stderr.decode()}"
            )
        # Now copy the appropriate generated files to the appropriate location to
        # account for version 5 and 6 of protobuf
        # This is defined in the `buf.gen.py.yaml` file, which is used to generate the proto files.
        protobuf_version = "5"  # Default to version 5
        try:
            protobuf_version = importlib.metadata.version("protobuf")
        except importlib.metadata.PackageNotFoundError as e:
            print(f"Error occurred while fetching protobuf version: {e}")
        protobuf_version = os.environ.get("PROTOBUF_VERSION", protobuf_version)
        if protobuf_version.startswith("6.") or protobuf_version == "6":
            versioned_path = _replace_last(str(self.proto_generated_path), "python", "python_proto_6")
        elif protobuf_version.startswith("5.") or protobuf_version == "5":
            versioned_path = _replace_last(str(self.proto_generated_path), "python", "python_proto_5")
        else:
            raise RuntimeError(f"Unsupported protobuf version: {protobuf_version}. Expected 5.x or 6.x.")

        shutil.copytree(versioned_path, self.proto_generated_path, dirs_exist_ok=True)

    def build_package(self):
        """
        Build the package from the proto files.

        Generate the proto files with `buf` (downloading a temporary binary if necessary) and populate the resultant
        package tree with the necessary `__init__.py` files.

        Raises:
            RuntimeError: If the `buf` binary cannot be found or downloaded.
        """
        buf_path = shutil.which("buf")
        if buf_path is not None:
            print(f"Generating proto files with buf binary at {buf_path}")
            self._generate_proto_files(buf_path)
        else:
            print("Could not find buf locally; attempting to download")
            r = requests.get(BUF_URL, allow_redirects=True, timeout=20)
            if r.status_code != 200:
                raise RuntimeError(f"Failed to download buf from {BUF_URL}. Status code: {r.status_code}")
            with tempfile.TemporaryDirectory() as tmpdir:
                buf_path = Path(tmpdir) / "buf"
                with buf_path.open("wb") as f:
                    f.write(r.content)
                print(f"Wrote buf binary to {buf_path}")
                os.chmod(buf_path, os.stat(buf_path).st_mode | stat.S_IEXEC)
                self._generate_proto_files(buf_path)
        # Create __init__.py files in the generated proto folders
        for entry in os.walk(self.proto_generated_path):
            path = Path(entry[0])
            if path.is_dir():
                (path / "__init__.py").touch()


def _replace_last(source: str, old, new):
    res_list = source.rsplit(old, 1)
    return new.join(res_list)
