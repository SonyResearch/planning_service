import os

from generate_protos_builder.proto_generator import ProtoGenerator
from hatchling.builders.hooks.plugin.interface import BuildHookInterface

CONFIG_PROTO_GENERATED_PATH = "proto_generated_path"
CONFIG_BUF_GEN_PY_YAML_PATH = "buf_gen_py_yaml_path"
CONFIG_PROTO_ROOT_PATH = "proto_root_path"


class GenerateProtos(BuildHookInterface):
    """
    Custom build hook to ensure the package is built with the correct Python version.
    """

    PLUGIN_NAME = "generate-protos"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if CONFIG_PROTO_GENERATED_PATH not in self.config:
            raise RuntimeError(
                f"Configuration key '{CONFIG_PROTO_GENERATED_PATH}' is missing in the build hook configuration."
            )
        if CONFIG_BUF_GEN_PY_YAML_PATH not in self.config:
            raise RuntimeError(
                f"Configuration key '{CONFIG_BUF_GEN_PY_YAML_PATH}' is missing in the build hook configuration."
            )
        if CONFIG_PROTO_ROOT_PATH not in self.config:
            raise RuntimeError(
                f"Configuration key '{CONFIG_PROTO_ROOT_PATH}' is missing in the build hook configuration."
            )

    def initialize(self, version: str, build_data) -> None:
        """
        Initialize the build hook with the specified Python version.
        """
        super().initialize(version, build_data)
        skip_proto_flag = os.environ.get("SKIP_PROTO_BUILD", "0").lower()
        if skip_proto_flag == "1" or skip_proto_flag == "true":
            print(f"Skipping proto generation as SKIP_PROTO_BUILD is set to {skip_proto_flag}.")
            return

        generator = ProtoGenerator(
            proto_generated_path=os.path.join(self.root, self.config[CONFIG_PROTO_GENERATED_PATH]),
            buf_gen_py_yaml_path=os.path.join(self.root, self.config[CONFIG_BUF_GEN_PY_YAML_PATH]),
            proto_root_path=os.path.join(self.root, self.config[CONFIG_PROTO_ROOT_PATH]),
        )
        generator.build_package()
