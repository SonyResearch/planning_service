# Overview
Proto generator for use with hatchling build system.

# Usage
Include something like below in your
```
[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[tool.uv.sources]
generate_protos_builder = { path = "../generate_protos_builder" }

[tool.hatch.build.targets.wheel.hooks.generate-protos]
dependencies = ["generate_protos_builder"]
proto_generated_path="src/proto"
buf_gen_py_yaml_path="../../buf.gen.py.yaml"
proto_root_path="../.."
```
Which sets the build system to hatchling, includes the path to this source, and finally configures the plugin to run.
You will need to have a buf.gen.yml file setup with code for python_proto_6 and python_proto_5 so this plugin can get the appropiate library version to use.

### Configuration
proto_generated_path -> Path the generated source code (Warning this will be cleaned as part of the build!)
buf_gen_py_yaml_path -> Path to the buf.gen file
proto_root_path -> Root of proto project (this is the cwd of where the buf cli will be executed from)
