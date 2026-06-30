# Planning Service Client

This repository is home to the [gRPC](https://grpc.io) API for interacting with the Planning Service, as well as language-specific client implementations for those API endpoints. Concretely, the Planning Service is comprised of:
- a motion planner, which can create smooth motion plans for a given robot system,
- an artifact builder, which generates the underlying planning artifacts used by the planner at runtime, and
- a visualizer, which displays the robot system and its associated planning artifacts for inspection.

We currently support client implementations in C++ and Python. These implementations are always subject to change.

---

## C++

### Bazel

For users of the Bazel build system, add this repository as a dependency and import targets into your project. For example:

- Full client suite: `@planning_service_client//client/cpp/planning_service_client/api:api`
- Motion planner client only: `@planning_service_client//client/cpp/planning_service_client/api:planner_client`
- Raw C++ Protobuf generated code: `@planning_service_client//proto:planner_cpp_proto`

All available targets are defined in [client/cpp/planning_service_client/api/BUILD.bazel](https://github.com/SonyResearch/planning_service_client/blob/main/client/cpp/planning_service_client/api/BUILD.bazel) and [proto/BUILD.bazel](https://github.com/SonyResearch/planning_service_client/blob/main/proto/BUILD.bazel).

### CMake

> **_NOTE:_** CMake support requires this repository to be added as a submodule.

To include the C++ client library in your project, add the following to your `CMakeLists.txt`:
```cmake
add_subdirectory(path/to/planning_service_client)
target_link_libraries(your_target PRIVATE planning_service_client)
```

---

## Python

The Python client is distributed as the `planning-service-client` package and managed with [**uv**](https://docs.astral.sh/uv/).

### Install from source

From `client/python/planning_service_client/`:
```bash
uv pip install .
```

With optional notebook dependencies (Jupyter + ipykernel):
```bash
uv pip install ".[notebook]"
```

### Build a wheel

```bash
uv build
```

### Bazel

Build a redistributable wheel via Bazel:
```bash
bazel build //client/python/planning_service_client:planning_service_client_wheel.dist
```

Install the resulting wheel:
```bash
pip install --upgrade \
    bazel-bin/client/python/planning_service_client/planning_service_client_wheel_dist/*.whl
```

See [client/python/planning_service_client/README.md](client/python/planning_service_client/README.md) for full Python installation details.
