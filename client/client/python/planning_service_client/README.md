# planning-service-client (Python)

Python client for the Planning Service, exposing the planner, artifact builder, and visualizer APIs via a pybind11 native extension.

## Installation

The package is built and managed with [**uv**](https://docs.astral.sh/uv/). All commands below should be run from this directory (`client/python/planning_service_client/`).

### Install `uv` (if not already present)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Install directly into a virtual environment

```bash
uv pip install .
```

With the optional notebook dependencies (Jupyter + ipykernel):

```bash
uv pip install ".[notebook]"
```

If the native extension is stale or you're seeing pybind11-related errors, force a rebuild:

```bash
FORCE_NATIVE_BUILD=1 uv pip install .
```

### Build a wheel

To produce a redistributable `.whl` (output goes to `dist/`):

```bash
uv build
```

Install the resulting wheel elsewhere:

```bash
uv pip install dist/planning_service_client-*.whl
```

## Bazel

Build the wheel via Bazel:

```bash
bazel build //client/python/planning_service_client:planning_service_client_wheel.dist
```

Install it locally:

```bash
pip install --upgrade \
    bazel-bin/client/python/planning_service_client/planning_service_client_wheel_dist/*.whl
```

Export to a `dist/` folder (useful for CI or Docker):

```bash
mkdir -p dist
cp -v bazel-bin/client/python/planning_service_client/planning_service_client_wheel_dist/*.whl dist/
```
