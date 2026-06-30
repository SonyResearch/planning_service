#!/bin/bash
set -eufo pipefail

PROJECT_ROOT="$(dirname "$(dirname "$(realpath "$0")")")"
cd "${PROJECT_ROOT}"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

wait_for_service() {
  local name="$1"
  local host="${2%%:*}"
  local port="${2##*:}"
  local timeout_s="${3:-120}"
  local interval_s="${4:-2}"
  local attempts=$(( timeout_s / interval_s ))
  echo "  Waiting for ${name} at ${host}:${port} (up to ${timeout_s}s)..."
  for i in $(seq 1 "${attempts}"); do
    if (echo > /dev/tcp/"${host}"/"${port}") 2>/dev/null; then
      echo "  ${name} is up."
      return 0
    fi
    sleep "${interval_s}"
  done
  echo "ERROR: ${name} did not become reachable within ${timeout_s}s." >&2
  return 1
}

cleanup() {
  local exit_code=$?
  echo ""
  if [[ "${exit_code}" -ne 0 ]]; then
    echo "========== Visualizer container logs (test failed) ======================"
    docker logs visualizer-dev 2>&1 || true
    echo "========================================================================="
  fi
  echo "========== Stopping visualizer container ================================="
  export USER_UID=${UID}
  docker compose \
    -f docker-compose.dev.yml \
    -f docker-compose.system-test.yml \
    down visualizer || true
  exit "${exit_code}"
}
trap cleanup EXIT

# ---------------------------------------------------------------------------
# 1. Pre-build all test binaries and the visualizer binary BEFORE starting the
#    service.
# ---------------------------------------------------------------------------
echo "========== Pre-building visualizer and test binaries ======================"
export USER_UID=${UID}
# The visualizer container runs bazel with --output_base=output_base_visualizer
# (see docker_start.sh).  We must pre-build using the same output base so the
# container finds the cached artifacts and skips the full rebuild.
./bazelw --startup-opt --output_base=/root/.cache/bazel/output_base_visualizer \
  build //planning_service/app:visualizer
./bazelw --system-test build \
  //system_tests:cpp_visualizer_system_test \
  //system_tests:py_visualizer_system_test

# ---------------------------------------------------------------------------
# 2. Start visualizer (lifty, dev build, default_init=true)
#    docker-compose.system-test.yml overrides SYSTEM_NAME=lifty and sets
#    DEFAULT_INIT=true so the service loads data/lifty/visualizer_options.yaml
#    on startup with no explicit StartVisualizer call required.
# ---------------------------------------------------------------------------
echo "========== Starting visualizer (lifty / dev) =============================="
# Remove any leftover container from a previous interrupted run before starting.
docker rm -f visualizer-dev 2>/dev/null || true
docker compose \
  -f docker-compose.dev.yml \
  -f docker-compose.system-test.yml \
  up -d visualizer

wait_for_service "visualizer" "127.0.0.1:5550"

# ---------------------------------------------------------------------------
# 3. C++ client test
# ---------------------------------------------------------------------------
echo "========== Running C++ visualizer system test ============================="
./bazelw --system-test test //system_tests:cpp_visualizer_system_test \
  --test_env=VIZ_ADDR=visualizer-dev:5550

echo ""
echo "========== All visualizer system tests passed! ============================"

# ---------------------------------------------------------------------------
# 4. Python client tests
# ---------------------------------------------------------------------------
echo "========== Running Python visualizer system tests ========================="
./bazelw --system-test test //system_tests:py_visualizer_system_test \
  --test_env=VIZ_ADDR=visualizer-dev:5550
