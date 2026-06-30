#!/bin/bash
set -eufo pipefail

# Run unit tests for the planning service. Optionally run with --coverage to generate a coverage report.

USE_COVERAGE=false
TEST_CMD="test"
# Break up targets and options
TARGETS=()
OPTIONS=()
while (($#)); do
  case "$1" in
  -h | --help)
    echo "Usage: $0 [--coverage] [--verbosity N] [OPTIONS...] [TARGETS...]"
    echo "Run unit tests for the planning service. Use --coverage to generate a coverage report."
    echo ""
    echo "TARGETS: The Bazel targets to test (default: //planning_service/...)"
    echo "OPTIONS: Additional options to pass to Bazel"
    echo "--verbosity N: Set spdlog verbosity (0=critical .. 5=trace, default: 3=info)"
    exit 0
    ;;
  --verbosity)
    OPTIONS+=("--test_env=VERBOSITY=$2")
    shift 2
    ;;
  --coverage)
    USE_COVERAGE=true
    TEST_CMD="coverage --combined_report=lcov"
    shift 1
    ;;
  //*)
    TARGETS+=("$1")
    shift 1
    ;;
  * | -* | --*=)
    OPTIONS+=("$1")
    shift 1
    ;;
    esac
done
# Default target if none specified
if [[ ${#TARGETS[@]} -eq 0 ]]; then
  TARGETS=(//planning_service/... //shokunin/...)
fi

# Build tests with fastbuild to get quickest results
./bazelw ${TEST_CMD} -c fastbuild --test_output=errors --verbose_failures --sandbox_debug \
  "${OPTIONS[@]+"${OPTIONS[@]}"}" "${TARGETS[@]}"

if [[ "$USE_COVERAGE" == "true" ]]; then
  export USER_UID=$UID
  docker compose -f docker-compose.build.yml run --rm generate_coverage
fi
