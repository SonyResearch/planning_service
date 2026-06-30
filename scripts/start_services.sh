#!/bin/bash

# Script to start one or more of the supported services using a dev or production configuration.
set -eufo pipefail

usage="$(basename "$0") [--help] [--dev] [OPTIONS] [SERVICES]... -- Convenience
script to start one or more services using either a dev or production build.

Multiple services can be started in a single invocation of the script - for example,
to start the IRIS builder and visualizer using a local dev build, run:
\`./scripts/start_services.sh --dev --iris --viz\`.

where:
    --help            Show this help text
    --dev             Use the development docker-compose configuration
    --attach          Attach to the running containers (all services are started
                      in \`--detached\` mode by default)
    --iris            Start the IRIS builder
    --planner         Start the motion planner
    --viz             Start the visualizer"
PROJECT_ROOT="$(dirname $(dirname $(realpath $0)))"

COMPOSE_FILE="${PROJECT_ROOT}/docker-compose.production.yml"
SERVICES=()
DETACHED_ARG="-d"
while (($#)); do
  case $1 in
  --help)
    echo "$usage"
    exit
    ;;
  --dev)
    COMPOSE_FILE="${PROJECT_ROOT}/docker-compose.dev.yml"
    shift 1
    ;;
  --attach)
    DETACHED_ARG=""
    shift 1
    ;;
  --iris)
    SERVICES+=(iris_builder)
    shift 1
    ;;
  --planner)
    SERVICES+=(motion_planner)
    shift 1
    ;;
  --viz)
    SERVICES+=(visualizer)
    shift 1
    ;;
  * | -* | --*=) # unsupported options
    echo "Error: Unsupported option $1" >&2
    exit 1
    ;;
  esac
done

if [[ ${#SERVICES[@]} -eq 0 ]]; then
  echo "No services have been specified! Exiting."
  exit 1
fi

if [[ ! -d "${PROJECT_ROOT}/logs" ]]; then
  echo "Creating logs directory at ${PROJECT_ROOT}/logs"
  mkdir "${PROJECT_ROOT}/logs"
fi

export USER_UID=${UID}
docker compose -f ${COMPOSE_FILE} up ${DETACHED_ARG} --force-recreate "${SERVICES[@]}"
