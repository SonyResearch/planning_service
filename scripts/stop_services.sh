#!/bin/bash

# Script to stop one or more of the supported services using a dev or production configuration.
set -eufo pipefail

usage="$(basename "$0") [--help] [--dev / --prod] [OPTIONS] [SERVICES]... -- Convenience
script to stop one or more services.

Multiple services can be stopped in a single invocation of the script. If no services are
specified, all services will be brought down.

where:
    --help            Show this help text
    --remove          Remove the container(s) after they've been stopped
    --iris            Stop the IRIS builder
    --planner         Stop the motion planner
    --viz             Stop the visualizer"

STOP_CMD=stop
SERVICES=()
while (($#)); do
  case $1 in
  --help)
    echo "$usage"
    exit
    ;;
  --remove)
    STOP_CMD=down
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

export USER_UID=${UID}
docker compose -f docker-compose.production.yml -f docker-compose.dev.yml $STOP_CMD "${SERVICES[@]}"
