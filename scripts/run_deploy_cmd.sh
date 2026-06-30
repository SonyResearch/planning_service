#!/bin/bash
set -eufo pipefail

docker compose -f docker-compose.production.yml run --rm -it motion_planner "$@"
